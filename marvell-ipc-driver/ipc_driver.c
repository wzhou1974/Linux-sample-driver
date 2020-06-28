#define DEBUG 1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include "ipc_api.h"

MODULE_LICENSE("Dual BSD/GPL");

#define IPC_NAME "ipc"
#define IPC_COMPATIBILITY_NAME "mrvl,IPC"

#define PREFIX IPC_NAME ": "

#define ENTER() pr_debug(PREFIX "ENTER %s\n", __FUNCTION__)
#define EXIT()  pr_debug(PREFIX "EXIT  %s:%d\n", __FUNCTION__, __LINE__)

#define IIR_ACK_SHIFT ( 9 )
#define IIR_ACK_MASK  ( 0x03 << IIR_ACK_SHIFT )

#define ACK_MSG_PROCESSED ( 0x3 )
#define ACK_MSG_DISCARDED ( 0x2 )

#define IIR_CMD_SHIFT ( 8 )
#define IIR_CMD_MASK  ( 1 << IIR_CMD_SHIFT )

#define IIR_PORT_SHIFT ( 0 )
#define IIR_PORT_MASK  ( 0xFF << IIR_PORT_SHIFT )

typedef struct IPC0_REGS_s
{
  volatile uint32_t IPC_ISRR;  ///< 0x0 [R]: IPC_ISRR
  volatile uint32_t IPC_WDR_0;  ///< 0x4 [W]: Write Data Register 0
  volatile uint32_t IPC_WDR_1;  ///< 0x8 [W]: Write Data Register 1
  volatile uint32_t IPC_ISRW;  ///< 0xc [W]: Interrupt Set Register Write
  volatile uint32_t IPC_ICR;  ///< 0x10 [W]: Interrupt Clear Register
  volatile uint32_t IPC_IIR;  ///< 0x14 [R]: Interrupt Identification Register
  volatile uint32_t IPC_RDR_0;  ///< 0x18 [R]: Read Data Register 0
  volatile uint32_t IPC_RDR_1;  ///< 0x1c [R]: Read Data Register 1
  volatile uint32_t IPC_MAJ_MID_REV;  ///< 0x20 [R]: Revision (Major and Mid) Register
  volatile uint32_t IPC_CFG_REV;  ///< 0x24 [R]: Revision (Configuration) Register
  volatile uint32_t IPC_DUMMY;  ///< 0x28 [W]: Dummy Register
} IPC0_REGS_t;

/* end of header file */

struct ipc_port_config_s;

typedef struct
{
    struct platform_device *pdev;
    char         dev_name[21];
    uint32_t     instance_id;
    IPC0_REGS_t *regs;
    uint32_t     int_num;
    int          open_count;
    struct ipc_port_config_s *open_ports;
    struct semaphore tx_ready_sem;
    struct semaphore tx_done_sem;
    uint8_t      ack_type;
} ipc_device_config_t;

typedef struct ipc_port_config_s
{
    struct list_head     list;
    ipc_device_config_t *ipc_device;
    uint8_t              port_number;
    ipc_recv_callback    recv_callback;
    void                *user_param;
} ipc_port_config_t;

static int num_ipc_devices = 0;
static ipc_device_config_t *ipc_devices;

static struct semaphore list_sem;
static struct workqueue_struct *ipc_workqueue;

typedef struct
{
    struct work_struct delayed_work;
    ipc_device_config_t *device;
    uint8_t   port_number;
    uint8_t   cmd;
    uint16_t  len;
    void     *buffer;
} recv_data_t;

static recv_data_t recv_data;

uint32_t ipc_get_num_devices( void )
{
    return num_ipc_devices;
}
EXPORT_SYMBOL(ipc_get_num_devices);

const char *ipc_get_device_name( uint32_t device_index )
{
    if (device_index < ipc_get_num_devices() )
    {
        return ipc_devices[device_index].dev_name;
    }
    return NULL;
}
EXPORT_SYMBOL(ipc_get_device_name);

static bool device_is_valid( ipc_device_config_t *device )
{
    if ( ( device != NULL ) &&
         ( device->instance_id < ipc_get_num_devices() ) &&
         ( &ipc_devices[device->instance_id] == device ) 
       )
    {
        return true;
    }
    return false;
}

static bool port_is_valid( ipc_port_config_t *port )
{
    if ( ( port != NULL ) && 
         ( port->ipc_device != NULL )
       )
    {
        return device_is_valid( port->ipc_device );
    }
    return false;
}

static ipc_port_config_t *find_device_port( ipc_device_config_t *device, uint8_t port_number )
{
    ipc_port_config_t *port = NULL;
    ipc_port_config_t *temp;

    ENTER();

    if ( !device_is_valid( device ) )
    {
        return NULL;
    }

    down( &list_sem );

    list_for_each_entry( temp, &device->open_ports->list, list )
    {
        if ( temp->port_number == port_number )
        {
            pr_debug("find_device_port found %d:%d\n", device->instance_id, port_number);
            port = temp;
            break;
        }
        else
        {
            pr_debug("(nonmatching device %d:%d)\n", device->instance_id, temp->port_number);
        }
    }

    up( &list_sem );

    EXIT();

    return port;
}

static void non_isr_recv( struct work_struct *work)
{
    recv_data_t *data = container_of( work, recv_data_t, delayed_work );
    uint8_t ack_type = ACK_MSG_DISCARDED;

    ENTER();

    if ( device_is_valid( data->device ) )
    {
        ipc_port_config_t *port;

        port = find_device_port(data->device, data->port_number);

        if ( port_is_valid(port) )
        {
            void *buffer_va = NULL;
            pr_debug("Port %d, rx cmd %d, buffer 0x%p, len %d\n", data->port_number, data->cmd, data->buffer, data->len);

            ack_type = ACK_MSG_PROCESSED;

            if ((data->buffer != NULL) && (data->len > 0))
            {
                request_mem_region((uint32_t)data->buffer, data->len, IPC_NAME);
                buffer_va = ioremap((uint32_t)data->buffer, data->len);

                port->recv_callback(port, port->user_param, data->cmd, buffer_va, data->len);

                iounmap(buffer_va); 
                release_mem_region((uint32_t)data->buffer, data->len);
            }
            else
            {
                port->recv_callback(port, port->user_param, data->cmd, data->buffer, data->len);
            }
        }
        else
        {
            ack_type = ACK_MSG_DISCARDED;
            pr_debug("<CLOSED> Port %d, rx cmd %d, buffer 0x%p, len %d\n", data->port_number, data->cmd, data->buffer, data->len);
        }
    }

    iowrite32( ( ( uint32_t )ack_type ) << IIR_ACK_SHIFT, &data->device->regs->IPC_ISRW);

    EXIT();
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
    uint32_t iir;
    ipc_device_config_t *device = ( ipc_device_config_t * )dev_id;

    iowrite32(0, &device->regs->IPC_DUMMY);
    iir = ioread32(&device->regs->IPC_IIR);

    if ( iir & IIR_ACK_MASK )
    {
        device->ack_type = ( iir & IIR_ACK_MASK ) >> IIR_ACK_SHIFT;

        up(&device->tx_done_sem);

        iowrite32(IIR_ACK_MASK, &device->regs->IPC_ICR);
    }
    if ( iir & IIR_CMD_MASK )
    {
        uint32_t  p1, p2;
        int ret;

        p1 = ioread32(&device->regs->IPC_RDR_0);
        p2 = ioread32(&device->regs->IPC_RDR_1);

        memset(&recv_data, 0, sizeof(recv_data));
        INIT_WORK( &recv_data.delayed_work, non_isr_recv );

        recv_data.cmd         = p1 >> 24;
        recv_data.len         = p1 & 0xFFFF;
        recv_data.buffer      = (char *)p2;
        recv_data.port_number = iir & IIR_PORT_MASK;
        recv_data.device      = device;

        iowrite32(IIR_CMD_MASK | IIR_PORT_MASK, &device->regs->IPC_ICR);

        ret = queue_work(ipc_workqueue, &recv_data.delayed_work);
    }

    return IRQ_HANDLED;
}

ipc_drvr_handle  ipc_attach( uint32_t device_index, uint8_t port_number, ipc_recv_callback recv_callback, void *user_param )
{
    ipc_device_config_t *device = NULL;
    ipc_port_config_t *port = NULL;

    ENTER();
    if ( device_index >= ipc_get_num_devices() ||
         ipc_devices == NULL )
    {
        EXIT();
        return NULL;
    }

    device = &ipc_devices[ device_index ];

    if ( find_device_port( device, port_number) == NULL )
    {
        down(&list_sem);

        port = kmalloc( sizeof( ipc_port_config_t ), GFP_KERNEL );
        if ( port != NULL )
        {
            memset(port, 0, sizeof(ipc_port_config_t));

            port->ipc_device    = device;
            port->port_number   = port_number;
            port->recv_callback = recv_callback;
            port->user_param    = user_param;

            list_add_tail(&port->list, &device->open_ports->list);

            if ( device->open_count == 0 )
            {
                int retval;
                pr_debug(PREFIX "first port (%d:%d) being opened, attach ISR\n", device_index, port_number);
                retval = request_irq(port->ipc_device->int_num, irq_handler, 0, IPC_NAME, device );
            }
            device->open_count++;
        }
        up(&list_sem);
    }

    EXIT();
    return port;
}
EXPORT_SYMBOL(ipc_attach);

ipc_error_type_t ipc_detach( ipc_drvr_handle handle )
{
    ipc_port_config_t   *port = ( ipc_port_config_t * )handle;
    bool                 found_port = false;
    ipc_port_config_t   *test_port;
    ipc_device_config_t *device;
    struct list_head    *cur, *q;

    ENTER();

    if ( !port_is_valid(port) )
    {
        return e_IPC_ERROR;
    }

    device = port->ipc_device;

    down(&list_sem);

    list_for_each_safe(cur, q, &device->open_ports->list)
    {
        test_port = list_entry(cur, ipc_port_config_t, list);
        if ( test_port->port_number == port->port_number )
        {
            found_port = true;

            list_del( cur );
            kfree( test_port );

            device->open_count--;
            if ( device->open_count == 0 )
            {
                pr_debug(PREFIX "last port (%d) being being closed, free ISR\n", device->instance_id);
                free_irq( device->int_num, device );
            }

            break;
        }
    }

    up(&list_sem);

    if ( found_port )
    {
        return e_IPC_SUCCESS;
    }

    EXIT();
    return e_IPC_ERROR;
}
EXPORT_SYMBOL(ipc_detach);

ipc_error_type_t ipc_send(ipc_drvr_handle handle, uint8_t command, void *buffer, uint16_t length)
{
    ipc_port_config_t *port = ( ipc_port_config_t * )handle;
    ipc_device_config_t *device = NULL;
    ipc_error_type_t result;

    ENTER();

    if ( !port_is_valid( port ) )
    {
        EXIT();
        return e_IPC_ERROR;
    }

    device = port->ipc_device;

    down(&device->tx_ready_sem);

    iowrite32( ( ( ( uint32_t ) command ) << 24 ) | length, &device->regs->IPC_WDR_0);
    iowrite32( (uint32_t)buffer, &device->regs->IPC_WDR_1);
    iowrite32( ( port->port_number << IIR_PORT_SHIFT ) | ( IIR_CMD_MASK ), &device->regs->IPC_ISRW);

    down(&device->tx_done_sem);

    if ( device->ack_type == ACK_MSG_PROCESSED )
    {
        result = e_IPC_SUCCESS;
    }
    else if ( device->ack_type == ACK_MSG_DISCARDED )
    {
        result = e_IPC_NO_LISTENER;
    }
    else
    {
        result = e_IPC_ERROR;
    }

    up(&device->tx_ready_sem);

    EXIT();
    return result;
}
EXPORT_SYMBOL(ipc_send);

static int ipc_platform_probe(struct platform_device *pdev)
{
    int retval = 0;
    int irq;
    struct resource *reg_addr;
    struct device_node *node = pdev->dev.of_node;
    struct property *prop;
    char *name;
    uint32_t dev_id;
    IPC0_REGS_t *regs;

    ENTER();
 
    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
    {
        dev_err(&pdev->dev, PREFIX "platform_get_irq failed\n");
        EXIT();
        return -ENXIO;
    }

    reg_addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!reg_addr)
    {
        dev_err(&pdev->dev, "platform_get_resource failed\n");
        EXIT();
        return -ENXIO;
    }

    prop = of_find_property(node, "device_name", NULL);
    name = (char *)of_prop_next_string(prop, NULL);

    dev_dbg(&pdev->dev, "'%s' registers at addr=0x%X, size=0x%X, irq=%d\n",
           name, reg_addr->start, resource_size(reg_addr), irq);

    if (!request_mem_region(reg_addr->start, resource_size(reg_addr), IPC_NAME))
    {
        dev_err(&pdev->dev, "request_mem_region failed\n");
        retval = -EBUSY;
    }
    regs = ioremap(reg_addr->start, resource_size(reg_addr));
    if (!regs)
    {
        dev_err(&pdev->dev, "ioremap failed\n");
        retval = -ENOMEM;
    }

    down( &list_sem );

    dev_id = num_ipc_devices;
    num_ipc_devices++;
    ipc_devices = krealloc( ipc_devices, sizeof( ipc_device_config_t ) * num_ipc_devices, GFP_KERNEL );

    ipc_devices[ dev_id ].instance_id = dev_id;
    strcpy( ipc_devices[ dev_id ].dev_name, name );
    ipc_devices[ dev_id ].int_num = irq;
    ipc_devices[ dev_id ].open_count = 0;
    ipc_devices[ dev_id ].pdev = pdev;
    ipc_devices[ dev_id ].regs = regs;

    ipc_devices[ dev_id ].open_ports = kmalloc( sizeof( ipc_port_config_t ), GFP_KERNEL );
    INIT_LIST_HEAD( &ipc_devices[ dev_id ].open_ports->list );

    sema_init( &ipc_devices[ dev_id ].tx_done_sem,  0 );
    sema_init( &ipc_devices[ dev_id ].tx_ready_sem, 1 );

    up( &list_sem );

    EXIT();
    return retval;
}


static int ipc_platform_remove(struct platform_device *pdev)
{
    ENTER();
/* 
    TODO - clean up on exit
 
    struct resource *reg_addr;
    int irq;

    ENTER();

    iounmap(regs); 
    reg_addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(reg_addr->start, resource_size(reg_addr));

    irq = platform_get_irq(pdev, 0);
    free_irq(irq, NULL); 

*/
    EXIT();
    return 0;
}

static int ipc_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
    ENTER();

    EXIT();
    return 0;
}

static int ipc_platform_resume(struct platform_device *pdev)
{
    ENTER();

    EXIT();
    return 0;
}

static struct platform_device_id mrvl_ipc_driver_ids[] = {
    {
        .name		= IPC_NAME,
    },
	{ },
};
MODULE_DEVICE_TABLE(platform, mrvl_ipc_driver_ids);

static const struct of_device_id mrvl_ipc_dt_match[] = {
    { .compatible = IPC_COMPATIBILITY_NAME, },
    {},
};
MODULE_DEVICE_TABLE(of, mrvl_ipc_dt_match);

static struct platform_driver ipc_platform_driver =
{
    .probe    = ipc_platform_probe,
    .remove   = ipc_platform_remove,
    .suspend  = ipc_platform_suspend,
    .resume   = ipc_platform_resume,
    .id_table = mrvl_ipc_driver_ids,
    .driver   = {
        .name  = IPC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mrvl_ipc_dt_match),
    }
};

static int ipc_driver_init(void)
{
    int retval;

    ENTER();

    pr_debug(PREFIX "loading driver\n");

    sema_init( &list_sem, 1 );

    num_ipc_devices = 0;
    ipc_devices = NULL;

    ipc_workqueue = create_workqueue(IPC_NAME "_wq");

    retval = platform_driver_register(&ipc_platform_driver);
    if (retval)
    {
        // do any needed cleanup
        retval = pr_err(PREFIX "%s: error registering platform driver\n",__func__);
    }
    else
    {
        pr_debug(PREFIX "platform registration complete\n");
    }
    EXIT();
    return retval;
}

static void ipc_driver_exit(void)
{
    ENTER();

    platform_driver_unregister(&ipc_platform_driver);  

    pr_debug(PREFIX "removed IPC driver\n");

    EXIT();
}

module_init(ipc_driver_init);
module_exit(ipc_driver_exit);

