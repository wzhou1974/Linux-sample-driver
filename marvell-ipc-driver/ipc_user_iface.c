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
#include <linux/kernel.h>

#include "ipc_api.h"

MODULE_LICENSE("Dual BSD/GPL");

#define IPC_NAME "ipc"
#define IPC_PORT_NAME (IPC_NAME "_port")
#define IPC_IFACE_NAME "ipc_iface"
#define PREFIX IPC_IFACE_NAME ": "

#define ENTER() pr_debug(PREFIX "ENTER %s\n", __FUNCTION__)
#define EXIT()  pr_debug(PREFIX "EXIT  %s:%d\n", __FUNCTION__, __LINE__)

static ssize_t port_export_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t port_unexport_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t port_read(struct device *dev, struct device_attribute *attr,char *buf);
static ssize_t port_write(struct device *dev, struct device_attribute *attr,const char *buf, size_t count);

static struct device_attribute ipc_dev_attrs[] =
{
    __ATTR(export,   S_IWUSR, NULL, port_export_set),
    __ATTR(unexport, S_IWUSR, NULL, port_unexport_set),
    __ATTR_NULL,
};

static struct class ipc_class =
{
    .name        = IPC_NAME,
    .owner       = THIS_MODULE,
    .dev_attrs   = ipc_dev_attrs,
};

static struct device_attribute ipc_port_attrs[] =
{
    __ATTR(rw,   S_IWUSR | S_IRUSR, port_read, port_write),
    __ATTR_NULL,
};

static struct class ipc_port_class =
{
    .name        = IPC_PORT_NAME,
    .owner       = THIS_MODULE,
    .dev_attrs   = ipc_port_attrs,
};


uint32_t get_device_index(struct device *dev)
{
    void *data = dev_get_drvdata(dev);
    return (uint32_t)data;
}

void recv_callback(ipc_drvr_handle handle, void *user_param, uint8_t command, void *buffer, uint16_t length)
{
    if ((buffer != NULL) && (length > 0))
    {
        pr_alert("IPC_IFACE (%s.%d) received cmd %d, buffer 0x%p, len %d\n", ipc_get_device_name(((uint32_t)user_param) >> 8), ((uint32_t)user_param) & 0xFF, command, buffer, length);
        if (command == 255)
        {
            pr_alert("\"%s\"\n", (char*)buffer);
        }
    }
    else
    {
        pr_alert("IPC_IFACE (%s.%d) received cmd %d, param %d\n", ipc_get_device_name(((uint32_t)user_param) >> 8), ((uint32_t)user_param) & 0xFF, command, (int)buffer);
    }
}

static ssize_t port_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t port_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd = 0;
    int param = 0;
    ipc_error_type_t err;

    ENTER();

    sscanf(buf, "%i,%i", &cmd, &param);

    err = ipc_send(dev_get_drvdata(dev), cmd, (void *)param, 0);

    if ( err == e_IPC_NO_LISTENER )
    {
        pr_alert( PREFIX "message sent, but nothing is attached to that port on the remote side.\n" );
    }

    EXIT();
    return count;
}

static ssize_t port_export_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long port_number;
    uint32_t device_index = get_device_index(dev);

    ENTER();

    if ((kstrtoul(buf, 10, &port_number) == 0) && (port_number > 0) && (port_number < 256))
    {
        ipc_drvr_handle  port_handle;
        struct device   *port_dev;

        port_handle = ipc_attach(device_index, (uint8_t)port_number, recv_callback, (void*)((device_index << 8) | port_number));

        port_dev = device_create(&ipc_port_class, dev, MKDEV(0,0), port_handle, "%s.%d", dev_name(dev), (int)port_number);
        if (IS_ERR( port_dev ))
        {
            pr_err(PREFIX "failed to create device for exported port %d\n", (int)port_number);
        }
    }

    EXIT();

    return count;
}

static int match_port( struct device *dev, void *data )
{
    if (strcmp(dev_name(dev), (char*)data) == 0)
        return 1;
    return 0;
}

static ssize_t port_unexport_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long port_number;

    ENTER();

    if ((kstrtoul(buf, 10, &port_number) == 0) && (port_number > 0) && (port_number < 256))
    {
        struct device *port_dev;
        char          *port_name;

        port_name = kmalloc(strlen(dev_name(dev)) + count + 2, GFP_KERNEL );
        if (port_name != NULL)
        {
            sprintf(port_name, "%s.%d", dev_name(dev), (int)port_number);
            port_dev = device_find_child( dev, port_name, match_port);
            if (port_dev != NULL)
            {
                ipc_detach(dev_get_drvdata(port_dev));
                device_unregister(port_dev);
                put_device(port_dev);
            }
            kfree(port_name);
        }
    }

    EXIT();
    return count;
}

static int ipc_user_iface_init(void)
{
    int retval = 0;
    int i;
    int num_devices = ipc_get_num_devices();

    ENTER();

    pr_debug(PREFIX "loading driver\n");

    class_register(&ipc_class);

    for (i = 0; i < num_devices; i++)
    {
        device_create(&ipc_class, NULL, MKDEV(0, 0), (void *)i, ipc_get_device_name(i));
    }

    class_register(&ipc_port_class);

    EXIT();
    return retval;
}

static void ipc_user_iface_exit(void)
{
    struct class_dev_iter iter;
    struct device *dev;

    ENTER();

    class_unregister(&ipc_port_class);

    class_dev_iter_init(&iter, &ipc_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter)))
    {
        device_unregister(dev);
        put_device(dev);
    }
    class_dev_iter_exit(&iter);

    class_unregister(&ipc_class);

    pr_debug(PREFIX "removed IPC driver\n");

    EXIT();
}

module_init(ipc_user_iface_init);
module_exit(ipc_user_iface_exit);

