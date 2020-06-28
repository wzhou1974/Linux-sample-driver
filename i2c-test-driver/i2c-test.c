#include <linux/stddef.h>       // for NULL
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm-generic/bug.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/i2c.h>
#include <linux/compiler.h>
#include <linux/dcache.h>
#include <linux/debugfs.h>
#include <linux/random.h>

/*
 * Usage: insmod i2c-test.ko i2c_num=0 i2c_dev_addr=0x53
 *
 * The  256-size eeprom STTS2002 is attached to I2C0 bus,
 * its i2c address is 0x53.
 *
 * We could check the params from
 * " /sys/module/i2c-test/parameters/i2c_num" and
 * " /sys/module/i2c-test/parameters/i2c_dev_addr"
 *
 * mount -t debugfs none /sys/kernel/debug
 * /sys/kernel/debug/i2c-test/backup
 * /sys/kernel/debug/i2c-test/read-buf-0
 * /sys/kernel/debug/i2c-test/read-buf-1
 * /sys/kernel/debug/i2c-test/write-buf-0
 * /sys/kernel/debug/i2c-test/write-buf-1
 *
 * backup file contains all 256-size data after writing write-buf-0 
 * and write-buf-1.   
 * 
 * write-buf-0 and write-buf-1 contain generated random data.
 * read-buf-0 should contains the data that is same as write-buf-0  
 * read-buf-1 should contains the data that is same as write-buf-1 
 *
 * The test code is for checking the modification of i2c-pxa driver.
 * 
 * There are 2 restrictions in the original i2c_pxa_do_xfer()
 * 1. The i2c message array could contain only one read-operation message
 * 2. The read-operation message must be the last one
 * The modification removes the 2 restrictions.
 * 
 */

static uint i2c_num = 0;
module_param(i2c_num, uint, 0644);

static uint i2c_dev_addr = 0x53;
module_param(i2c_dev_addr, uint, 0644);

// STTS2002 eeprom size
#define I2C_TEST_EEPROM_SIZE     256
static uint8_t backup[I2C_TEST_EEPROM_SIZE];

#define I2C_TEST_RDWR_LENGTH    8

static struct i2c_adapter *adapter;

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfsdir;
static struct debugfs_blob_wrapper debug_backup;
static struct debugfs_blob_wrapper debug_i2c_read_buf[2];
static struct debugfs_blob_wrapper debug_i2c_write_buf[2];
#endif

static uint8_t i2c_read_buf_0[I2C_TEST_RDWR_LENGTH];
static uint8_t i2c_read_buf_1[I2C_TEST_RDWR_LENGTH];

static uint8_t i2c_write_buf_0[I2C_TEST_RDWR_LENGTH];
static uint8_t i2c_write_buf_1[I2C_TEST_RDWR_LENGTH];

static void i2c_test_msg_prepare(struct i2c_msg msgs[], uint8_t i2c_dev_addr,  uint8_t *offset, uint8_t *buf,  uint16_t len, uint16_t operation)
{

	/*
	 * random read
	*/
    msgs[0].addr = i2c_dev_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = offset;

    msgs[1].addr = i2c_dev_addr;
    msgs[1].flags = operation;
    msgs[1].len = len;
    msgs[1].buf = buf;

    return;
}

/*
 * read all data from 256-size eeprom STTS2002 for checking
 * return 0 means successful 
*/
int i2c_test_eeprom_save(uint8_t *buffer, uint16_t size)
{
    uint8_t        offset;
    struct i2c_msg msgs[2];

    // read from offset 0        
    offset = 0;
    i2c_test_msg_prepare(msgs, i2c_dev_addr, &offset, buffer, size, I2C_M_RD);
    if(i2c_transfer(adapter, msgs, 2) != 2) {
       printk(KERN_ERR "save data fail\n");
       return  -EIO;
   }
   return   0;
}


int i2c_test_repeated_read_write(uint i2c_num, uint i2c_dev_addr)
{
    int     rc = 0;
    struct i2c_msg msgs[6];
    int     io_completed;
    int     cmp_0, cmp_1;

    /*
     *  the offset address of eeprom drvice.
     *  Because STTS2002 has only 256 bytes, the offset is a byte size.
     *
    */
    uint8_t     write_offset;
    uint8_t     read_offset_0, read_offset_1;    
    uint8_t     temp_buf[I2C_TEST_EEPROM_SIZE + 1];

#ifdef CONFIG_DEBUG_FS
    struct dentry *debug_buf_file;
#endif

	BUG_ON(adapter == NULL);    

	/*
	 * test [write, read]
	*/

    /*
     * write data to offset 0
    */        
    #define I2C_RDWR_POS_0    0x10
    #define I2C_RDWR_POS_1    0x20
    
    temp_buf[0] = I2C_RDWR_POS_0;
    memcpy(&temp_buf[1], i2c_write_buf_0, I2C_TEST_RDWR_LENGTH);
    
	msgs[0].addr = i2c_dev_addr;
	msgs[0].flags = 0;
	msgs[0].len = I2C_TEST_RDWR_LENGTH + 1;
	msgs[0].buf = temp_buf;
    
    if(i2c_transfer(adapter, msgs, 1) == 1) {
        printk(KERN_INFO "i2c_test: write (io) successfully\n");
    } else {
        printk(KERN_ERR "i2c_test: write (io) fail\n");
		return 1;
    }    
    
    write_offset = I2C_RDWR_POS_1;
	msgs[0].addr = i2c_dev_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &write_offset;        
    
    // write data
	msgs[1].addr = i2c_dev_addr;
	msgs[1].flags = I2C_M_NOSTART;      // the flag set is must according to    
                                        // STTS2002's write mode sequence requirement    
	msgs[1].len = I2C_TEST_RDWR_LENGTH;
	msgs[1].buf = i2c_write_buf_1;
    
    if(i2c_transfer(adapter, msgs, 2) == 2) {
        printk(KERN_INFO "i2c_test: write (io2) successfully\n");
    } else {
        printk(KERN_ERR "i2c_test: write (io2) fail\n");
		return 2;
    }    
    
    //--------------------------------------------------
    // read     
    
    // write offset
    read_offset_0 = I2C_RDWR_POS_0;
	msgs[0].addr = i2c_dev_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &read_offset_0;
    
    // read data
	msgs[1].addr = i2c_dev_addr;
	msgs[1].flags = I2C_M_RD | I2C_M_STOP;  // I2C_M_STOP flag is must according to
                                            // STTS2002's read mode sequence requirement
	msgs[1].len = I2C_TEST_RDWR_LENGTH;
	msgs[1].buf = i2c_read_buf_0;
    
    // write offset
    read_offset_1 = I2C_RDWR_POS_1;
	msgs[2].addr = i2c_dev_addr;
	msgs[2].flags = 0;
	msgs[2].len = 1;
	msgs[2].buf = &read_offset_1;
    
    // read data
	msgs[3].addr = i2c_dev_addr;
	msgs[3].flags = I2C_M_RD;               // I2C_M_STOP flag is optional, because the 
                                            // message is the last one. STOP condition
                                            // is sent inevitably           
	msgs[3].len = I2C_TEST_RDWR_LENGTH;
	msgs[3].buf = i2c_read_buf_1;
    
    if((io_completed = i2c_transfer(adapter, msgs, 4)) == 4) {
        printk(KERN_INFO "i2c_test: write (io3) successfully: %d\n", io_completed);
		cmp_0 = memcmp(i2c_write_buf_0, i2c_read_buf_0, I2C_TEST_RDWR_LENGTH);
		cmp_1 = memcmp(i2c_write_buf_1, i2c_read_buf_1, I2C_TEST_RDWR_LENGTH);
		if (cmp_0 | cmp_0) {
			printk(KERN_ERR "i2c_test: test fail!!!\n");
			return 3;
		} else {
			printk(KERN_INFO "i2c_test: test successfully!!!\n");
		}

    } else {
        printk(KERN_ERR "i2c_test: write (io3) fail: %d\n", io_completed);
		return 4;
    }                        

	/*
	 * read all eeprom data for checking
	*/
    rc = i2c_test_eeprom_save(backup, I2C_TEST_EEPROM_SIZE);

	/*
	 * dump the read/ write data for checking manually
	*/
#ifdef CONFIG_DEBUG_FS
    debug_backup.data = backup;
    debug_backup.size = I2C_TEST_EEPROM_SIZE;
    debug_buf_file = debugfs_create_blob("backup", S_IRUSR, debugfsdir, &debug_backup);
    BUG_ON(IS_ERR(debug_buf_file));

    debug_i2c_read_buf[0].data = i2c_read_buf_0;
    debug_i2c_read_buf[0].size = I2C_TEST_RDWR_LENGTH;
    debug_buf_file = debugfs_create_blob("read-buf-0", S_IRUSR, debugfsdir, &debug_i2c_read_buf[0]);
    BUG_ON(IS_ERR(debug_buf_file));

    debug_i2c_read_buf[1].data = i2c_read_buf_1;
    debug_i2c_read_buf[1].size = I2C_TEST_RDWR_LENGTH;
    debug_buf_file = debugfs_create_blob("read-buf-1", S_IRUSR, debugfsdir, &debug_i2c_read_buf[1]);
    BUG_ON(IS_ERR(debug_buf_file));

    debug_i2c_write_buf[0].data = i2c_write_buf_0;
    debug_i2c_write_buf[0].size = I2C_TEST_RDWR_LENGTH;
    debug_buf_file = debugfs_create_blob("write-buf-0", S_IRUSR, debugfsdir, &debug_i2c_write_buf[0]);
    BUG_ON(IS_ERR(debug_buf_file));

    debug_i2c_write_buf[1].data = i2c_write_buf_1;
    debug_i2c_write_buf[1].size = I2C_TEST_RDWR_LENGTH;
    debug_buf_file = debugfs_create_blob("write-buf-1", S_IRUSR, debugfsdir, &debug_i2c_write_buf[1]);
    BUG_ON(IS_ERR(debug_buf_file));
#endif
		
    return rc;
}

static int __init i2c_test_init(void)
{
	printk(KERN_INFO "i2c_test init (i2c-bus = %d i2c-address = 0x%x): \n", i2c_num, i2c_dev_addr);
    
    
    /*
     * fill the random data for writing them to eeprom
    */    
    get_random_bytes(i2c_write_buf_0, I2C_TEST_RDWR_LENGTH);
    get_random_bytes(i2c_write_buf_1, I2C_TEST_RDWR_LENGTH);
    
#ifdef CONFIG_DEBUG_FS
    debugfsdir = debugfs_create_dir("i2c_test", NULL);
    BUG_ON(IS_ERR(debugfsdir));
#endif

	printk(KERN_INFO "i2c_test get adapter\n");
	adapter = i2c_get_adapter(i2c_num);
	BUG_ON(adapter == NULL);
        
    if(i2c_test_repeated_read_write(i2c_num, i2c_dev_addr))  {
        printk(KERN_ERR "i2c_test NG\n");
    } else {
        printk(KERN_INFO "i2c_test pass\n");
    }    

    return  0;
}
module_init(i2c_test_init);

static void __exit i2c_test_exit(void)
{
    printk(KERN_INFO "i2c_test unload\n");
#ifdef CONFIG_DEBUG_FS
    debugfs_remove_recursive(debugfsdir);
#endif

    i2c_put_adapter(adapter);
}
module_exit(i2c_test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Marvell i2c_test driver");
