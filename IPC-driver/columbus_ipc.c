/*------------------------------------------------------------------------*/
/*                                                                        */
/* Copyright (C) 2017 Brite Semiconductor Co., Ltd. All rights reserved.  */
/*                                                                        */
/*------------------------------------------------------------------------*/


#include <linux/stddef.h>       /* for NULL */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <asm-generic/bug.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <misc/columbus_ipc.h>
#include "columbus_ipc_internal.h"

#ifdef DEBUG
#define COLUMBUS_IPC_UNITTEST
#define IPC_BUG	BUG_ON
#define COLUMBUS_IPC_MISC_DEVICE
#else
#define IPC_BUG
#endif

#define COLUMBUS_IPC_CDL_RECEIVE_BACK
#define COLUMBUS_IPC_CDL_SEND_BACK


#define COLUMBUS_IPC_NAME	"columbus_ipc"
#define IPC_IRQ_CHANNEL_NUM	8

#define A7_RF_IPC_CHANNEL_NUM   16
#define A7_PLC_IPC_CHANNEL_NUM  16

#define SHARED_RAM_PAGE_NUM	32

#define IPC_CHANNEL_USED	1
#define IPC_CHANNEL_UNUSED	0

/* 32K IPC SRAM */
#define COLUMBUS_IPC_SRAM_SIZE	0x8000

/* 1024 bytes per channel */
#define COLUMBUS_IPC_PAGE_SIZE	0x400

/* Maybe the following config value could be moved to dts */
/* Shared RAM from RF DSP view */
#define COLUMBUS_SRAM_RF_VIEW		0x72600000
/* Shared RAM from PLC DSP view */
#define COLUMBUS_SRAM_PLC_VIEW		0x61600000


struct ipc_irq_data {
	unsigned int irq_from_dsp;	/* save the virtual irq */
	struct completion irq_done;
};

struct columbus_ipc_info {
	struct device	*dev;
	/* ipc io space from ARM A7 side */
	void __iomem	*io_base;

	/* The shared RAM base from ARM A7 side */
	void		*sram;
	phys_addr_t	sram_phy;

	/*
	 * The DSP senders(RF and PLC) could trigger the receiver's
	 * interrupt.
	 */
	struct ipc_irq_data ipc_irq[IPC_IRQ_CHANNEL_NUM + IPC_IRQ_CHANNEL_NUM];
};

/* make the crashed data structure could be detectable */
#define COLUMBUS_IPC_REQ_MAGIC_1    0x49504331	/* IPC1 */
#define COLUMBUS_IPC_REQ_MAGIC_2    0x49504332	/* IPC2 */

struct ipc_channel {
	unsigned int magic_1;
	int used;	/*
			 * if used is IPC_CHANNEL_USED, means the channel
			 * has been occupied.
			 */
	int partner;	/* A7's communication partner, RF dsp or PLC dsp */
	int operation;	/* send or receive */
	int mode;	/*
			 * currently, the hardware only support the
			 * receiver could get interrupt.
			 */
	unsigned int magic_2;
};

/* protect ipc_rf_channel[] table */
static DEFINE_MUTEX(ipc_rf_req_tbl_mutex);

/* protect ipc_plc_channel[] table */
static DEFINE_MUTEX(ipc_plc_req_tbl_mutex);

static struct ipc_channel ipc_rf_channel[A7_RF_IPC_CHANNEL_NUM];
static struct ipc_channel ipc_plc_channel[A7_PLC_IPC_CHANNEL_NUM];

static struct columbus_ipc_info columbus_ipc;

#ifdef COLUMBUS_IPC_MISC_DEVICE
/* create misc device for somebody happiness and the annoying noise */
unsigned char open_count;
static struct miscdevice miscdev;
#endif

/*
 * The following mutex prevents the different threads running on A7 to access
 * the shared ram at the same time. BUT it could not prevent multi-cores to
 * request the same page(s) ownership at the same time.
 */
static DEFINE_MUTEX(ipc_sram_mutex);

/* These global variables are only for sysfs interfaces */
static int current_partner = COLUMBUS_IPC_INVALID;
static int current_operation = COLUMBUS_IPC_INVALID;
static int current_mode = COLUMBUS_IPC_INVALID;
static int current_channel = COLUMBUS_IPC_INVALID;

/* Meet cdl code running on DSP happily */
static int current_page = COLUMBUS_IPC_INVALID;


#ifdef DEBUG
static void ipc_dump_shared_ram_ownership(void)
{
	u32 srmsel0, srmsel1;
	int i;
	u32 ownership;

	static char *status[4] = {
		"free",
		"a7",
		"rf-dsp",
		"plc-dsp"
	};

	srmsel0 = ioread32(columbus_ipc.io_base + SRMSEL0);
	srmsel1 = ioread32(columbus_ipc.io_base + SRMSEL1);

	dev_info(columbus_ipc.dev, "------------------------------------------------\n");
	dev_info(columbus_ipc.dev, "Shared RAM ownership:\n");
	dev_info(columbus_ipc.dev, "%08X\t%08X\n", srmsel0, srmsel1);

	for (i = 0; i < 16; i++) {
		ownership = (srmsel0 >> (i * 2)) & 0x3;
		dev_info(columbus_ipc.dev, "page %2d: %s\n", i,
			 status[ownership]);
	}

	for (i = 0; i < 16; i++) {
		ownership = (srmsel1 >> (i * 2)) & 0x3;
		dev_info(columbus_ipc.dev, "page %2d: %s\n", i + 16,
			 status[ownership]);
	}
	dev_info(columbus_ipc.dev, "------------------------------------------------\n");
}

#else
static void ipc_dump_shared_ram_ownership(void) {}
#endif

enum ownership {
	ownership_free,
	ownership_a7,
	ownership_rf,
	ownership_plc
};

static enum ownership get_sram_page_ownership(int page_num)
{
	int srmsel;
	int bit_position;
	u32 status;

	enum ownership retval;

	IPC_BUG(page_num < 0);
	IPC_BUG(page_num >= SHARED_RAM_PAGE_NUM);

	srmsel = page_num / 16;
	bit_position = page_num % 16;

	status = ioread32(columbus_ipc.io_base + SRMSEL0 + 4 * srmsel);
	retval = (enum ownership)((status >> (bit_position << 1)) & 0x3);

	return retval;
}

/* return 1, means successfully; otherwise return 0. */
static int grab_one_sram_page(int page_num)
{
	u32 request_ownership;
	enum ownership status;

	status = get_sram_page_ownership(page_num);
	if (status != ownership_free) {
		/* The page has already been occupied. */
		return	0;
	}

	request_ownership = (A7_REQ_KEY << 4) | ownership_a7;
	iowrite32(request_ownership,
		  columbus_ipc.io_base + A7SRP00REQ + page_num * 4);

	status = get_sram_page_ownership(page_num);
	if (unlikely(status != ownership_a7)) {
		dev_err(columbus_ipc.dev, "fail to grab %d page\n", page_num);
		return 0;
	}
	return 1;
}

/* release the specific sram page */
static void release_one_sram_page(int page_num)
{
	u32 request_ownership;
	enum ownership status;

	status = get_sram_page_ownership(page_num);
	IPC_BUG(status != ownership_a7);

	request_ownership = (A7_REQ_KEY << 4) | ownership_free;
	iowrite32(request_ownership,
		  columbus_ipc.io_base + A7SRP00REQ + page_num * 4);

	status = get_sram_page_ownership(page_num);
	IPC_BUG(status != ownership_free);
}

/*
 * If successfully, return 1 and *page_num is the start page number,
 * otherwise return 0.
 */
static int find_consecutive_free_sram_pages(int npages, int *page_num)
{
	u64 sram_ownership_status;
	u32 srmsel0, srmsel1;
	int i;
	int count;
	int start;
	enum ownership page_status;

	srmsel0 = ioread32(columbus_ipc.io_base + SRMSEL0);
	srmsel1 = ioread32(columbus_ipc.io_base + SRMSEL1);

	sram_ownership_status = srmsel1;
	sram_ownership_status = sram_ownership_status << 32;
	sram_ownership_status |= srmsel0;

	for (i = 0, count = 0, start = -1; i < 32; i++) {
		page_status = (enum ownership)
			(sram_ownership_status >> (i * 2) & 0x3);

		if (page_status == ownership_free) {
			count++;
			if (start == -1)
				start = i;

			if (count >= npages)
				break;
		} else {
			/* reset count and start */
			count = 0;
			start = -1;
		}
	}

	if (count == npages) {
		IPC_BUG(start == -1);
		IPC_BUG(start > 32 - count);
		*page_num = start;

		return 1;
	}

	return 0;
}

/*
 * If page_num is -1, means the invoker doesn't care about the specific
 * page(s), it only wants to allocate npages sram space.
 *
 * If page_num is not -1, means the invoker hopes to the allocated npages
 * sram space is start from page_num.
 *
 * return the start page number of the allocated shared ram;
 * return -1, fail
 */
static int try_to_grab_sram_pages(int page_num, int npages)
{
	int i;
	int grab;
	int start_page = -1;
	int find_result;

	if (page_num == -1) {
		/*
		 * We need to find the consecutive free pages in the shared
		 * ram.
		 */
		find_result = find_consecutive_free_sram_pages(npages,
							       &start_page);
		if (unlikely(find_result == 0)) {
			dev_err(columbus_ipc.dev, "Could not find %d "
						"consecutive free pages in "
						"shared ram\n",
				npages);

			ipc_dump_shared_ram_ownership();
			return -1;
		}

		page_num = start_page;
	}

	mutex_lock(&ipc_sram_mutex);

	for (i = 0; i < npages; i++) {
		grab = grab_one_sram_page(page_num + i);
		if (unlikely(grab == 0)) {
			dev_err(columbus_ipc.dev,
				"fail to grab %d page\n",
				page_num + i);
			ipc_dump_shared_ram_ownership();
			break;
		}
	}

	mutex_unlock(&ipc_sram_mutex);

	if (i == npages)
		return page_num;
	else
		return -1;
}

static void free_sram_pages(int page_num, int npages)
{
	int i;

	mutex_lock(&ipc_sram_mutex);

	for (i = 0; i < npages; i++)
		release_one_sram_page(page_num + i);

	mutex_unlock(&ipc_sram_mutex);
}

/*
 * if addr is not NULL, the function will try to allocate size shared ram from
 * the assigned address; otherwise it will allocate buffer at will.
 */
static char *ipc_sram_alloc(char *addr, int size)
{
	int	npages;
	int page_num = -1;
	char *ret_addr = NULL;

	if (unlikely(size <= 0))
		return	NULL;

	/* The shared RAM size is 32K */
	if (unlikely(size > COLUMBUS_IPC_SRAM_SIZE))
		return	NULL;

	/*
	 * if the addr is assigned, it must be aligned on page size
	 * boundary(1K)
	 */
	if (unlikely((u32)addr & (COLUMBUS_IPC_PAGE_SIZE - 1)))
		return	NULL;

	npages = size / COLUMBUS_IPC_PAGE_SIZE;
	if (size % COLUMBUS_IPC_PAGE_SIZE)
		npages++;

	if (addr) {
		/* Check validation of the assigned addr */
		if (unlikely((void *)addr < columbus_ipc.sram ||
			     (void *)addr >= columbus_ipc.sram +
			     COLUMBUS_IPC_SRAM_SIZE))
			return	NULL;

		page_num = (addr - (char *)columbus_ipc.sram) /
			COLUMBUS_IPC_PAGE_SIZE;
		IPC_BUG((addr - (char *)columbus_ipc.sram) %
		       COLUMBUS_IPC_PAGE_SIZE);
	}

	page_num = try_to_grab_sram_pages(page_num, npages);

	if (page_num != -1)
		ret_addr = columbus_ipc.sram + page_num *
			COLUMBUS_IPC_PAGE_SIZE;
	else
		dev_err(columbus_ipc.dev,
			"failed to allocate %d page(s) from %d\n",
			npages, page_num);


	return	ret_addr;
}

static void ipc_sram_free(char *addr, int size)
{
	int	npages;
	int page_num = -1;

	IPC_BUG(size <= 0);
	IPC_BUG(size > COLUMBUS_IPC_SRAM_SIZE);
	IPC_BUG(addr == NULL);

	/* addr must be align on page size boundary(1K) */
	IPC_BUG((u32)addr & (COLUMBUS_IPC_PAGE_SIZE - 1));
	IPC_BUG((void *)addr < columbus_ipc.sram ||
	       (void *)addr >= columbus_ipc.sram + COLUMBUS_IPC_SRAM_SIZE);

	page_num = (addr - (char *)columbus_ipc.sram) / COLUMBUS_IPC_PAGE_SIZE;
	IPC_BUG(
		(addr - (char *)columbus_ipc.sram) % COLUMBUS_IPC_PAGE_SIZE
		);

	npages = size / COLUMBUS_IPC_PAGE_SIZE;
	if (size % COLUMBUS_IPC_PAGE_SIZE)
		npages++;

	free_sram_pages(page_num, npages);
}


/*
 * if appointed_channel is COLUMBUS_IPC_INVALID, the function will return a
 * free channel at will.
 *
 * Note: Invoking the function should be protected by the specific lock
 */
static struct ipc_channel *get_free_channel(struct ipc_channel *channels,
					    int max_channel,
					    int appointed_channel)
{
	int i;

	IPC_BUG(max_channel > A7_RF_IPC_CHANNEL_NUM);
	IPC_BUG(channels != &ipc_rf_channel[0] &&
	       channels != &ipc_plc_channel[0]);
	IPC_BUG(appointed_channel != COLUMBUS_IPC_INVALID &&
	       appointed_channel >= max_channel);
	IPC_BUG(appointed_channel < COLUMBUS_IPC_INVALID);

	if (appointed_channel != COLUMBUS_IPC_INVALID) {
		/* user hope to get the appointed channel */
		if (channels[appointed_channel].used == IPC_CHANNEL_UNUSED) {
			channels[appointed_channel].used = IPC_CHANNEL_USED;
			return &channels[appointed_channel];
		}
	} else {
		/*
		 * user only want to get a free channel, not care which
		 * channel
		 */
		for (i = 0; i < max_channel; i++) {
			if (likely(channels[i].used == IPC_CHANNEL_UNUSED)) {
				channels[i].used = IPC_CHANNEL_USED;
				return  &channels[i];
			}
		}
	}

	return  NULL;
}

static void set_ipc_channel(struct ipc_channel * const channel,
			    int partner,
			    int operation,
			    int mode)
{
	channel->magic_1 = COLUMBUS_IPC_REQ_MAGIC_1;
	channel->mode = mode;
	channel->operation = operation;
	channel->partner = partner;
	channel->magic_2 = COLUMBUS_IPC_REQ_MAGIC_2;
}

/*
 * Make all fields in struct ipc_channel are invalid.
 *
*/
static void clear_ipc_channel(struct ipc_channel * const channel)
{
	channel->used = IPC_CHANNEL_UNUSED;
	channel->mode = COLUMBUS_IPC_INVALID;
	channel->operation = COLUMBUS_IPC_INVALID;
	channel->partner = COLUMBUS_IPC_INVALID;

	channel->magic_1 = COLUMBUS_IPC_INVALID;
	channel->magic_2 = COLUMBUS_IPC_INVALID;
}


static struct mutex *get_lock(int partner)
{
	struct mutex *pmutex;

	if (partner == IPC_PARTNER_RF_DSP)
		pmutex = &ipc_rf_req_tbl_mutex;
	else {
		IPC_BUG(partner != IPC_PARTNER_PLC_DSP);
		pmutex = &ipc_plc_req_tbl_mutex;
	}
	return  pmutex;
}


static struct ipc_channel *get_channels(int partner)
{
	struct ipc_channel *channels;

	if (partner == IPC_PARTNER_RF_DSP)
		channels = &ipc_rf_channel[0];
	else {
		IPC_BUG(partner != IPC_PARTNER_PLC_DSP);
		channels = &ipc_plc_channel[0];
	}

	return channels;
}


static int get_max_channel(int operation, int mode)
{
	int max_channel = -1;

	if (mode == IPC_COMMUNICATION_INT)
		max_channel = IPC_IRQ_CHANNEL_NUM;
	else {
		IPC_BUG(mode != IPC_COMMUNICATION_POLL);
		max_channel = A7_RF_IPC_CHANNEL_NUM;
	}

	return  max_channel;
}

/* return the unused channel, if return -1, means there is no free channel */
channel_handle columbus_ipc_get_channel(int partner,
					int operation,
					int mode,
					int appointed_channel)
{
	struct mutex *plock;
	struct ipc_channel *channels;
	struct ipc_channel *channel;
	int max_channel;

	IPC_BUG(partner != IPC_PARTNER_RF_DSP &&
	       partner != IPC_PARTNER_PLC_DSP);
	IPC_BUG(operation != IPC_SEND_OPERATION &&
	       operation != IPC_RECEIVE_OPERATION);
	IPC_BUG(mode != IPC_COMMUNICATION_INT &&
	       mode != IPC_COMMUNICATION_POLL);

	plock = get_lock(partner);
	channels = get_channels(partner);
	max_channel = get_max_channel(operation, mode);

	mutex_lock(plock);

	channel = get_free_channel(channels, max_channel, appointed_channel);

	mutex_unlock(plock);

	if (likely(channel != NULL))
		set_ipc_channel(channel, partner, operation, mode);

	return  channel;
}
EXPORT_SYMBOL(columbus_ipc_get_channel);


/* put the used channel, if return -1, means the put channel is invalid */
void columbus_ipc_put_channel(channel_handle channel)
{
	struct mutex *plock;
	struct ipc_channel *channel_2 = (struct ipc_channel *)channel;

	int max_channel;

	IPC_BUG(channel_2->magic_1 != COLUMBUS_IPC_REQ_MAGIC_1);
	IPC_BUG(channel_2->magic_2 != COLUMBUS_IPC_REQ_MAGIC_2);
	IPC_BUG(channel_2->used != IPC_CHANNEL_USED);
	IPC_BUG(channel_2->partner != IPC_PARTNER_RF_DSP &&
	       channel_2->partner != IPC_PARTNER_PLC_DSP);
	IPC_BUG(channel_2->operation != IPC_SEND_OPERATION &&
	       channel_2->operation != IPC_RECEIVE_OPERATION);
	IPC_BUG(channel_2->mode != IPC_COMMUNICATION_INT &&
	       channel_2->mode != IPC_COMMUNICATION_POLL);

	plock = get_lock(channel_2->partner);
	max_channel = get_max_channel(channel_2->operation, channel_2->mode);

	mutex_lock(plock);

	clear_ipc_channel(channel_2);

	mutex_unlock(plock);
}
EXPORT_SYMBOL(columbus_ipc_put_channel);


static int channel2num(channel_handle channel)
{
	struct ipc_channel *channels;
	struct ipc_channel *channel_2;
	int max_channel;
	int num;

	channel_2 = (struct ipc_channel *)channel;
	channels = get_channels(channel_2->partner);

	IPC_BUG(channel_2 < channels);

	max_channel = get_max_channel(channel_2->operation, channel_2->mode);
	IPC_BUG(channel_2 >= channels + max_channel);

	num = channel_2 - channels;

	IPC_BUG(num < 0 || num >= max_channel);

	return num;
}

static int notify_partner(channel_handle channel)
{
	int	channel_num = channel2num(channel);
	struct	ipc_channel *channel_2 = (struct ipc_channel *)channel;
	u32     channel_set;

	channel_set = 0x01 << channel_num;

	if (channel_2->partner == IPC_PARTNER_RF_DSP)
		iowrite32(channel_set, columbus_ipc.io_base + A7TORFIPCSET);
	else
		iowrite32(channel_set, columbus_ipc.io_base + A7TOPLCIPCSET);

	return  0;
}


static int ack_partner(channel_handle channel)
{
	int	channel_num = channel2num(channel);
	struct	ipc_channel *channel_2 = (struct ipc_channel *)channel;
	u32     channel_ack;

	channel_ack = 0x01 << channel_num;

	if (channel_2->partner == IPC_PARTNER_RF_DSP)
		iowrite32(channel_ack, columbus_ipc.io_base + RFTOA7IPCACK);
	else
		iowrite32(channel_ack, columbus_ipc.io_base + PLCTOA7IPCACK);

	return  0;
}

/* page_num is shared ram's page number */
static char *pagenum2pageaddr(int page_num)
{
	if (page_num == -1)
		return	NULL;
	else
		return columbus_ipc.sram + page_num * COLUMBUS_IPC_PAGE_SIZE;
}

/* The page_addr must be the virtual address of the shared ram. */
static int pageaddr2pagenum(char *page_addr)
{
	IPC_BUG((u32)page_addr % COLUMBUS_IPC_PAGE_SIZE);
	IPC_BUG(page_addr < (char *)columbus_ipc.sram);
	IPC_BUG(page_addr >=
		       (char *)columbus_ipc.sram + COLUMBUS_IPC_SRAM_SIZE);
	return (page_addr - (char *)columbus_ipc.sram) / COLUMBUS_IPC_PAGE_SIZE;
}

static int is_valid_address(channel_handle channel, phys_addr_t address)
{
	struct ipc_channel *channel_2 = (struct ipc_channel *)channel;

	if (channel_2->partner == IPC_PARTNER_RF_DSP) {
		if (address >= COLUMBUS_SRAM_RF_VIEW &&
		    address < COLUMBUS_SRAM_RF_VIEW + COLUMBUS_IPC_SRAM_SIZE) {
			IPC_BUG(address % COLUMBUS_IPC_PAGE_SIZE);
			return	1;
		}
	} else {
		if (address >= COLUMBUS_SRAM_PLC_VIEW &&
		    address < COLUMBUS_SRAM_PLC_VIEW + COLUMBUS_IPC_SRAM_SIZE) {
			IPC_BUG(address % COLUMBUS_IPC_PAGE_SIZE);
			return	1;
		}
	}
	return	0;
}

/* The address is from DSP side, need to convert address from A7 side. */
static phys_addr_t address_from_a7_view(channel_handle channel,
					phys_addr_t address)
{
	resource_size_t offset;
	struct ipc_channel *channel_2 = (struct ipc_channel *)channel;

	if (channel_2->partner == IPC_PARTNER_RF_DSP)
		offset = address - COLUMBUS_SRAM_RF_VIEW;
	else
		offset = address - COLUMBUS_SRAM_PLC_VIEW;

	return	columbus_ipc.sram_phy + offset;
}

/* get the virtual address of sram  */
static void __iomem *phy2vir(phys_addr_t address)
{
	IPC_BUG(address < columbus_ipc.sram_phy ||
	       address >= columbus_ipc.sram_phy + COLUMBUS_IPC_SRAM_SIZE);

	return columbus_ipc.sram + (address - columbus_ipc.sram_phy);
}

/*
 *  Note: In the current IPC ip design, the sender could trigger the receiver's
 *  interrupt, but the receiver could not trigger the sender's interrupt.
 *  The sender could know the received has retrived the message by polling
 *  A7TORFIPCFLG / A7TOPLCIPCFLG is unset.
 *
*/
int columbus_ipc_send_message(channel_handle channel,
			      char *message,
			      size_t len,
			      int page_num)
{
	char *sram;
	int channel_num = channel2num(channel);
	struct ipc_channel *channel_2 = (struct ipc_channel *)channel;
	u32 flag_offset;
	u32 ack_offset;
	u32 ipc_flag;

#ifdef	DEBUG
	int count;
#endif

#ifdef COLUMBUS_IPC_CDL_INTERACTIVE
	channel_handle receive_handle;
	char	*receive_msg;
	size_t	receive_len;
#endif

	if (unlikely(len == 0))
		return	0;

	sram = ipc_sram_alloc(pagenum2pageaddr(page_num), len);

	if (unlikely(sram == NULL)) {
		ipc_dump_shared_ram_ownership();
		return	-ENOSPC;
	}

	page_num = pageaddr2pagenum(sram);

	memset(sram, 0, len);
	memcpy(sram, message, len);

	if (channel_2->partner == IPC_PARTNER_RF_DSP) {
		iowrite32(IPC_DATA_READ, columbus_ipc.io_base + A7TORFIPCCOMM);

		/*
		 * send the physical address of the message from A7 view,
		 * in fact, currently, the code of dsp side skips the
		 * parameter.
		 */
		iowrite32(columbus_ipc.sram_phy +
			  page_num * COLUMBUS_IPC_PAGE_SIZE,
			  columbus_ipc.io_base + A7TORFIPCADDR);

		iowrite32(len, columbus_ipc.io_base + A7TORFIPCDATA0);

		/*
		 *  dsp side code will extract the page number to get the
		 *  messageaddress
		 */
		iowrite32(page_num | (IPC_END_MSG << 16),
			  columbus_ipc.io_base + A7TORFIPCDATA1);

		/*
		 * RF DSP partner will set A7TORFIPCACK to make clear
		 * A7TORFIPCFLAG
		 */

		flag_offset = A7TORFIPCFLG;
		ack_offset = RFTOA7IPCACK;

	} else {
		iowrite32(IPC_DATA_READ, columbus_ipc.io_base + A7TOPLCIPCCOMM);

		/*
		 *  send the physical address of the message from A7 view,
		 *  in fact, currently, the code of dsp side skips the
		 *  parameter.
		 */
		iowrite32(
			  columbus_ipc.sram_phy +
			  page_num * COLUMBUS_IPC_PAGE_SIZE,
			  columbus_ipc.io_base + A7TOPLCIPCADDR);

		iowrite32(len, columbus_ipc.io_base + A7TOPLCIPCDATA0);
		iowrite32(page_num | (IPC_END_MSG << 16),
				  columbus_ipc.io_base + A7TOPLCIPCDATA1);

		/*
		 * PLC DSP partner will set A7TOPLCIPCACK to make clear
		 * A7TOPLCIPCFLAG
		 */
		flag_offset = A7TOPLCIPCFLG;
		ack_offset = PLCTOA7IPCACK;
	}

	/*
	 * The current IPC IP design doesn't support the partner that received
	 * message to trigger A7's interrupt by ACKing A7TORFIPCACK. So I must
	 * poll the A7TORFIPCFLG. The designer say maybe the next release could
	 * provide the feature.
	 *
	 */

	dev_dbg(columbus_ipc.dev, "send message to dsp.\n");

	/* trigger the receiver's interrupt */
	notify_partner(channel);

	ipc_flag = 1 << channel_num;

	/*
	 * The DSP partner is responsible for notifying ARM A7 it has received
	 * the message by ACK, the ACK will clear A7TOXXXIPCFLAG and exit the
	 * loop. If could not exit loop, please check DSP side code.
	 */
	while (ipc_flag) {
		ipc_flag = ioread32(columbus_ipc.io_base + flag_offset);
	#ifdef	DEBUG
		for (count = 0; count % 10 == 0; count++)
			dev_dbg(columbus_ipc.dev, "wait dsp's ack ...\n");
	#endif

		cond_resched();
	}

	iowrite32(1 << channel_num, columbus_ipc.io_base + ack_offset);

	dev_dbg(columbus_ipc.dev, "dsp has received the message.\n");

	ipc_sram_free(pagenum2pageaddr(page_num), len);

	return  len;
}
EXPORT_SYMBOL(columbus_ipc_send_message);


/*
 * The received message will be returned in *message buffer, the message size
 * is returned in *len. The message buffer is allocated in the callee, the
 * caller is responsible for freeing the allocated message buffer by kfree.
 *
 * Note: the invoker of columbus_ipc_receive_message() is responsible for
 * freeing the *message by kfree().
*/
int columbus_ipc_receive_message(channel_handle channel,
				 char **message,
				 size_t *len)
{
	char *msg_buf;
	int channel_num = channel2num(channel);
	struct ipc_channel *channel_2 = (struct ipc_channel *)channel;
	struct completion *sync;
	unsigned int virq;
	int offset = 0;
	u32 ipc_status;

	u32 command, address, data0, data1;
	phys_addr_t	msg_addr_from_a7_view = 0;
	void __iomem *msg = NULL;

	/* Firstly, ARM need ack RFTOA7IPCACK or PLCTOA7IPCACK */

	if (channel_2->partner == IPC_PARTNER_RF_DSP) {
		iowrite32(1 << channel_num,
			  columbus_ipc.io_base + RFTOA7IPCACK);
	} else {
		iowrite32(1 << channel_num,
			  columbus_ipc.io_base + PLCTOA7IPCACK);
	}

	if (channel_2->mode == IPC_COMMUNICATION_INT) {
		/*
		 * receive message by interrupt mode, the sender will need
		 * trigger receiver's interrupt
		 */

		if (channel_2->partner == IPC_PARTNER_PLC_DSP)
			offset += IPC_IRQ_CHANNEL_NUM;

		sync = &(columbus_ipc.ipc_irq[channel_num + offset].irq_done);

		init_completion(sync);

		virq = columbus_ipc.ipc_irq[offset + channel_num].irq_from_dsp;
		enable_irq(virq);

		/* wait the dsp partner send message to a7. */
		wait_for_completion(sync);

		disable_irq(virq);

	} else {
		/* receive message by poll mode */
		IPC_BUG(channel_2->mode != IPC_COMMUNICATION_POLL);

		/*
		 *  RF DSP partner will set RFTOA7IPCSET register to make
		 *  XXXTOA7IPCSTS "set" for A7.
		 */
		do {
			if (channel_2->partner == IPC_PARTNER_RF_DSP)
				ipc_status = ioread32(columbus_ipc.io_base +
						      RFTOA7IPCSTS);
			else
				ipc_status = ioread32(columbus_ipc.io_base +
						      PLCTOA7IPCSTS);

		} while ((ipc_status & (1 << channel_num)) == 0);
	}

	/* retrieve the message */
	if (channel_2->partner == IPC_PARTNER_RF_DSP) {
		command = ioread32(columbus_ipc.io_base + RFTOA7IPCCOMM);
		address = ioread32(columbus_ipc.io_base + RFTOA7IPCADDR);
		data0 = ioread32(columbus_ipc.io_base + RFTOA7IPCDATA0);
		data1 = ioread32(columbus_ipc.io_base + RFTOA7IPCDATA1);
	} else {
		command = ioread32(columbus_ipc.io_base + PLCTOA7IPCCOMM);
		address = ioread32(columbus_ipc.io_base + PLCTOA7IPCADDR);
		data0 = ioread32(columbus_ipc.io_base + PLCTOA7IPCDATA0);
		data1 = ioread32(columbus_ipc.io_base + PLCTOA7IPCDATA1);
	}

	if (command == IPC_DATA_READ) {
		IPC_BUG(is_valid_address(channel, address) == 0);

		/*
		 * The "address" is from sender's view, need convert to
		 * A7's view
		 */
		msg_addr_from_a7_view = address_from_a7_view(channel, address);
		msg = phy2vir(msg_addr_from_a7_view);

		IPC_BUG(data0 > COLUMBUS_IPC_SRAM_SIZE);

		/* Note: DON'T FORGET FREE THE FOLLOWING MEMORY !!! */
		msg_buf = kmalloc(data0, GFP_KERNEL);
		if (unlikely(msg_buf == NULL)) {
			dev_err(columbus_ipc.dev,
				"kmalloc fail: %s-%d\n",
				__func__,
				__LINE__);

			return	-ENOMEM;
		}

		if ((data1 >> 16) == IPC_END_MSG)
			memcpy(msg_buf, msg, data0);
	}

	*message = msg_buf;
	*len = data0;

	return  *len;
}
EXPORT_SYMBOL(columbus_ipc_receive_message);


/*
 * create virtual files in /sys/class/columbus_ipc/ipc directory
 *
 * columbus_ipc driver support the following interfaces files in /sys
 *
 * partner		: ARM A7 core want to communicate with which dsp core ?
 *              : 0 = RF DSP, 1 = PLC DSP
 * operation	: Send or Receive
 *              : 0 = send, 1 = Receive
 * channel	    : communication by which channel, 0 - 15, total 16 channels
 *              : 0- 7 channels support interrupt and poll modes to
 *              : communication; but 8 - 15 channels only support poll mode
 *              :
 * mode         : Receive message by interrupt or poll mode
 *              : 0 - interrupt, 1 - poll
 * message      : message file
 *
 * scenario 1   : send message in msg.bin file to RF DSP by channel 3 with
 *              : interrupt mode
 *
 * echo partner > 0
 * echo operation > 0
 * echo channel > 3
 * cat msg.bin > message
 *
 *
 * scenario 2   : receive message in msg.bin file from PLC DSP by channel 6
 *              : with poll mode
 *
 * echo partner > 1
 * echo operation > 1
 * echo mode > 1
 * echo channel > 6
 * cat message > msg.bin
 *
 * scenario 3   :
 *
*/

static ssize_t columbus_ipc_partner_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
static ssize_t columbus_ipc_partner_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len);
static DEVICE_ATTR(partner, S_IRUGO | S_IWUSR, columbus_ipc_partner_read,
		   columbus_ipc_partner_write);


static ssize_t columbus_ipc_operation_read(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);
static ssize_t columbus_ipc_operation_write(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t len);
static DEVICE_ATTR(operation, S_IRUGO | S_IWUSR, columbus_ipc_operation_read,
		   columbus_ipc_operation_write);



static ssize_t columbus_ipc_mode_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);
static ssize_t columbus_ipc_mode_write(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t len);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, columbus_ipc_mode_read,
		   columbus_ipc_mode_write);


static ssize_t columbus_ipc_channel_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
static ssize_t columbus_ipc_channel_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len);
static DEVICE_ATTR(channel, S_IRUGO | S_IWUSR, columbus_ipc_channel_read,
		   columbus_ipc_channel_write);


static ssize_t page_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t len);
static DEVICE_ATTR_WO(page);



static ssize_t columbus_ipc_message_read(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *a,
					 char *src,
					 loff_t src_off,
					 size_t src_size);


static ssize_t columbus_ipc_message_write(struct file *filp,
					  struct kobject *kobj,
					  struct bin_attribute *a,
					  char *src,
					  loff_t src_off,
					  size_t src_size);

static struct bin_attribute ipc_message = {
	.attr = { .name = "message", .mode = S_IRUGO | S_IWUSR, },
	.read = columbus_ipc_message_read,
	.write = columbus_ipc_message_write,
};

static struct attribute *columbus_ipc_attrs[] = {
	&dev_attr_partner.attr,
	&dev_attr_operation.attr,
	&dev_attr_mode.attr,
	&dev_attr_channel.attr,
	&dev_attr_page.attr,
	NULL,
};

static struct bin_attribute *columbus_ipc_bin_attrs[] = {
	&ipc_message,
	NULL,
};

static struct attribute_group columbus_ipc_attr_group = {
	.attrs = columbus_ipc_attrs,
	.bin_attrs = columbus_ipc_bin_attrs,
};

static const struct attribute_group *columbus_ipc_attr_groups[] = {
	&columbus_ipc_attr_group,
	NULL,
};

static struct class columbus_ipc_class = {
	.name       = "columbus_ipc",
	.owner      = THIS_MODULE,
	.dev_groups = columbus_ipc_attr_groups,
};


static ssize_t columbus_ipc_partner_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	ssize_t retval = 0;

	if (current_partner == IPC_PARTNER_RF_DSP ||
	    current_partner == IPC_PARTNER_PLC_DSP)
		retval = sprintf(buf, "%d", current_partner);
	else {
		retval = sprintf(buf, "%d", COLUMBUS_IPC_INVALID);
		IPC_BUG(current_partner != COLUMBUS_IPC_INVALID);
	}

	return  retval;
}

static ssize_t columbus_ipc_partner_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	ssize_t retval = 0;
	int     partner = -1;

	if (kstrtoint(buf, 10, &partner)) {
		dev_err(columbus_ipc.dev, "failed parsing partner: %s\n", buf);
		return  -EINVAL;
	}

	if (partner != IPC_PARTNER_RF_DSP && partner != IPC_PARTNER_PLC_DSP) {
		dev_err(columbus_ipc.dev, "invalid partner: %s\n", buf);
		return  -EINVAL;
	}

	current_partner = partner;

	retval = len;

	return  retval;
}


static ssize_t columbus_ipc_operation_read(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	ssize_t retval = 0;

	if (current_operation == IPC_SEND_OPERATION ||
	    current_operation == IPC_RECEIVE_OPERATION)
		retval = sprintf(buf, "%d", current_operation);
	else {
		retval = sprintf(buf, "%d", COLUMBUS_IPC_INVALID);
		IPC_BUG(current_operation != COLUMBUS_IPC_INVALID);
	}

	return  retval;
}

static ssize_t columbus_ipc_operation_write(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t len)
{
	ssize_t retval = 0;
	int     operation = -1;

	if (kstrtoint(buf, 10, &operation)) {
		dev_err(columbus_ipc.dev,
			"failed parsing operation: %s\n",
			buf);
		return  -EINVAL;
	}

	if (operation != IPC_SEND_OPERATION &&
	    operation != IPC_RECEIVE_OPERATION) {
		dev_err(columbus_ipc.dev, "invalid operation: %s\n", buf);
		return  -EINVAL;
	}

	current_operation = operation;

	retval = len;

	return  retval;
}


static ssize_t columbus_ipc_mode_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t retval = 0;

	if (current_mode == IPC_COMMUNICATION_INT ||
		current_mode == IPC_COMMUNICATION_POLL)
		retval = sprintf(buf, "%d", current_mode);
	else {
		retval = sprintf(buf, "%d", COLUMBUS_IPC_INVALID);
		IPC_BUG(current_mode != COLUMBUS_IPC_INVALID);
	}

	return  retval;
}

static ssize_t columbus_ipc_mode_write(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t len)
{
	ssize_t retval = 0;
	int     mode = -1;

	if (kstrtoint(buf, 10, &mode)) {
		dev_err(columbus_ipc.dev, "failed parsing mode: %s\n", buf);
		return  -EINVAL;
	}

	if (mode != IPC_COMMUNICATION_INT && mode != IPC_COMMUNICATION_POLL) {
		dev_err(columbus_ipc.dev, "invalid mode: %s\n", buf);
		return  -EINVAL;
	}

	current_mode = mode;

	retval = len;

	return  retval;
}

static ssize_t columbus_ipc_channel_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	ssize_t retval = 0;

	if (current_channel >= 0 && current_channel < A7_RF_IPC_CHANNEL_NUM)
		retval = sprintf(buf, "%d", current_channel);
	else {
		IPC_BUG(current_channel != COLUMBUS_IPC_INVALID);
		retval = sprintf(buf, "%d", current_channel);
	}

	return  retval;
}

static ssize_t columbus_ipc_channel_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	ssize_t retval = 0;
	int     channel;

	if (kstrtoint(buf, 10, &channel)) {
		dev_err(columbus_ipc.dev, "failed parsing channel: %s\n", buf);
		return  -EINVAL;
	}

	if (current_mode == COLUMBUS_IPC_INVALID) {
		dev_err(columbus_ipc.dev, "Please set correct mode\n");
		return  -EINVAL;
	}

	if (current_mode == IPC_COMMUNICATION_POLL &&
		channel >= A7_RF_IPC_CHANNEL_NUM) {
		dev_err(columbus_ipc.dev, "invalid channel: %s\n", buf);
		return  -EINVAL;
	}

	if (current_mode == IPC_COMMUNICATION_INT &&
		channel >= IPC_IRQ_CHANNEL_NUM) {
		dev_err(columbus_ipc.dev, "invalid channel: %s\n", buf);
		return  -EINVAL;
	}

	current_channel = channel;

	retval = len;

	return  retval;
}


static ssize_t page_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t len)
{
	ssize_t retval = 0;
	int	page;

	if (kstrtoint(buf, 10, &page)) {
		dev_err(columbus_ipc.dev, "failed parsing page: %s\n", buf);
		return  -EINVAL;
	}

	current_page = page;

	retval = len;

	return	retval;
}


static ssize_t columbus_ipc_message_read(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *a,
					 char *src,
					 loff_t src_off,
					 size_t src_size)
{
	ssize_t		retval = 0;
	channel_handle  handle;
	char		*msg;
	size_t		len;

	if (src_off != 0) {
		return	0;
	}

	handle = columbus_ipc_get_channel(current_partner,
					  current_operation,
					  current_mode,
					  current_channel);
	if (unlikely(handle == NULL)) {
		dev_err(columbus_ipc.dev, "invalid channel params:"
					  "partner = %d, "
					  "operation = %d, "
					  "mode = %d, "
					  "channel = %d\n",
			current_partner, current_operation, current_mode,
			current_channel);

		return  -EINVAL;
	}

	columbus_ipc_receive_message(handle, &msg, &len);

	IPC_BUG(len > src_size);

	memcpy(src, msg, len);

	kfree(msg);

	columbus_ipc_put_channel(handle);

	retval = len;
	return  retval;
}


static ssize_t columbus_ipc_message_write(struct file *filp,
					  struct kobject *kobj,
					  struct bin_attribute *a,
					  char *src,
					  loff_t src_off,
					  size_t src_size)
{
	ssize_t         retval = 0;
	channel_handle  handle;

	handle = columbus_ipc_get_channel(current_partner,
					  current_operation,
					  current_mode,
					  current_channel);
	if (unlikely(handle == NULL)) {
		dev_err(columbus_ipc.dev, "invalid channel params: "
					  "partner = %d,"
					  "operation = %d, "
					  "mode = %d, "
					  "channel = %d\n",
			current_partner,
			current_operation,
			current_mode,
			current_channel);
		return  -EINVAL;
	}

	retval = columbus_ipc_send_message(handle,
					   src,
					   src_size,
					   current_page);

	columbus_ipc_put_channel(handle);

	return  retval;
}


#ifdef CONFIG_DEBUG_FS
static struct dentry	*columbus_ipc_debugfs;
struct debugfs_regset32 columbus_ipc_regset;

static const struct debugfs_reg32 columbus_ipc_regs[] = {
	/* ARM A7 -> RF DSP, W/R */
	{ "A7TORFIPCCOMM",	A7TORFIPCCOMM },
	{ "A7TORFIPCADDR",	A7TORFIPCADDR },
	{ "A7TORFIPCDATA0",	A7TORFIPCDATA0 },
	{ "A7TORFIPCDATA1",	A7TORFIPCDATA1 },
	/* ARM A7 -> PLC DSP, W/R */
	{ "A7TOPLCIPCCOMM",	A7TOPLCIPCCOMM },
	{ "A7TOPLCIPCADDR",	A7TOPLCIPCADDR },
	{ "A7TOPLCIPCDATA0",	A7TOPLCIPCDATA0 },
	{ "A7TOPLCIPCDATA1",	A7TOPLCIPCDATA1 },
	/* RF DSP -> ARM A7 , ReadOnly */
	{ "RFTOA7IPCCOMM",	RFTOA7IPCCOMM },
	{ "RFTOA7IPCADDR",	RFTOA7IPCADDR },
	{ "RFTOA7IPCDATA0",	RFTOA7IPCDATA0 },
	{ "RFTOA7IPCDATA1",	RFTOA7IPCDATA1 },
	/* PLC DSP -> ARM A7 , ReadOnly */
	{ "PLCTOA7IPCCOMM",	PLCTOA7IPCCOMM },
	{ "PLCTOA7IPCADDR",	PLCTOA7IPCADDR },
	{ "PLCTOA7IPCDATA0",	PLCTOA7IPCDATA0 },
	{ "PLCTOA7IPCDATA1",	PLCTOA7IPCDATA1 },
	/* 64-bit timestamp counter */
	{ "IPCCOUNTERL",	IPCCOUNTERL },
	{ "IPCCOUNTERH",	IPCCOUNTERH },
	/* Shared RAM Page's ownership status */
	{ "SRMSEL0",		SRMSEL0 },
	{ "SRMSEL1",		SRMSEL1 },
	/* ARM A7 -> RF DSP */
	{ "A7TORFIPCSET",	A7TORFIPCSET },
	{ "A7TORFIPCCLR",	A7TORFIPCCLR },
	{ "A7TORFIPCFLG",	A7TORFIPCFLG },
	{ "RFTOA7IPCACK",	RFTOA7IPCACK },
	{ "RFTOA7IPCSTS",	RFTOA7IPCSTS },
	/* ARM A7 -> PLC DSP */
	{ "A7TOPLCIPCSET",	A7TOPLCIPCSET },
	{ "A7TOPLCIPCCLR",	A7TOPLCIPCCLR },
	{ "A7TOPLCIPCFLG",	A7TOPLCIPCFLG },
	{ "PLCTOA7IPCACK",	PLCTOA7IPCACK },
	{ "PLCTOA7IPCSTS",	PLCTOA7IPCSTS },
	/* Shared RAM ownership request semaphore */
	{ "A7SRP00REQ",		A7SRP00REQ },
	{ "A7SRP01REQ",		A7SRP01REQ },
	{ "A7SRP02REQ",		A7SRP02REQ },
	{ "A7SRP03REQ",		A7SRP03REQ },
	{ "A7SRP04REQ",		A7SRP04REQ },
	{ "A7SRP05REQ",		A7SRP05REQ },
	{ "A7SRP06REQ",		A7SRP06REQ },
	{ "A7SRP07REQ",		A7SRP07REQ },
	{ "A7SRP08REQ",		A7SRP08REQ },
	{ "A7SRP09REQ",		A7SRP09REQ },
	{ "A7SRP10REQ",		A7SRP10REQ },
	{ "A7SRP11REQ",		A7SRP11REQ },
	{ "A7SRP12REQ",		A7SRP12REQ },
	{ "A7SRP13REQ",		A7SRP13REQ },
	{ "A7SRP14REQ",		A7SRP14REQ },
	{ "A7SRP15REQ",		A7SRP15REQ },
	{ "A7SRP16REQ",		A7SRP16REQ },
	{ "A7SRP17REQ",		A7SRP17REQ },
	{ "A7SRP18REQ",		A7SRP18REQ },
	{ "A7SRP19REQ",		A7SRP19REQ },
	{ "A7SRP20REQ",		A7SRP20REQ },
	{ "A7SRP21REQ",		A7SRP21REQ },
	{ "A7SRP22REQ",		A7SRP22REQ },
	{ "A7SRP23REQ",		A7SRP23REQ },
	{ "A7SRP24REQ",		A7SRP24REQ },
	{ "A7SRP25REQ",		A7SRP25REQ },
	{ "A7SRP26REQ",		A7SRP26REQ },
	{ "A7SRP27REQ",		A7SRP27REQ },
	{ "A7SRP28REQ",		A7SRP28REQ },
	{ "A7SRP29REQ",		A7SRP29REQ },
	{ "A7SRP30REQ",		A7SRP30REQ },
	{ "A7SRP31REQ",		A7SRP31REQ },
};

void columbus_ipc_regdump_create(void)
{
	struct dentry *file;

	columbus_ipc_debugfs = debugfs_create_dir("columbus_ipc", NULL);
	IPC_BUG(IS_ERR(columbus_ipc_debugfs));

	columbus_ipc_regset.regs = columbus_ipc_regs;
	columbus_ipc_regset.nregs = ARRAY_SIZE(columbus_ipc_regs);
	IPC_BUG(columbus_ipc.io_base == NULL);
	columbus_ipc_regset.base = columbus_ipc.io_base;

	file = debugfs_create_regset32("regdump",
				       S_IRUGO,
				       columbus_ipc_debugfs,
				       &columbus_ipc_regset);
	if (!file) {
		debugfs_remove_recursive(columbus_ipc_debugfs);
		pr_err("fail to create debugfs entry!\n");
	}
}

void columbus_ipc_regdump_destroy(void)
{
	IPC_BUG(columbus_ipc.io_base == NULL);
	debugfs_remove_recursive(columbus_ipc_debugfs);
}

#else

void columbus_ipc_regdump_create(void)
{
}

void columbus_ipc_regdump_destroy(void)
{
}

#endif

/* DSP partner (RF / LPC) will trigger the ARM A7's interrupt */
static irqreturn_t columbus_ipc_irq_handler(int irq, void *private)
{
	/* 0-7 is for RF DSP, 8 - 15 is for PLC DSP. */
	int int_channel_num = (int)private;

	int channel_num;
	struct ipc_channel *channel;
	struct completion *done;
	u32 ack_offset;

	dev_dbg(columbus_ipc.dev, "in ipc isr\n");

	IPC_BUG(int_channel_num < 0);
	IPC_BUG(int_channel_num >=
		IPC_IRQ_CHANNEL_NUM + IPC_IRQ_CHANNEL_NUM);

	if (int_channel_num < IPC_IRQ_CHANNEL_NUM) {
		/* The RF DSP triggers the interrupt */
		channel_num = int_channel_num;
		channel = &ipc_rf_channel[channel_num];
		IPC_BUG(channel->partner != IPC_PARTNER_RF_DSP);
		ack_offset = RFTOA7IPCACK;
	} else {
		/* The PLC DSP trigger the interrupt */
		channel_num = int_channel_num - IPC_IRQ_CHANNEL_NUM;
		channel = &ipc_plc_channel[channel_num];
		IPC_BUG(channel->partner != IPC_PARTNER_PLC_DSP);
		ack_offset = PLCTOA7IPCACK;
	}

	IPC_BUG(channel->used != IPC_CHANNEL_USED);
	IPC_BUG(channel->mode != IPC_COMMUNICATION_INT);

	if (channel->operation == IPC_SEND_OPERATION) {
		/* The current IPC IP hardware doesn't support the feature. */

		/*
		 * ARM A7 send message to DSP partner, the partner has received
		 * the message and trigger the A7's interrupt.
		 */
		dev_dbg(columbus_ipc.dev,
			"dsp has received message successfully.\n");
	} else {
		/*
		 * THe DSP partner want to send message to ARM A7 core,
		 * the message is ready, the partner notify the A7 by
		 * triggering the interrupt.
		 */

		IPC_BUG(channel->operation != IPC_RECEIVE_OPERATION);
		dev_dbg(columbus_ipc.dev, "dsp send message to a7.\n");
	}

	done = &(columbus_ipc.ipc_irq[int_channel_num].irq_done);
	complete(done);


	/* clear the interrupt. */
	iowrite32(1 << channel_num, columbus_ipc.io_base + ack_offset);

	dev_dbg(columbus_ipc.dev, "out ipc isr\n");

	return IRQ_HANDLED;
}

#ifdef COLUMBUS_IPC_MISC_DEVICE

/* State information that is tracked on a per-client basis */
struct instance_state {
	/* simply an example of something that could be tracked. */
	uint32_t ioctl_access_cnt;
};

/*
 * This function is called whenever a client opens the driver's device node.
 */
static int ipc_open(struct inode *ind, struct file *filp)
{
	struct instance_state *s;

	open_count++;
	s = kmalloc(sizeof(struct instance_state), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->ioctl_access_cnt = 0;
	filp->private_data = s;

	return 0;
}


/*
 * This function is called whenever a client closes its connection to the
 * driver.
 */
static int ipc_close(struct inode *ind, struct file *filp)
{
	open_count--;
	kfree(filp->private_data);	/* Free instance_state */
	filp->private_data = NULL;
	return 0;
}


/*
 * Handles all ioctl() calls.  There is nothing magic about the
 * cmd numbers -- they simply need to be unique within a particular driver
 */
static long ipc_ioctl(struct file *filp,
			 unsigned int cmd,
			 unsigned long args)
{
	int32_t rc = -EINVAL;
	struct instance_state *s = (struct instance_state *)filp->private_data;

	s->ioctl_access_cnt++;

	switch (cmd) {
	/* IOCTL handlers */
	default:
		break;
	}

	return rc;
}

static const struct file_operations ipc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = ipc_ioctl,
	.open		= ipc_open,
	.release	= ipc_close,
};

#endif

#ifdef COLUMBUS_IPC_UNITTEST

#define IPC_UNITTEST_ASSERT(condition) do {	\
	if (!condition)				\
		IPC_BUG(1);		\
} while (0)

/*
 * The unittest suppose, only ARM A7 core manipulate the shared ram.
*/

static void unittest_make_sram_free(void)
{
	int		page_num;
	enum ownership	status;

	for (page_num = 0; page_num < SHARED_RAM_PAGE_NUM; page_num++) {
		status = get_sram_page_ownership(page_num);
		if (status == ownership_a7)
			release_one_sram_page(page_num);
	}
}

static void unittest_check_sram_free(void)
{
	int		page_num;
	enum ownership	status;

	for (page_num = 0; page_num < SHARED_RAM_PAGE_NUM; page_num++) {
		status = get_sram_page_ownership(page_num);
		IPC_UNITTEST_ASSERT((status == ownership_free));
	}
}

static void unittest_sram_operation(void)
{
	int i;
	int page_num[SHARED_RAM_PAGE_NUM];
	int start_page;
	int result;
	char *page;
	char *page2;
	enum ownership status;

	unittest_check_sram_free();

	for (i = 0; i < SHARED_RAM_PAGE_NUM; i++)
		IPC_UNITTEST_ASSERT((grab_one_sram_page(i) == 1));

	for (i = 0; i < SHARED_RAM_PAGE_NUM; i++) {
		status = get_sram_page_ownership(i);
		IPC_UNITTEST_ASSERT((ownership_a7 == status));
	}

	for (i = 0; i < SHARED_RAM_PAGE_NUM; i++)
		release_one_sram_page(i);

	unittest_check_sram_free();

	/*********************************************************/

	page_num[0]	= 0;
	page_num[1]	= 0;
	page_num[2]	= 1;
	page_num[3]	= 0;
	page_num[4]	= 0;
	page_num[5]	= 0;
	page_num[6]	= 1;
	page_num[7]	= 0;
	page_num[8]	= 0;
	page_num[9]	= 0;
	page_num[10]	= 0;
	page_num[11]	= 1;
	page_num[12]	= 0;
	page_num[13]	= 0;
	page_num[14]	= 0;
	page_num[15]	= 0;
	page_num[16]	= 0;
	page_num[17]	= 1;
	page_num[18]	= 0;
	page_num[19]	= 0;
	page_num[20]	= 0;
	page_num[21]	= 0;
	page_num[22]	= 0;
	page_num[23]	= 0;
	page_num[24]	= 1;
	page_num[25]	= 0;
	page_num[26]	= 0;
	page_num[27]	= 0;
	page_num[28]	= 0;
	page_num[29]	= 0;
	page_num[30]	= 0;
	page_num[31]	= 0;

	for (i = 0; i < SHARED_RAM_PAGE_NUM; i++) {
		if (page_num[i])
			IPC_UNITTEST_ASSERT((grab_one_sram_page(i) == 1));
	}

	result = find_consecutive_free_sram_pages(2, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 0));

	result = find_consecutive_free_sram_pages(3, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 3));

	result = find_consecutive_free_sram_pages(4, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 7));

	result = find_consecutive_free_sram_pages(5, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 12));

	result = find_consecutive_free_sram_pages(6, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 18));

	result = find_consecutive_free_sram_pages(7, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 25));

	free_sram_pages(2, 1);
	page_num[2] = 0;

	result = find_consecutive_free_sram_pages(6, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 0));

	free_sram_pages(24, 1);
	page_num[24] = 0;

	result = find_consecutive_free_sram_pages(14, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 18));

	result = find_consecutive_free_sram_pages(15, &start_page);
	IPC_UNITTEST_ASSERT((result == 0));

	/* free all occupied sram */
	for (i = 0; i < SHARED_RAM_PAGE_NUM; i++) {
		if (page_num[i]) {
			release_one_sram_page(i);
			page_num[i] = 0;
		}
	}

	result = find_consecutive_free_sram_pages(32, &start_page);
	IPC_UNITTEST_ASSERT((result == 1));
	IPC_UNITTEST_ASSERT((start_page == 0));

	IPC_UNITTEST_ASSERT((try_to_grab_sram_pages(-1, 32) == 0));

	free_sram_pages(0, 32);

	IPC_UNITTEST_ASSERT((grab_one_sram_page(0) == 1));
	IPC_UNITTEST_ASSERT((try_to_grab_sram_pages(-1, 32) == -1));
	IPC_UNITTEST_ASSERT((try_to_grab_sram_pages(0, 1) == -1));
	IPC_UNITTEST_ASSERT((try_to_grab_sram_pages(0, 3) == -1));
	IPC_UNITTEST_ASSERT((try_to_grab_sram_pages(1, 3) == 1));

	/*********************************************************/

	unittest_make_sram_free();

	page = ipc_sram_alloc(NULL, 1000);
	IPC_UNITTEST_ASSERT((columbus_ipc.sram == page));

	status = get_sram_page_ownership(0);
	IPC_UNITTEST_ASSERT((status == ownership_a7));
	status = get_sram_page_ownership(1);
	IPC_UNITTEST_ASSERT((status == ownership_free));

	ipc_sram_free(page, 1000);
	status = get_sram_page_ownership(0);
	IPC_UNITTEST_ASSERT((status == ownership_free));

	page = ipc_sram_alloc(pagenum2pageaddr(3), 1000);
	IPC_UNITTEST_ASSERT((page ==
			     columbus_ipc.sram + COLUMBUS_IPC_PAGE_SIZE * 3));

	page2 = ipc_sram_alloc(pagenum2pageaddr(2), 2000);
	IPC_UNITTEST_ASSERT((page2 == NULL));

	page2 = ipc_sram_alloc(NULL, 2000);
	IPC_UNITTEST_ASSERT((page2 == columbus_ipc.sram));

	page2 = ipc_sram_alloc(pagenum2pageaddr(16), 7000);
	IPC_UNITTEST_ASSERT((page2 ==
			     columbus_ipc.sram + COLUMBUS_IPC_PAGE_SIZE * 16));

	ipc_sram_free(page2, 7000);

	page2 = ipc_sram_alloc(pagenum2pageaddr(16), 1000);
	IPC_UNITTEST_ASSERT((page2 ==
			     columbus_ipc.sram + COLUMBUS_IPC_PAGE_SIZE * 16));

	unittest_make_sram_free();
}

static void unittest_channel_operation(void)
{
	int i;
	channel_handle channel;
	channel_handle channel2;
	int channel_num;
	channel_handle backup[16];

	channel = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					   IPC_SEND_OPERATION,
					   IPC_COMMUNICATION_POLL,
					   COLUMBUS_IPC_INVALID);
	IPC_UNITTEST_ASSERT(channel);

	channel_num = channel2num(channel);

	channel2 = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					    IPC_SEND_OPERATION,
					    IPC_COMMUNICATION_POLL,
					    channel_num);
	IPC_UNITTEST_ASSERT((channel2 == NULL));
	columbus_ipc_put_channel(channel);

	channel2 = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					    IPC_SEND_OPERATION,
					    IPC_COMMUNICATION_POLL,
					    channel_num);
	IPC_UNITTEST_ASSERT(channel2);
	IPC_UNITTEST_ASSERT((channel2num(channel2) == channel_num));
	columbus_ipc_put_channel(channel2);

	for (i = 0; i < IPC_IRQ_CHANNEL_NUM; i++) {
		backup[i] = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
						     IPC_RECEIVE_OPERATION,
						     IPC_COMMUNICATION_INT,
						     COLUMBUS_IPC_INVALID);
		IPC_UNITTEST_ASSERT((channel2num(backup[i]) == i));
	}

	channel = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					   IPC_RECEIVE_OPERATION,
					   IPC_COMMUNICATION_INT,
					   COLUMBUS_IPC_INVALID);
	IPC_UNITTEST_ASSERT((channel == NULL));

	channel = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					   IPC_RECEIVE_OPERATION,
					   IPC_COMMUNICATION_POLL,
					   COLUMBUS_IPC_INVALID);
	IPC_UNITTEST_ASSERT(channel);

	columbus_ipc_put_channel(channel);
	for (i = 0; i < IPC_IRQ_CHANNEL_NUM; i++)
		columbus_ipc_put_channel(backup[i]);

	for (i = 0; i < A7_RF_IPC_CHANNEL_NUM; i++) {
		backup[i] = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
						     IPC_RECEIVE_OPERATION,
						     IPC_COMMUNICATION_POLL,
						     COLUMBUS_IPC_INVALID);
		IPC_UNITTEST_ASSERT((channel2num(backup[i]) == i));
	}

	channel = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					   IPC_RECEIVE_OPERATION,
					   IPC_COMMUNICATION_POLL,
					   COLUMBUS_IPC_INVALID);
	IPC_UNITTEST_ASSERT((channel == NULL));

	channel = columbus_ipc_get_channel(IPC_PARTNER_RF_DSP,
					   IPC_RECEIVE_OPERATION,
					   IPC_COMMUNICATION_INT,
					   COLUMBUS_IPC_INVALID);
	IPC_UNITTEST_ASSERT((channel == NULL));

	for (i = 0; i < A7_RF_IPC_CHANNEL_NUM; i++)
		columbus_ipc_put_channel(backup[i]);
}

static void columbus_ipc_unittest(void)
{
	dev_dbg(columbus_ipc.dev, "start ipc unit test ...\n");
	unittest_sram_operation();
	unittest_channel_operation();
	dev_dbg(columbus_ipc.dev, "complete ipc unit test.\n");
}

#else
static void columbus_ipc_unittest(void) {}
#endif

static struct device *columbus_ipc_device;

static int columbus_ipc_probe(struct platform_device *pdev)
{
	struct device_node	*node = pdev->dev.of_node;
	void __iomem		*base;
	struct resource		*res;
	int			err = 0;
	int			channel_num;
	unsigned int        virq;

	struct              resource irq_res;
	int                 irq_dummy;

	dev_info(&pdev->dev, "probe columbus ipc hardware!\n");

	memset(&columbus_ipc, 0, sizeof(columbus_ipc));

	/* remap ipc's io space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	IPC_BUG(!res);

	dev_dbg(&pdev->dev,
		"columbus ipc io address(physical): 0x%0X\n",
		res->start);

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		dev_dbg(&pdev->dev, "Couldn't remap ipc io space!\n");
		err = PTR_ERR(base);
		return  err;
	}
	columbus_ipc.io_base = base;
	dev_info(&pdev->dev,
		 "columbus ipc io address: 0x%p\n",
		 columbus_ipc.io_base);

	/* remap ipc's shared RAM space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	IPC_BUG(!res);

	columbus_ipc.sram_phy = res->start;

	dev_dbg(&pdev->dev,
		"columbus ipc shared ram address (physical): 0x%0X\n",
		res->start);

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		dev_dbg(&pdev->dev, "Couldn't remap ipc's shared RAM!\n");
		err = PTR_ERR(base);
		return  err;
	}
	columbus_ipc.sram = base;
	dev_info(&pdev->dev,
		 "columbus ipc shared ram address: 0x%p\n",
		 columbus_ipc.sram);

	/* acquire DSPs(RF and LPC) to ARM A7's interrupt number */
	for (channel_num = 0;
	      channel_num < IPC_IRQ_CHANNEL_NUM + IPC_IRQ_CHANNEL_NUM;
	      channel_num++) {
		virq = irq_of_parse_and_map(node, channel_num);
		IPC_BUG(virq == 0);

		irq_dummy = of_irq_to_resource(node, channel_num, &irq_res);
		IPC_BUG(irq_dummy != virq);
		IPC_BUG(irq_res.name == NULL);

		err = devm_request_irq(&pdev->dev,
				       virq,
				       columbus_ipc_irq_handler,
				       0,
				       irq_res.name,
				       (void *)channel_num);

		if (err < 0) {
			dev_err(&pdev->dev, "Failed to allocate IRQ.\n");
			return  err;
		}

		columbus_ipc.ipc_irq[channel_num].irq_from_dsp = virq;

		disable_irq(virq);
	}

	columbus_ipc.dev = &pdev->dev;

	/* create user interface */
	class_register(&columbus_ipc_class);
	columbus_ipc_device = device_create(&columbus_ipc_class,
					    NULL,
					    MKDEV(0, 0),
					    NULL,
					    "ipc");
	if (IS_ERR(columbus_ipc_device)) {
		dev_err(&pdev->dev,
			"failed to create device for columbus ipc\n");
		err = PTR_ERR(columbus_ipc_device);

		class_unregister(&columbus_ipc_class);

		return err;
	}

	columbus_ipc_regdump_create();

#ifdef COLUMBUS_IPC_MISC_DEVICE
	miscdev.minor = MISC_DYNAMIC_MINOR;
	miscdev.name  = COLUMBUS_IPC_NAME;
	miscdev.fops  = &ipc_fops;
	misc_register(&miscdev);
#endif

	columbus_ipc_unittest();

	dev_info(&pdev->dev, "probe columbus ipc successfully!\n");

	return 0;
}

static int __exit columbus_ipc_remove(struct platform_device *pdev)
{
	int	            channel_num;
	unsigned int    virq;

	columbus_ipc_regdump_destroy();

	device_unregister(columbus_ipc_device);

	class_unregister(&columbus_ipc_class);

	for (channel_num = 0;
	      channel_num < IPC_IRQ_CHANNEL_NUM + IPC_IRQ_CHANNEL_NUM;
	      channel_num++) {
		virq = columbus_ipc.ipc_irq[channel_num].irq_from_dsp;
		free_irq(virq, (void *)channel_num);
	}

#ifdef COLUMBUS_IPC_MISC_DEVICE
	misc_deregister(&miscdev);
#endif

	return 0;
}

static void columbus_ipc_shutdown(struct platform_device *pdev)
{
	/* do nothing currently */
}

static int columbus_ipc_suspend_noirq(struct device *dev)
{
	/* do nothing currently */
	return 0;
}

static int columbus_ipc_resume_noirq(struct device *dev)
{
	/* do nothing currently */
	return 0;
}

static const struct of_device_id columbus_ipc_dt_ids[] = {
	{ .compatible = "brite,columbus_ipc", },
	{}
};
MODULE_DEVICE_TABLE(of, columbus_ipc_dt_ids);

static const struct dev_pm_ops columbus_ipc_dev_pm_ops = {
	.suspend_noirq = columbus_ipc_suspend_noirq,
	.resume_noirq = columbus_ipc_resume_noirq,
};


static struct platform_driver columbus_ipc_driver = {
	.probe          = columbus_ipc_probe,
	.remove		= __exit_p(columbus_ipc_remove),
	.shutdown	= columbus_ipc_shutdown,
	.driver = {
		.name	= COLUMBUS_IPC_NAME,
		.pm	= &columbus_ipc_dev_pm_ops,
		.of_match_table = columbus_ipc_dt_ids,
	},
};

static int __init columbus_ipc_init(void)
{
	int rc;

	pr_info("columbus module init\n");

	rc = platform_driver_register(&columbus_ipc_driver);
	if (rc != 0)
		pr_err("failed to register columbus_ipc_driver %d\n", rc);

	return rc;
}
module_init(columbus_ipc_init);

static void __exit columbus_ipc_exit(void)
{
	platform_driver_unregister(&columbus_ipc_driver);
	pr_info("columbus ipc module unregistered\n");
}
module_exit(columbus_ipc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Columbus IPC driver");
