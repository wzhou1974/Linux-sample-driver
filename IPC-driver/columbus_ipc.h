/*------------------------------------------------------------------------*/
/*                                                                        */
/* Copyright (C) 2017 Brite Semiconductor Co., Ltd. All rights reserved.  */
/*                                                                        */
/*------------------------------------------------------------------------*/

#include <linux/types.h>

typedef void *channel_handle;

#define IPC_PARTNER_RF_DSP  0
#define IPC_PARTNER_PLC_DSP 1

#define IPC_SEND_OPERATION      0
#define IPC_RECEIVE_OPERATION   1

#define IPC_COMMUNICATION_INT   0
#define IPC_COMMUNICATION_POLL  1

#define COLUMBUS_IPC_INVALID    -1

channel_handle columbus_ipc_get_channel(int partner,
					int operation,
					int mode,
					int appointed_channel);


void columbus_ipc_put_channel(channel_handle channel);

int columbus_ipc_send_message(channel_handle channel,
			      char *message,
			      size_t len,
			      int page_num);

int columbus_ipc_receive_message(channel_handle channel,
				 char **message,
				 size_t *len);
