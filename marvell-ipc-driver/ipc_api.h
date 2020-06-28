/*
* ============================================================================
* Copyright (c) 2013 Marvell International, Ltd. All Rights Reserved
*
* Marvell Confidential
* ============================================================================
*/
 
/** 
* \file ipc_api.h
* 
* \brief API for accessing interprocessor 
*        communication blocks within the ASIC.
* 
* In ASIC with multiple cores IPC can be used to send messages 
* between the cores.  Each core should use a different 
* instantiation of the IPC core blocks. 
*  
**/

#ifndef INC_IPC_API_H
#define INC_IPC_API_H

#include <linux/io.h>

typedef void      *ipc_drvr_handle;
typedef void     (*ipc_recv_callback)(ipc_drvr_handle handle, void *user_param, uint8_t command, void *buffer, uint16_t length);

typedef enum 
{
    e_IPC_SUCCESS = 0,
    e_IPC_ERROR,
    e_IPC_NO_LISTENER,
} ipc_error_type_t;

/** 
 *  @brief Determine the number of IPC devices in the system
 * 
 *  @return Number of hardware IPC devices (instances)
 *  
 */
uint32_t         ipc_get_num_devices( void );

/** 
 *  @brief Determine the name of a particular IPC instance (the
 *         name is read from the DTSI file on bootup)
 *  
 *  @param device_index index of the IPC device you're
 *                      interested in.
 *  
 *  @return Name of the IPC device
 *  
 */
const char      *ipc_get_device_name( uint32_t device_index );

/** 
 *  @brief Activates the specified IPC device so it can be used
 *         for sending and receiving messages on the given port.
 * 
 *  @param device_index Index specifying which of the hardware
 *                      devices to access.
 *  
 *  @param port         Which IPC port (1-255) to attach to.
 *                      The port is a well-known number shared
 *                      between both sides allowing the IPC to
 *                      funnel traffic for a specific module.
 *  
 *  @param recv_callback Function that IPC should call when data
 *                       has been received on your port.  Any
 *                       incoming data is only valid during the
 *                       callback.  If you need it longer, make
 *                       a copy.
 *  
 *  @param user_param Arbitrary user-supplied parameter that
 *                    will be included in all calls to the
 *                    recv_callback.
 *  
 *  @return NULL on failure, otherwise, a handle to the IPC
 *          device that will be used in other calls.
 *  
 */
ipc_drvr_handle  ipc_attach( uint32_t device_index, uint8_t port_number, ipc_recv_callback recv_callback, void *user_param );

/** 
 *  @brief Informs the IPC driver that a module that attached to
 *         the IPC block no longer needs it.
 *  
 *  @param handle Handle to IPC device that was returned from
 *                ipc_attach
 *  
 *  @return OK on success, FAIL otherwise
 * 
 */
ipc_error_type_t ipc_detach( ipc_drvr_handle handle );

/** 
 *  @brief Sends data to the remote processor
 *  
 *  @param handle Handle to IPC device that was returned from
 *                ipc_attach
 *  
 *  @param command Used by caller to let the other side know how
 *                 the message should be interpreted.
 *  
 *  @param data Address of a buffer to be sent with the command.
 *              (Note - if length is set to 0 this can be used
 *              as an arbitrary 32-bit parameter)
 *  
 *  @param length Length of the buffer to send.
 *  
 *  @return OK on success, FAIL otherwise
 * 
 */
ipc_error_type_t ipc_send(ipc_drvr_handle handle, uint8_t command, void *buffer, uint16_t length);

#endif // INC_IPC_API_H

