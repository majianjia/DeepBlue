/*
 * File      : thread_canbus.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-16    majianjia   the first version
 */
 
#ifndef __THREAD_CANBUS_H__
#define __THREAD_CANBUS_H__

#include "db_can_message.h"

//·¢ËÍ
int can_message_send(CanTxMsg *TxMessage);
//½ô¼±·¢ËÍ
int can_message_urgent(CanTxMsg *TxMessage);

extern void thread_canbus(void);
extern void	canbus_rx_handle(void);

#endif
 
//
