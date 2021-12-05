/*
 * File      : thread_can.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.15    majianjia   the first version
 */
 
#ifndef __THREAD_CAN_H__
#define __THREAD_CAN_H__

extern void thread_can(void);

extern unsigned int volatile systick;
extern unsigned int volatile seconds;

#include "stm32f10x.h"
//send 
extern int can_message_send(CanTxMsg *TxMessage);
//urgent send
extern int can_message_urgent(CanTxMsg *TxMessage);
#endif
