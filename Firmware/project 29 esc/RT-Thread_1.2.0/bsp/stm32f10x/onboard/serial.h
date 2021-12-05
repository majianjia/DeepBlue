/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _SERIAL_H
#define _SERIAL_H

#include "stm32f10x_usart.h"

#define SERIAL_UART		USART1
#define SERIAL_FLOW_CONTROL	USART_HardwareFlowControl_None
#define SERIAL_UART_PORT	GPIOA
#define SERIAL_UART_TX_PIN	GPIO_Pin_9
#define SERIAL_UART_RX_PIN	GPIO_Pin_10
//#define SERIAL_UART_CTS_PIN	GPIO_Pin_11
//#define SERIAL_UART_RTS_PIN	GPIO_Pin_12
#define SERIAL_TX_DMA		DMA1_Channel4
#define SERIAL_RX_DMA		DMA1_Channel5

#define SERIAL_MIN_BAUD		9600
#define SERIAL_MAX_BAUD		921600

#define SERIAL_TX_BUFSIZE	4096
#define SERIAL_RX_BUFSIZE	256

typedef struct {
    volatile unsigned char txBuf[SERIAL_TX_BUFSIZE];
    unsigned int txHead, txTail;
    volatile unsigned char rxBuf[SERIAL_RX_BUFSIZE];
    volatile unsigned int rxHead, rxTail;
    unsigned int rxPos;
} serialPort_t;

extern void serialInit(void);
extern void serialWrite(int ch);
extern void serialPrint(const char *str);
extern unsigned char serialAvailable();
extern int serialRead();
extern void serialSetConstants(void);


#endif
