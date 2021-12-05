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

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "serial.h"
#include "config.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include <stdio.h>
#include <stdlib.h>

serialPort_t serialPort;

void serialStartTxDMA() {
    serialPort_t *s = &serialPort;

    SERIAL_TX_DMA->CMAR = (uint32_t)&s->txBuf[s->txTail];
    if (s->txHead > s->txTail) {
	SERIAL_TX_DMA->CNDTR = s->txHead - s->txTail;
	s->txTail = s->txHead;
    }
    else {
	SERIAL_TX_DMA->CNDTR = SERIAL_TX_BUFSIZE - s->txTail;
	s->txTail = 0;
    }

    DMA_Cmd(SERIAL_TX_DMA, ENABLE);
}

void serialWrite(int ch) {
    serialPort_t *s = &serialPort;

    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % SERIAL_TX_BUFSIZE;

    if (!(SERIAL_TX_DMA->CCR & 1))
	serialStartTxDMA();
}

unsigned char serialAvailable() {
    return (SERIAL_RX_DMA->CNDTR != serialPort.rxPos);
}

// only call after a affirmative return from serialAvailable()
int serialRead() {
    serialPort_t *s = &serialPort;
    int ch;

    ch = s->rxBuf[SERIAL_RX_BUFSIZE - s->rxPos];
    if (--s->rxPos == 0)
	s->rxPos = SERIAL_RX_BUFSIZE;

    return ch;
}

void serialPrint(const char *str) {
    while (*str)
	serialWrite(*(str++));
}

void serialOpenPort(int baud) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = SERIAL_FLOW_CONTROL;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SERIAL_UART, &USART_InitStructure);
}

void serialInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s = &serialPort;

    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SERIAL_UART_PORT, &GPIO_InitStructure);

    // input floating w/ pull ups
    GPIO_InitStructure.GPIO_Pin = SERIAL_UART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(SERIAL_UART_PORT, &GPIO_InitStructure);

    // Enable the DMA1_Channel4 global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;

    serialOpenPort(p[BAUD_RATE]);

    // Configure DMA for rx
    DMA_DeInit(SERIAL_RX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SERIAL_UART + 0x04;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s->rxBuf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = SERIAL_RX_BUFSIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_Init(SERIAL_RX_DMA, &DMA_InitStructure);

    DMA_Cmd(SERIAL_RX_DMA, ENABLE);

    USART_DMACmd(SERIAL_UART, USART_DMAReq_Rx, ENABLE);
    s->rxPos = DMA_GetCurrDataCounter(SERIAL_RX_DMA);

    // Configure DMA for tx
    DMA_DeInit(SERIAL_TX_DMA);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SERIAL_UART + 0x04;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(SERIAL_TX_DMA, &DMA_InitStructure);
    DMA_ITConfig(SERIAL_TX_DMA, DMA_IT_TC, ENABLE);
    SERIAL_TX_DMA->CNDTR = 0;

    USART_DMACmd(SERIAL_UART, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(SERIAL_UART, ENABLE);
}

// USART tx DMA IRQ
void DMA1_Channel4_IRQHandler(void) {
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(SERIAL_TX_DMA, DISABLE);

    if (serialPort.txHead != serialPort.txTail)
	serialStartTxDMA();
}

void serialSetConstants(void) {
    p[BAUD_RATE] = (int)p[BAUD_RATE];

    if (p[BAUD_RATE] < SERIAL_MIN_BAUD)
	p[BAUD_RATE] = SERIAL_MIN_BAUD;
    else if (p[BAUD_RATE] > SERIAL_MAX_BAUD)
	p[BAUD_RATE] = SERIAL_MAX_BAUD;

    serialOpenPort(p[BAUD_RATE]);
}