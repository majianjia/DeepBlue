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
/*
 * 此文件是1wire的通讯协议的实现,在pwm.c中,会产生pwm捕获中断,然后调用owEdgeDetect函数.来收发数据,处理
 * 复位，初始化：
 * 1、owReset 必须先调用此函数
 * 2、inputMode设置变量，输入模式为ow
 * 3、owResetIsr 进行进一步初始化
 * 
 * 数据的接收：
 * 1、pwm.c文件，PWM_IRQ_HANDLER中断函数中，会调用owEdgeDetect函数
 * 2、owEdgeDetect 判断是收数据还是发送数据，如果是接收数据
 * 3、	owReadBitIsr 函数中，读取GPIO的值，然后将数据放入owByteValue变量中，同时owBit加1，
 * 4、		owByteReceived 函数中，当owBit接收完成了一个字节后，才会调用，然后继续判断是否还有数据要接收，如果全部接收完则解析数据
 * 5、			命令解析
 * 数据的发送：
 * 1、pwm.c文件，PWM_IRQ_HANDLER中断函数中，会调用owEdgeDetect函数
 * 2、owEdgeDetect 判断是收数据还是发送数据
 */
#include "ow.h"
#include "main.h"
#include "timer.h"
#include "pwm.h"
#include "run.h"
#include "cli.h"
#include <string.h>

static uint8_t owROMCode[8];
static uint8_t owBuf[16];       //接收到的数据实际就是放入这里.
static uint8_t *owBufPointer;   //接收到的数据放入到这里，如果是要发送则从此变量中发送数据  指向owBuf数据的指针
static uint8_t owState;/*对应     OW_RESET_STATE_0, OW_RESET_STATE_1, OW_RESET_STATE_2, OW_READ, OW_WRITE 几种不同的状态 一般只会出现READ或WRITE的情况 */
static uint8_t owLastCommand;

static uint8_t owByteValue;//读取到的值.或要发送的数据都放在这里  (当前值)
static uint8_t owReadNum;  //当前还要读取的数据个数
static uint8_t owWriteNum; //当前要发送的数据个数
static uint8_t owBit;      //一个字节一共有8个位,代表当前操作的是第几个位 范围0~7

static uint8_t owCRC(uint8_t inData, uint8_t seed) {
	uint8_t bitsLeft;
	uint8_t temp;

	for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) 
	{
		temp = ((seed ^ inData) & 0x01);
		if (temp == 0) 
		{
			seed >>= 1;
		}
		else 
		{
			seed ^= 0x18;
			seed >>= 1;
			seed |= 0x80;
		}
		inData >>= 1;
	}

	return seed;
}

void owInit(void) {
	int i;

	owROMCode[0] = 0xAA;
	for (i = 0; i < 6; i++)
		owROMCode[i+1] = *(((uint8_t *)(OW_UID_ADDRESS))+i);

	owROMCode[7] = 0x00;
	for (i = 0; i < 7; i++)
		owROMCode[7] = owCRC(owROMCode[i], owROMCode[7]);
}

static void owPullLow(void) {
    pwmIsrAllOff();
    PWM_OUTPUT;
}

static void owRelease(void) {
    PWM_INPUT;
    pwmIsrRunOn();
}

static void owTimeout(int i) {
    timerSetAlarm3(i*TIMER_MULT, owTimeoutIsr, 0);
}
//复位初始化函数
void owReset(void) {
    inputMode = ESC_INPUT_OW;
    owResetIsr(OW_RESET_STATE_0);
}

static void owReadBytes(uint8_t num) {
    owTimeout(300);

    owReadNum = num;
    owByteValue = 0;
    owBit = 0;
}

static void owWriteBytes(uint8_t num) {
    owTimeout(300);

    owByteValue = *owBufPointer;
    owWriteNum = num;
    owBit = 0;
}

// command parser
static void owReadComplete(void) {
	uint8_t *pointer;

	switch (owBuf[0]) 
	{
	case OW_ROM_MATCH://比较ROM版本
		if (owLastCommand != owBuf[0]) {
			owLastCommand = owBuf[0];
			// read an additional 8 bytes
			owBufPointer = &owBuf[1];
			owReadBytes(8);
		}
		else {
			owLastCommand = 0x00;
			// does the ROM code match ours?
			if (!memcmp(&owBuf[1], owROMCode, 8)) {
				// prepare to read next command
				owBufPointer = owBuf;
				owReadBytes(1);
			}
			else {
				// terminate transaction
				owTimeoutIsr(0);
			}
		}
		break;

	case OW_ROM_SKIP://跳过本条命令
		// prepare to read next command
		owBufPointer = owBuf;
		owReadBytes(1);
		break;

	case OW_ROM_READ://ROMCODE
		// prepare to write out ROM CODE (8 bytes)
		owState = OW_WRITE;
		owBufPointer = owROMCode;
		owWriteBytes(8);
		break;

	case OW_VERSION://显示版本
		owState = OW_WRITE;
		owBufPointer = (uint8_t *)version;
		owWriteBytes(sizeof(version));
		break;


	case OW_PARAM_READ://读取参数
		if (owLastCommand != owBuf[0]) {
			owLastCommand = owBuf[0];
			// read an additional 1 bytes
			owBufPointer = &owBuf[1];
			owReadBytes(1);
		}
		else {
			owLastCommand = 0x00;
			if (owBuf[1] < CONFIG_NUM_PARAMS) {
				// copy config value into send buffer
				pointer = (uint8_t *)&(p[owBuf[1]]);
				owBuf[2] = pointer[0];
				owBuf[3] = pointer[1];
				owBuf[4] = pointer[2];
				owBuf[5] = pointer[3];

				owState = OW_WRITE;
				owBufPointer = owBuf;
				owWriteBytes(6);
			}
		}
		break;

	case OW_PARAM_WRITE://设置参数
		if (owLastCommand != owBuf[0]) {
			owLastCommand = owBuf[0];
			// read an additional 5 bytes
			owBufPointer = &owBuf[1];
			owReadBytes(5);
		}
		else {
			owLastCommand = 0x00;
			if (owBuf[1] < CONFIG_NUM_PARAMS) {
				// set parameter
				configSetParamByID(owBuf[1], *(float *)(&owBuf[2]));

				// copy config value into send buffer
				pointer = (uint8_t *)&p[owBuf[1]];
				owBuf[2] = pointer[0];
				owBuf[3] = pointer[1];
				owBuf[4] = pointer[2];
				owBuf[5] = pointer[3];

				owState = OW_WRITE;
				owBufPointer = owBuf;
				owWriteBytes(6);
			}
		}
		break;


	case OW_CONFIG_READ://从Flash上读取配置
		configReadFlash();
		// return command ack
		owState = OW_WRITE;
		owBufPointer = owBuf;
		owWriteBytes(1);
		break;

	// required 30ms to complete
	case OW_CONFIG_WRITE://将配置文件写入到Flash上
		configWriteFlash();
		break;

	case OW_CONFIG_DEFAULT://加载默认的配置文件
		configLoadDefault();
		// return command ack
		owState = OW_WRITE;
		owBufPointer = owBuf;
		owWriteBytes(1);
		break;


	case OW_GET_MODE://获取运行模式
		owBuf[0] = OW_GET_MODE;
		owBuf[1] = runMode;

		owState = OW_WRITE;
		owBufPointer = owBuf;
		owWriteBytes(2);
		break;

	case OW_SET_MODE://设置运行模式
		if (owLastCommand != owBuf[0]) {
			owLastCommand = owBuf[0];
			// read an additional 1 byte
			owBufPointer = &owBuf[1];
			owReadBytes(1);
		}
		else {
			owLastCommand = 0x00;
			if (owBuf[1] >= 0 && owBuf[1] < NUM_RUN_MODES) {
				runMode = owBuf[1];

				owState = OW_WRITE;
				owBufPointer = owBuf;
				owWriteBytes(2);
			}
		}
		;
		break;
	}
}

//数据全部发送完成
static void owWriteComplete(void) {
    // we're finished with the transaction
    owTimeoutIsr(0);
}

static void owByteReceived(void) {
	*(owBufPointer++) = owByteValue;

	owTimeout(300);

	if (--owReadNum > 0)
		owReadBytes(owReadNum);//还有数据要接收的
	else
		owReadComplete();      //数据都接收完成了.命令解析
}

static void owByteSent(void) {
	owBufPointer++;

	owTimeout(300);

	if (--owWriteNum > 0)
		owWriteBytes(owWriteNum);  //还有剩余的数据还没发送完成
	else
		owWriteComplete();         //数据全部发送完成
}

//此函数在pwm.c文件中,PWM中断函数里调用
void owEdgeDetect(uint8_t edge) {
	if (edge == 0) {
		switch (owState) {
		case OW_READ://读取数据
			// read bit in 15us
			pwmIsrAllOff();
			owRelease();
			timerSetAlarm3(15*TIMER_MULT, owReadBitIsr, 0);
			break;

		case OW_WRITE://写数据
			// write bit
			owRelease();
			owWriteBitIsr(0);
			break;
		}
	}
}

static void owResetIsr(int state) {
	switch (state) {
	case OW_RESET_STATE_0:
		// wait 15us
		timerSetAlarm3(15*TIMER_MULT, owResetIsr, OW_RESET_STATE_1);
		break;

	case OW_RESET_STATE_1:
		// signal presence for 70us
		owPullLow();
		timerSetAlarm3(70*TIMER_MULT, owResetIsr, OW_RESET_STATE_2);
		break;

	case OW_RESET_STATE_2:
		// release bus and wait for byte
		owRelease();

		// light status LED
		digitalLo(statusLed);

		// prepare to read a byte
		owState = OW_READ;     //设置为接收模式
		owBufPointer = owBuf;  //设置指针
		owLastCommand = 0x00;
		owReadBytes(1);        //先设置接收一个字节

		// 15ms timeout
		owTimeout(15000);
		break;
	}
}

static void owReadBitIsr(int unused) {
    // sample bus  读取GPIO，将数据放入owByteValue变量中
    owByteValue |= (PWM_SAMPLE_LEVEL<<owBit);

    // wait for next write slot
    pwmIsrRunOn();

    owTimeout(150);

    // is byte complete?
    if (++owBit > 7)
		owByteReceived(); //接收到8位数据了.组成一个字节
}

static void owWriteBitIsr(int state) 
{	
	if (state == 0) 
	{
		// write bit
		// only 0 bits need action, 1 bits let the bus rise on its own
		if (!(owByteValue & (0x01<<owBit++))) 
		{
			//发数据0
			owPullLow();
			// schedule release
			timerSetAlarm3(30*TIMER_MULT, owWriteBitIsr, 1);
		}
		else 
		{
			//发数据1
			owTimeout(150);

			// finished with this byte
			if (owBit > 7)
				owByteSent();
		}
	}
	// release bus
	else 
	{
		owRelease();

		owTimeout(150);

		// finished with this byte
		if (owBit > 7)
			owByteSent();
	}
}

static void owTimeoutIsr(int unused) {
    timerCancelAlarm3();
    // revert to listening for PWM & OW
    owRelease();
    pwmIsrAllOn();
    // turn off status LED
    digitalHi(statusLed);
    inputMode = ESC_INPUT_PWM;
}
