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

#include "main.h"
#include "can.h"
#include "config.h"
#include "run.h"
#include "pwm.h"
#include "fet.h"
#include "misc.h"
#include "timer.h"
#include "xxhash.h"

canDataStruct_t canData;

static inline uint32_t canGetSeqId(void) {
    uint32_t seqId;

    seqId = canData.seqId;
    canData.seqId = (canData.seqId + 1) & 0x3f;

    return seqId;
}

static inline int8_t canGetFreeMailbox(void) {
    int8_t mailbox;

    if ((CAN_CAN->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
	mailbox = 0;
    else if ((CAN_CAN->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
	mailbox = 1;
    else if ((CAN_CAN->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
	mailbox = 2;
    else
	mailbox = -1;

    return mailbox;
}

static int8_t canSend(uint32_t id, uint8_t tid, uint8_t seqId, uint8_t n, void *data) {
    int8_t mailbox;
    uint32_t *d = data;

    mailbox = canGetFreeMailbox();

    if (mailbox >= 0) {
	CAN_CAN->sTxMailBox[mailbox].TIR = id | (canData.networkId<<14) | ((tid & 0x1f)<<9) | (seqId<<3) | CAN_Id_Extended;

	n = n & 0xf;
	CAN_CAN->sTxMailBox[mailbox].TDTR = n;

	if (n) {
	    CAN_CAN->sTxMailBox[mailbox].TDLR = *d++;
	    CAN_CAN->sTxMailBox[mailbox].TDHR = *d;
	}

	CAN_CAN->sTxMailBox[mailbox].TIR |= 0x1;
    }
    else {
	canData.mailboxFull++;
    }

    return mailbox;
}

static inline void canNack(canPacket_t *pkt) {
    canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_NACK, pkt->sid, pkt->seq, 0, 0);
}

static inline void canAck(canPacket_t *pkt) {
    canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_ACK, pkt->sid, pkt->seq, 0, 0);
}

static inline void canReply(canPacket_t *pkt, uint8_t size) {
    canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_REPLY | (pkt->doc<<19), pkt->sid, pkt->seq, size, pkt->data);
}

static inline void canSetParam(canPacket_t *pkt) {
    if (configSetParamByID(*(pkt->data), *(float *)(pkt->data+1)))
	canAck(pkt);
    else
	canNack(pkt);
}

static inline void canSendGetAddr(void) {
    uint8_t d[8];

    *((uint32_t *)&d[0]) = canData.uuid;

    d[4] = CAN_TYPE_ESC;
    d[5] = escId;

    canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_REQ_ADDR, 0, canGetSeqId(), 6, d);
}

static inline void canFilterGroup(void) {
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    // additionally allow messages for our group
    if (canData.groupId > 0) {
	CAN_FilterInitStructure.CAN_FilterNumber = 2;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_TT_GROUP>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = canData.groupId<<9;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_TT_MASK>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_TID_MASK;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	    // TODO: perhaps use FIFO2 & interrupt
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
    }
}

static inline void canProcessAddr(canPacket_t *pkt) {
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    // our UUID?
    if (*((uint32_t *)&pkt->data[0]) == canData.uuid) {
	canData.networkId = pkt->tid;
	canData.groupId = ((uint8_t *)pkt->data)[4];
	canData.subGroupId = ((uint8_t *)pkt->data)[5];

	// set filter such that we only get messages destined for our TID
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_TT_NODE>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = canData.networkId<<9;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_TT_MASK>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_TID_MASK;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	// and bus reset
    	CAN_FilterInitStructure.CAN_FilterNumber = 1;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_FID_RESET_BUS>>16;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_FID_MASK>>16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	// if we got a ground assignment
	if (canData.groupId)
	    canFilterGroup();
    }
}

static void inline canBusReset(void) {
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    int i;

    runDisarm(REASON_CAN_USER);

    canData.networkId = 0;
    canData.groupId = 0;
    canData.subGroupId = 0;
    canData.telemRate = 0;

    for (i = 0; i < CAN_TELEM_NUM; i++)
	canData.telemValues[i] = 0;


    // Initially only listen for CAN_FID_GRANT_ADDR
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_FID_GRANT_ADDR>>16;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_FID_MASK>>16;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 2;
    CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // ask for new address
    canSendGetAddr();
}

static inline void canProcessSet(canPacket_t *pkt) {
    switch (pkt->doc) {
    case CAN_DATA_GROUP:
	canData.groupId = ((uint8_t *)pkt->data)[0];
	canData.subGroupId = ((uint8_t *)pkt->data)[1];
	canFilterGroup();
	canAck(pkt);
	break;

    case CAN_DATA_TYPE:
	canNack(pkt);
	break;

    case CAN_DATA_INPUT_MODE:
	if (*(uint8_t *)pkt->data < ESC_INPUT_MAX) {
	    inputMode = *(uint8_t *)pkt->data;
	    canAck(pkt);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_DATA_RUN_MODE:
	if (*(uint8_t *)pkt->data < NUM_RUN_MODES) {
	    runDisarm(REASON_CAN_USER);
	    runMode = *(uint8_t *)pkt->data;
	    canAck(pkt);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_DATA_STATE:
	canNack(pkt);
	break;

    case CAN_DATA_PARAM:
	canSetParam(pkt);
	break;

    case CAN_DATA_TELEM:
	canNack(pkt);
	break;
    }
}

static inline void canProcessGet(canPacket_t *pkt) {
    uint8_t *p1, *p2;

    switch (pkt->doc) {
    case CAN_DATA_GROUP:
	((uint8_t *)pkt->data)[0] = canData.groupId;
	((uint8_t *)pkt->data)[1] = canData.subGroupId;
	canReply(pkt, 2);
	break;

    case CAN_DATA_TYPE:
	((uint8_t *)pkt->data)[0] = CAN_TYPE_ESC;
	canReply(pkt, 1);
	break;

    case CAN_DATA_ID:
	((uint8_t *)pkt->data)[0] = escId;
	canReply(pkt, 1);
	break;

    case CAN_DATA_INPUT_MODE:
	((uint8_t *)pkt->data)[0] = inputMode;
	canReply(pkt, 1);
	break;

    case CAN_DATA_RUN_MODE:
	((uint8_t *)pkt->data)[0] = runMode;
	canReply(pkt, 1);
	break;

    case CAN_DATA_STATE:
	((uint8_t *)pkt->data)[0] = state;
	canReply(pkt, 1);
	break;

    case CAN_DATA_PARAM:
	if (*((uint16_t *)pkt->data) < CONFIG_NUM_PARAMS) {
	    float val = p[*((uint16_t *)pkt->data)];

	    *(float *)(pkt->data) = val;
	    canReply(pkt, 4);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_DATA_VERSION:
	p1 = (uint8_t *)pkt->data;
	p2 = (uint8_t *)VERSION;

	while (*p2)
	    *p1++ = *p2++;

	*p1 = 0;

	canReply(pkt, 8);
	break;

    case CAN_DATA_TELEM:
	// TODO
	break;
    }
}

static inline void canProcessSetpoint10(canPacket_t *pkt) {
    uint16_t val;

    if ((pkt->id & CAN_TT_MASK) != CAN_TT_NODE) {
	canGroup10_t *gPkt10 = (canGroup10_t *)pkt->data;

	switch (canData.subGroupId) {
	    case 1:
		val = gPkt10->value1;
		break;
	    case 2:
		val = gPkt10->value2;
		break;
	    case 3:
		val = gPkt10->value3;
		break;
	    case 4:
		val = gPkt10->value4;
		break;
	    case 5:
		val = gPkt10->value5;
		break;
	    case 6:
		val = gPkt10->value6;
		break;
	}
    }
    else {
	val = *pkt->data;
    }

    runSetpoint(val<<6);
}

static inline void canProcessSetpoint12(canPacket_t *pkt) {
    uint16_t val;

    if ((pkt->id & CAN_TT_MASK) == CAN_TT_GROUP) {
	canGroup12_t *gPkt12 = (canGroup12_t *)pkt->data;

	switch (canData.subGroupId) {
	    case 1:
		val = gPkt12->value1;
		break;
	    case 2:
		val = gPkt12->value2;
		break;
	    case 3:
		val = gPkt12->value3;
		break;
	    case 4:
		val = gPkt12->value4;
		break;
	    case 5:
		val = gPkt12->value5;
		break;
	}
    }
    else {
	val = *pkt->data;
    }

    runSetpoint(val<<4);
}

static inline void canProcessSetpoint16(canPacket_t *pkt) {
    uint16_t val;

    if ((pkt->id & CAN_TT_MASK) == CAN_TT_GROUP) {
	canGroup16_t *gPkt16 = (canGroup16_t *)pkt->data;

	switch (canData.subGroupId) {
	    case 1:
		val = gPkt16->value1;
		break;
	    case 2:
		val = gPkt16->value2;
		break;
	    case 3:
		val = gPkt16->value3;
		break;
	    case 4:
		val = gPkt16->value4;
		break;
	}
    }
    else {
	val = *pkt->data;
    }

    runSetpoint(val);
}

static inline void canProcessRpm(canPacket_t *pkt) {
    uint16_t val;

    if ((pkt->id & CAN_TT_MASK) == CAN_TT_GROUP) {
	canGroup16_t *gPkt16 = (canGroup16_t *)pkt->data;

	switch (canData.subGroupId) {
	    case 1:
		val = gPkt16->value1;
		break;
	    case 2:
		val = gPkt16->value2;
		break;
	    case 3:
		val = gPkt16->value3;
		break;
	    case 4:
		val = gPkt16->value4;
		break;
	}
    }
    else {
	val = *pkt->data;
    }

    runMode = CLOSED_LOOP_RPM;
    targetRpm = (float)val;
}

static inline void canProcessBeep(canPacket_t *pkt) {
    uint16_t *data = (uint16_t *)pkt->data;

    fetBeep(data[0], data[1]);
}

static inline void canProcessPos(canPacket_t *pkt) {
    float *data = (float *)pkt->data;

    fetSetAngle(data[0]);
}

void canSendStatus(void) {
    esc32CanStatus_t stat;

    stat.state = state;
    stat.vin = avgVolts * 100;
    stat.amps = avgAmps * 100;
    stat.rpm = rpm;
    stat.duty = fetActualDutyCycle * 255 /fetPeriod;
    stat.errors = fetTotalBadDetects;
    stat.errCode = disarmReason;

    canSend(CAN_LCC_INFO | CAN_TT_NODE | CAN_FID_TELEM | (CAN_TELEM_STATUS<<19), 0, canGetSeqId(), 8, &stat);
}

void canTelemDo(void) {
    int i;

    for (i = 0; i < CAN_TELEM_NUM; i++) {
	switch (canData.telemValues[i]) {
	    case 0:
		break;

	    case CAN_TELEM_STATUS:
		canSendStatus();
		break;

	    default:
		break;
	}
    }
}

static inline void canProcessCmd(canPacket_t *pkt) {
    uint8_t *data = (uint8_t *)pkt->data;
    inputMode = ESC_INPUT_CAN;

    switch (pkt->doc) {
    case CAN_CMD_ARM:
	if (state == ESC_STATE_DISARMED)
	    runArm();
	canAck(pkt);
	break;

    case CAN_CMD_DISARM:
	runDisarm(REASON_CAN_USER);
	canAck(pkt);
	break;

    case CAN_CMD_START:
	runStart();
	canAck(pkt);
	break;

    case CAN_CMD_STOP:
	runStop();
	canAck(pkt);
	break;

    case CAN_CMD_SETPOINT10:
	canData.validMicros = timerMicros;
	canProcessSetpoint10(pkt);
	break;

    case CAN_CMD_SETPOINT12:
	canData.validMicros = timerMicros;
	canProcessSetpoint12(pkt);
	break;

    case CAN_CMD_SETPOINT16:
	canData.validMicros = timerMicros;
	canProcessSetpoint16(pkt);
	break;

    case CAN_CMD_RPM:
	canData.validMicros = timerMicros;
	canProcessRpm(pkt);
	break;

    case CAN_CMD_CFG_READ:
	if (state <= ESC_STATE_STOPPED) {
	    configReadFlash();
	    canAck(pkt);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_CMD_CFG_WRITE:
	if (state <= ESC_STATE_STOPPED && configWriteFlash())
	    canAck(pkt);
	else
	    canNack(pkt);
	break;

    case CAN_CMD_CFG_DEFAULT:
	if (state <= ESC_STATE_STOPPED) {
	    configLoadDefault();
	    canAck(pkt);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_CMD_TELEM_RATE:
	canData.telemRate = *(uint16_t *)pkt->data;
	if (canData.telemRate > RUN_FREQ)
	    canData.telemRate = RUN_FREQ;
	canAck(pkt);
	break;

    case CAN_CMD_TELEM_VALUE:
	canData.telemValues[data[0]] = data[1];
	canAck(pkt);
	break;

    case CAN_CMD_RESET:
	// TODO
	break;

    case CAN_CMD_BEEP:
	if (state <= ESC_STATE_STOPPED) {
	    canProcessBeep(pkt);
	    canAck(pkt);
	}
	else {
	    canNack(pkt);
	}
	break;

    case CAN_CMD_POS:
	if (runMode == SERVO_MODE || state == ESC_STATE_RUNNING) {
	    canProcessPos(pkt);
	}
	else {
	    canNack(pkt);
	}

    case CAN_CMD_USER_DEFINED:
	// TODO
	break;
    }
}

void canProcess(void) {
    static uint32_t loops = 0;
    CanRxMsg rx;
    canPacket_t pkt;

    loops++;

    // telemetry
    if (canData.telemRate && !(loops % (RUN_FREQ / canData.telemRate)))
	canTelemDo();

    if (CAN_MessagePending(CAN_CAN, CAN_FIFO0) == 0) {
	// keep trying to get an address
	if (canData.networkId == 0 && !(loops % (100 * 1000 / RUN_FREQ)))
	    canSendGetAddr();

	return;
    }

    CAN_Receive(CAN_CAN, CAN_FIFO0, &rx);

    // ignore standard id's
    if (rx.IDE != CAN_Id_Standard) {
	pkt.id = rx.ExtId << 3;
	pkt.doc = (pkt.id & CAN_DOC_MASK)>>19;
	pkt.sid = (pkt.id & CAN_SID_MASK)>>14;
	pkt.tid = (pkt.id & CAN_TID_MASK)>>9;
	pkt.seq = (pkt.id & CAN_SEQ_MASK)>>3;
	pkt.data = (uint32_t *)rx.Data;
	canData.packetsReceived++;

	// do we need a network address?
	if (canData.networkId == 0 && (pkt.id & CAN_FID_GRANT_ADDR)) {
	    canProcessAddr(&pkt);
	    return;
	}

	switch (pkt.id & CAN_FID_MASK) {
	case CAN_FID_RESET_BUS:
	    canBusReset();
	    break;

	case CAN_FID_CMD:
	    canProcessCmd(&pkt);
	    break;

	case CAN_FID_SET:
	    canProcessSet(&pkt);
	    break;

	case CAN_FID_GET:
	    canProcessGet(&pkt);
	    break;

	case CAN_FID_PING:
	    canAck(&pkt);
	    break;

	case CAN_FID_ACK:
	    // NOP
	    break;

	case CAN_FID_NACK:
	    // NOP
	    break;

	default:
	    NOP;
	    break;
	}
    }
}

void canInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Configure CAN pin: RX
    GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(CAN_PORT, &GPIO_InitStructure);

    // Configure CAN pin: TX
    GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN_PORT, &GPIO_InitStructure);

    // CAN register init
    CAN_DeInit(CAN_CAN);
    CAN_StructInit(&CAN_InitStructure);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    // CAN Baudrate = 1MBps
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Prescaler = 4;
    CAN_Init(CAN_CAN, &CAN_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//    CAN_ITConfig(CAN_CAN, CAN_IT_TME, ENABLE);
//    CAN_ITConfig(CAN_CAN, CAN_IT_FMP0, ENABLE);
//    CAN_ITConfig(CAN_CAN, CAN_IT_FMP1, ENABLE);

    canData.uuid = XXH32((void *)CAN_UUID, 3*4, 0);

    canBusReset();
    canSendGetAddr();
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
}

void canSetConstants(void) {
}

void CAN1_RX1_IRQHandler(void) {
}

void USB_HP_CAN1_TX_IRQHandler(void)  {
}
