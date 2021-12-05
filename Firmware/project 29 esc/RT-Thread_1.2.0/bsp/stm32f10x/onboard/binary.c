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

#include "binary.h"
#include "serial.h"
#include "run.h"
#include "fet.h"
#include "adc.h"
#include "config.h"

binaryCommandStruct_t commandBuf;
uint32_t binaryLoop;
uint16_t sendAck, sendNack;
uint32_t binaryTelemRate;
uint8_t binaryTelemetryStop;
uint8_t binaryTelemRows;
uint8_t binaryTelemCols;
uint8_t outChkA, outChkB;
uint8_t inChkA, inChkB;
uint8_t binaryParseState;
uint8_t binaryTelemValues[BINARY_VALUE_NUM];

uint8_t binaryGetChar(uint8_t checkSum) {
    uint8_t c;

    c = serialRead();

    if (checkSum) {
	inChkA += c;
	inChkB += inChkA;
    }

    return c;
}

void binarySendChar(int ch) {
    serialWrite(ch);
    outChkA += ch;
    outChkB += outChkA;
}

void binarySendFloat(float f) {
    uint8_t j;
    uint8_t *c = (uint8_t *)&f;

    for (j = 0; j < sizeof(float); j++)
	binarySendChar(*c++);
}

void binarySendShort(uint16_t i) {
    uint8_t j;
    uint8_t *c = (uint8_t *)&i;

    for (j = 0; j < sizeof(uint16_t); j++)
	binarySendChar(*c++);
}

void binaryProcessAck(void) {
    while (sendAck || sendNack) {
	serialPrint("AqC");
	outChkA = outChkB = 0;

	// (N)ACK + checksum chars
	binarySendChar(1 + 2);

	if (sendAck) {
	    binarySendChar(BINARY_COMMAND_ACK);
	    binarySendShort(sendAck);
	    sendAck = 0;
	}
	else {
	    binarySendChar(BINARY_COMMAND_NACK);
	    binarySendShort(sendNack);
	    sendNack = 0;
	}

	serialWrite(outChkA);
	serialWrite(outChkB);
    }
}

void binaryTelemSend(void) {
    uint8_t *c;
    int i;

    serialWrite(outChkA);
    serialWrite(outChkB);

    // process any (N)Acks between telemetry packets
    binaryProcessAck();

    if (!binaryTelemetryStop) {
	serialPrint("AqT");

	outChkA = outChkB = 0;

	binarySendChar(binaryTelemRows);    // rows
	binarySendChar(binaryTelemCols);    // cols
    }
    else {
	binaryTelemRate = 0;
	binaryTelemetryStop = 0;
    }
}

//  TODO: yet to be implemented:
//    BINARY_COMMAND_PWM,
//    BINARY_COMMAND_STATUS,
//    BINARY_COMMAND_VERSION

void binaryAck(void) {
    sendAck = commandBuf.seqId;
}

void binaryNack(void) {
    sendNack = commandBuf.seqId;
}

void binaryCommandParse(void) {
    switch (commandBuf.command) {

    case BINARY_COMMAND_NOP:
	binaryAck();
	break;

    case BINARY_COMMAND_ARM:
	if (state == ESC_STATE_DISARMED) {
	    inputMode = ESC_INPUT_UART;
	    runArm();
	    binaryAck();
	}
	else {
	    binaryNack();
	}
	break;

    case BINARY_COMMAND_CLI:
	if (state > ESC_STATE_STOPPED) {
	    binaryNack();
	}
	else {
	    binaryTelemRate = 0;
	    commandMode = CLI_MODE;
	    binaryAck();
	}
	break;

    case BINARY_COMMAND_DISARM:
    	runDisarm(REASON_BINARY_USER);
	binaryAck();
	break;

    case BINARY_COMMAND_START:
	if (state == ESC_STATE_DISARMED || state > ESC_STATE_STOPPED) {
	    binaryNack();
	}
	else {
	    runStart();
	    binaryAck();
	}
	break;

    case BINARY_COMMAND_STOP:
	runStop();
	binaryAck();
	break;

    case BINARY_COMMAND_DUTY:
	if (runDuty(commandBuf.params[0]))
	    binaryAck();
	else
	    binaryNack();
	break;

    case BINARY_COMMAND_RPM:
	if (p[FF1TERM] != 0.0f) {
	    float rpm = commandBuf.params[0];

	    if (rpm < 100.0f)
		rpm = 100.0f;
	    else if (rpm > 10000.0f)
		rpm = 10000.0f;

	    if (runMode != CLOSED_LOOP_RPM) {
		runRpmPIDReset();
		runMode = CLOSED_LOOP_RPM;
	    }
	    targetRpm = rpm;

	    binaryAck();
	}
	else {
	    binaryNack();
	}
	break;

    case BINARY_COMMAND_TELEM_RATE:
	{
	    int32_t freq = (int32_t)commandBuf.params[0];

	    if (freq >= 1.0f) {
		if (freq > 1000)
		    freq = 1000;

		binaryTelemRate = RUN_FREQ / freq;
		binaryTelemRows = (freq <= 100) ? 1 : (freq / 50);
		binaryTelemetryStop = 0;

		// set initial output values
		if (binaryTelemCols == 0) {
		    binaryTelemValues[0] = BINARY_VALUE_RPM;
		    binaryTelemCols = 1;
		}
	    }
	    else {
		binaryTelemetryStop = 1;
	    }

	    binaryAck();
	}
	break;

    case BINARY_COMMAND_TELEM_VALUE:
	if (commandBuf.params[0] < BINARY_VALUE_NUM && commandBuf.params[1] < BINARY_VALUE_NUM) {
	    int i;

	    binaryTelemValues[(int)commandBuf.params[0]] = commandBuf.params[1];

	    binaryTelemCols = 0;
	    for (i = 0; i < BINARY_VALUE_NUM; i++) {
		if (binaryTelemValues[i] != BINARY_VALUE_NONE)
		    binaryTelemCols = i+1;
		else
		    break;
	    }

	    binaryAck();
	}
	else {
	    binaryNack();
	}
	break;

    case BINARY_COMMAND_SET:
	if (state <= ESC_STATE_STOPPED && configSetParamByID((int)commandBuf.params[0], commandBuf.params[1])) {
	    binaryAck();
	}
	else {
	    binaryNack();
	}
	break;

    case BINARY_COMMAND_CONFIG:
	switch ((int)commandBuf.params[0]) {
	    case 0:
		configReadFlash();
		binaryAck();
		break;
	    case 1:
		if (configWriteFlash())
		    binaryAck();
		else
		    binaryNack();
		break;
	    case 2:
		configLoadDefault();
		binaryAck();
		break;
	}
	break;

    default:
	break;
    }

    // only if telem is not running
    if (!binaryTelemRate)
	binaryProcessAck();

}

void binaryCommandRead(void) {
    static uint8_t n, i;
    uint8_t c;

    while (serialAvailable()) {
	c = binaryGetChar(binaryParseState < BINARY_STATE_CHKA);

	switch (binaryParseState) {
	case BINARY_STATE_SYNC1:
	    if (c == 'A')
		binaryParseState = BINARY_STATE_SYNC2;
	    break;

	case BINARY_STATE_SYNC2:
	    if (c == 'q') {
		inChkA = inChkB = 0;
		binaryParseState = BINARY_STATE_SIZE;
	    }
	    else {
		binaryParseState = BINARY_STATE_SYNC1;
	    }
	    break;

	case BINARY_STATE_SIZE:
	    n = c;
	    i = 0;
	    binaryParseState = BINARY_STATE_PAYLOAD;
	    break;

	case BINARY_STATE_PAYLOAD:
	    ((uint8_t *)&commandBuf)[i] = c;
	    i++;

	    if (i == n)
		binaryParseState = BINARY_STATE_CHKA;
	    break;

	case BINARY_STATE_CHKA:
	    if (inChkA != c)
		binaryParseState = BINARY_STATE_SYNC1;
	    else
		binaryParseState = BINARY_STATE_CHKB;
	    break;

	case BINARY_STATE_CHKB:
	    if (inChkB == c)
		binaryCommandParse();

	default:
	    binaryParseState = BINARY_STATE_SYNC1;
	    break;
	}
    }
}

void binaryCheck(void) {
    static int32_t telemRowsCount;

    binaryCommandRead();

    if (binaryTelemRate && !(binaryLoop % binaryTelemRate)) {
	int i;

	// send telemetry data
	for (i = 0; i < BINARY_VALUE_NUM; i++) {
	    if (binaryTelemValues[i] != BINARY_VALUE_NONE) {
		switch (binaryTelemValues[i]) {
		    case BINARY_VALUE_AMPS:
			binarySendFloat(avgAmps);
			break;

		    case BINARY_VALUE_VOLTS_BAT:
			binarySendFloat(avgVolts);
			break;

		    case BINARY_VALUE_VOLTS_MOTOR:
			binarySendFloat((float)fetActualDutyCycle/fetPeriod*avgVolts);
			break;

		    case BINARY_VALUE_RPM:
			binarySendFloat(rpm);
			break;

		    case BINARY_VALUE_DUTY:
			binarySendFloat((float)fetActualDutyCycle/fetPeriod);
			break;

		    case BINARY_VALUE_COMM_PERIOD:
			binarySendFloat((float)(crossingPeriod/TIMER_MULT));
			break;

		    case BINARY_VALUE_BAD_DETECTS:
			binarySendFloat((float)fetTotalBadDetects);
			break;

		    case BINARY_VALUE_ADC_WINDOW:
			binarySendFloat((float)histSize);
			break;

		    case BINARY_VALUE_IDLE_PERCENT:
			binarySendFloat(idlePercent);
			break;

		    case BINARY_VALUE_STATE:
			binarySendFloat((float)state);
			break;

		    case BINARY_VALUE_AVGA:
			binarySendFloat((float)avgA);
			break;

		    case BINARY_VALUE_AVGB:
			binarySendFloat((float)avgB);
			break;

		    case BINARY_VALUE_AVGC:
			binarySendFloat((float)avgC);
			break;

		    case BINARY_VALUE_AVGCOMP:
			binarySendFloat((float)(avgA+avgB+avgC)/3);
			break;

		    case BINARY_VALUE_FETSTEP:
			binarySendFloat((float)fetStep);
			break;
		}
	    }
	    else {
		break;
	    }
	}
	telemRowsCount++;

	if (telemRowsCount == binaryTelemRows) {
	    binaryTelemSend();
	    telemRowsCount = 0;
	}
    }

    binaryLoop++;
}