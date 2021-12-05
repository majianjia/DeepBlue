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

#ifndef _CLI_H
#define _CLI_H

#define CLI_INTR    3	    // interrupt
#define CLI_BELL    7	    // bell
#define CLI_TAB    9	    // tab
#define CLI_BS	    127	    // backspace

typedef struct {
    char *name;
    char *params;
    void (*cmdFunc)(void *cmd, char *cmdLine);
} cliCommand_t;

extern char version[16];

extern void cliInit(void);
extern void cliCheck(void);
extern void cliFuncArm(void *cmd, char *cmdLine);
extern void cliFuncBeep(void *cmd, char *cmdLine);
extern void cliFuncBinary(void *cmd, char *cmdLine);
extern void cliFuncBoot(void *cmd, char *cmdLine);
extern void cliFuncConfig(void *cmd, char *cmdLine);
extern void cliFuncDisarm(void *cmd, char *cmdLine);
extern void cliFuncDuty(void *cmd, char *cmdLine);
extern void cliFuncHelp(void *cmd, char *cmdLine);
extern void cliFuncInput(void *cmd, char *cmdLine);
extern void cliFuncMode(void *cmd, char *cmdLine);
extern void cliFuncPos(void *cmd, char *cmdLine);
extern void cliFuncPwm(void *cmd, char *cmdLine);
extern void cliFuncRpm(void *cmd, char *cmdLine);
extern void cliFuncSet(void *cmd, char *cmdLine);
extern void cliFuncStart(void *cmd, char *cmdLine);
extern void cliFuncStatus(void *cmd, char *cmdLine);
extern void cliFuncStop(void *cmd, char *cmdLine);
extern void cliFuncTelemetry(void *cmd, char *cmdLine);
extern void cliFuncVer(void *cmd, char *cmdLine);

#endif
