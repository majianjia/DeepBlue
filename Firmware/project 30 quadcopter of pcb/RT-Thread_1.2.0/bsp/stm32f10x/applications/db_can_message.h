/*
 * File      : DB_can_message.h
 * This file defines the deep blue plane's CAN bus message id and data type.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.23    majianjia   the first version
 */
 
#ifndef __DB_CAN_MESSAGE_H__
#define __DB_CAN_MESSAGE_H__


/*
	all definition base on CAN bus standar id
*/

//first two bits define the master or slaver, 
//the bit 8 to bit 3 define the message type, 
//and the last three bits define the indexes for messages;
#define CAN_FILTER   0x01F8   //bin 0011 1111 000	


//bit 10 to 9 defines the transer type 
#define CAN_TRANSER_TYPE_OFFSET (9)
// 0x00  master
// 0x01  slaver
#define CAN_TYPE_MASTER ((0x00) << CAN_TRANSER_TYPE_OFFSET)
#define CAN_TYPE_SLAVER ((0x01) << CAN_TRANSER_TYPE_OFFSET)

/*
	Bit 8 to 3 message type. Range from 0x00 to 0x3f
	there are 64 differnt messages have been define in this file. Every messages also have 8 indexes. 
	It makes that every message could include 8*8 = 64byte data, which always enough to carry all of the messages you want to transfer.

*/
#define CAN_MESSAGE_TYPE_OFFSET (3)

/* 
	most important message 
	range form 0x00 to 0x0F
*/
#define CAN_MESSAGE_IMPORTANCE_BASE (0x00)

//system time broadcast
#define CAN_TIME_BROADCAST 	((CAN_MESSAGE_IMPORTANCE_BASE + 1) << CAN_MESSAGE_TYPE_OFFSET) 

//controllor groups
#define CAN_CTRL_GROUP0 	((CAN_MESSAGE_IMPORTANCE_BASE + 2) << CAN_MESSAGE_TYPE_OFFSET) 
#define CAN_CTRL_GROUP1 	((CAN_MESSAGE_IMPORTANCE_BASE + 3) << CAN_MESSAGE_TYPE_OFFSET)

//Alttitude angle
#define CAN_ALT_ANGLE 		((CAN_MESSAGE_IMPORTANCE_BASE + 4) << CAN_MESSAGE_TYPE_OFFSET)

//position
#define CAN_POSITION		((CAN_MESSAGE_IMPORTANCE_BASE + 5) << CAN_MESSAGE_TYPE_OFFSET)


/* 
	normal message 
	rang form 0x10 to 0x2F
*/
#define CAN_MESSAGE_NORMAL_BASE (0x10)

//battery
#define CAN_BATTERY    		((CAN_MESSAGE_NORMAL_BASE + 0) << CAN_MESSAGE_TYPE_OFFSET)

//air infomation
#define CAN_AIR_INFO  		((CAN_MESSAGE_NORMAL_BASE + 1) << CAN_MESSAGE_TYPE_OFFSET)

//air speed
#define CAN_AIR_SPEED 		((CAN_MESSAGE_NORMAL_BASE + 2) << CAN_MESSAGE_TYPE_OFFSET)

//ultrasonic ranging
#define CAN_ULT_RANG		((CAN_MESSAGE_NORMAL_BASE + 3) << CAN_MESSAGE_TYPE_OFFSET)

//GPS
#define CAN_GPS				((CAN_MESSAGE_NORMAL_BASE + 4) << CAN_MESSAGE_TYPE_OFFSET)

//PPM
#define CAN_PPM				((CAN_MESSAGE_NORMAL_BASE + 5) << CAN_MESSAGE_TYPE_OFFSET)

//NAVIGATOR LIGHT
#define CAN_NAVIGATOR_LIGHT	((CAN_MESSAGE_NORMAL_BASE + 6) << CAN_MESSAGE_TYPE_OFFSET)

//navigator information
#define CAN_NAVIGATOR_INFO	((CAN_MESSAGE_NORMAL_BASE + 7) << CAN_MESSAGE_TYPE_OFFSET)

//Date
#define CAN_DATE			((CAN_MESSAGE_NORMAL_BASE + 8) << CAN_MESSAGE_TYPE_OFFSET)




/* 
	debug message 
	(not always support all of the messages at the same time.)
	range from 0x30 to 0x3f
	
*/
#define CAN_MESSAGE_DEBUG_BASE (0x30)

//IMU data
#define CAN_DEBUG_IMU 			((CAN_MESSAGE_DEBUG_BASE + 0) << CAN_MESSAGE_TYPE_OFFSET)

//navigator PID
#define CAN_DEBUG_NAV_PID   	((CAN_MESSAGE_DEBUG_BASE + 1) << CAN_MESSAGE_TYPE_OFFSET)

//controller PID
#define CAN_DEBUG_CTRL_PID   	((CAN_MESSAGE_DEBUG_BASE + 2) << CAN_MESSAGE_TYPE_OFFSET)

//key board
#define CAN_DEBUG_KEY_BOARD 	((CAN_MESSAGE_DEBUG_BASE + 3) << CAN_MESSAGE_TYPE_OFFSET)


//index filter
#define CAN_INDEX_FILTER 	0x07

//Bit 2 to 0 message index
#define CAN_MESSAGE_INDEX0 0x00
#define CAN_MESSAGE_INDEX1 0x01
#define CAN_MESSAGE_INDEX2 0x02
#define CAN_MESSAGE_INDEX3 0x03
#define CAN_MESSAGE_INDEX4 0x04
#define CAN_MESSAGE_INDEX5 0x05
#define CAN_MESSAGE_INDEX6 0x06
#define CAN_MESSAGE_INDEX7 0x07


#endif
