/*
 * File      : thread_compass.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-02    majianjia   the first version
 */
 
  
#ifndef __THREAD_COMPASS_H__
#define __THREAD_COMPASS_H__

//HMC5883L range register
#define HMC_RANGE_088_REG ((unsigned char)0x00 << 5)
#define HMC_RANGE_13_REG  ((unsigned char)0x01 << 5)
#define HMC_RANGE_19_REG  ((unsigned char)0x02 << 5)
#define HMC_RANGE_25_REG  ((unsigned char)0x03 << 5)
#define HMC_RANGE_40_REG  ((unsigned char)0x04 << 5)
#define HMC_RANGE_47_REG  ((unsigned char)0x05 << 5)
#define HMC_RANGE_56_REG  ((unsigned char)0x06 << 5)
#define HMC_RANGE_81_REG  ((unsigned char)0x07 << 5)

//LSB
#define HMC_RANGE_088 1370
#define HMC_RANGE_13  1090
#define HMC_RANGE_19  820
#define HMC_RANGE_25  660
#define HMC_RANGE_40  440
#define HMC_RANGE_47  390
#define HMC_RANGE_56  330
#define HMC_RANGE_81  230

#endif
