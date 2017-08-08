/*
 * alarm.h
 *
 *  Created on: Jun 25, 2017
 *      Author: Shirzad
 */

#ifndef APPLICATION_ALARM_H_
#define APPLICATION_ALARM_H_

enum Alarm_Cmd {
    ALARM_OFF 	    	= 0x00,
	ALARM_ON_SEN_1 		= 0x01,
	ALARM_ON_SEN_2   	= 0x02,
	ALARM_ON_SEN_3   	= 0x03,
	ALARM_ON_SEN_4   	= 0x04,
	ALARM_ON_SEN_5     	= 0x05,
	ALARM_ERROR			= 0xFF
};


bStatus_t Alarm_SetSetting( uint8_t alarm_setting );
uint8_t  Alarm_GetSetting (void);
void Start_Alarm (void);

#endif /* APPLICATION_ALARM_H_ */
