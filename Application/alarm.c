/*
 * alarm.c
 *
 *  Created on: Jun 25, 2017
 *      Author: Shirzad
 */

#include <stdint.h>
#include "bcomdef.h"
#include "comdef.h"
#include "alarm.h"



// Alarm Setting var
static uint8_t alarm_current_setting = ALARM_OFF;

/*********************************************************************
 */
bStatus_t Alarm_SetSetting( uint8_t alarm_setting )
{
  if (alarm_setting <= ALARM_ON_SEN_5)
  {
	  alarm_current_setting = alarm_setting;
	  return SUCCESS;
  }
  else
  {
	  alarm_current_setting = ALARM_ERROR;
	  return INVALIDPARAMETER;
  }
}

/*********************************************************************
 */
uint8_t  Alarm_GetSetting (void)
{
  return alarm_current_setting;
}

void Start_Alarm (void)
{

}
