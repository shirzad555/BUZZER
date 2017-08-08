/*
 * led.c
 *
 *  Created on: Jun 24, 2017
 *      Author: Shirzad
 */

#include "PINCC26XX.h"
#include "Board.h"
#include "led.h"
#include "util.h"

// Clock instances for internal periodic events.
static Clock_Struct ledBlinkClock;



void Led_init(void)
{
	  PINCC26XX_setOutputEnable(BOARD_LED1, 1);
	  PINCC26XX_setOutputValue(BOARD_LED1, LED_OFF);
	  PINCC26XX_setOutputEnable(BOARD_LED2, 1);
	  PINCC26XX_setOutputValue(BOARD_LED2, LED_OFF);
}

void toggle_led(void)
{
	uint8_t bVal = PINCC26XX_getOutputValue(BOARD_LED2);
	PINCC26XX_setOutputValue(BOARD_LED2, !bVal);
}

/*
void Led_Flash(LED_State flash_type)
{

}
*/
