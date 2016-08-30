/*
 * BatterySafety.h
 *
 *  Created on: 16.03.2016
 *      Author: Tschinder
 */

/*
 * All LEDs on -> Battery Voltage > 12V
 * Yellow and Red LED on -> Battery Voltage 12V - 11V
 * Red LED on -> Battery Voltage 11V - 10.5V
 * Red LED flashing -> Battery Voltage < 10.5V
 */


/*
 * The VBat Pin has to be connected to the right Pin of the Bat Safety Connector, also R13 must be min 30kOhm and C29(10µF) have to be mounted.
 */

#include <DAVE3.h>

#define VBat_LED_Green 	IO004_Handle0
#define VBat_LED_Yellow IO004_Handle1
#define VBat_LED_Red 	IO004_Handle2
