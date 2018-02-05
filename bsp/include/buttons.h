/*************************************************************************
 *
 *    File name   : buttons.h
 *    Description : Configures and Reads the buttons on the LPC2378 board
 *
 *    History :
 *    1. Date        : November 21, 2009
 *       Author      : WDH
 *       Description : Create
 *  
 *       Modified    : DK - 03-10-10
 *
 **************************************************************************/
#ifndef __BUTTONS_H
#define __BUTTONS_H

#include <stdint.h>
#include <stdbool.h>

#define BUT_1     0x00
#define BUT_2     0x01
#define JS_LEFT   0x02
#define JS_RIGHT  0x03
#define JS_UP     0x04
#define JS_DOWN   0x05
#define JS_CENTRE 0x06

typedef enum buttonState{
  B_IDLE,
  B_PRESSED,
  B_PRESSED_RELEASED
} buttonState_t;


/* Initialise the interface to the buttons and joystick
 * by ensuring that all the button and joystick GPIO pins
 * are configured as inputs
 */
void buttonsInit(void);

/* Read the state of the buttons and joystick
 * return result with buttons in bottom half of
 * 32 bit int and joystick in top half
 */
uint32_t buttonsRead(void);


/* Test the state of a button or the joystick
 * Return true if the button is pressed
 * otherwise return false
 */
bool isButtonPressed(uint8_t button);

/* Test the state of a button or the joystick
 * in a given state - avoids a call to buttonsRead()
 * Return true if the button is pressed
 * otherwise return false
 */
bool isButtonPressedInState(uint32_t state, uint8_t button);

/* Update the state of a selected button and return its
 * new state
 */
buttonState_t updateButtonState(uint32_t state, uint8_t button);


#endif