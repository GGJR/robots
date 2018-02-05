/*************************************************************************
 *
 *    File name   : buttons.c
 *    Description : Configures and Reads the buttons on the LPC2378 board
 *
 *    History :
 *    1. Date        : November 21, 2009
 *       Author      : WDH
 *       Description : Create
 *
 *       Modified    : DK - 03-10-10
 **************************************************************************/

#include <assert.h>
#include <iolpc2378.h>
#include <board.h>
#include <buttons.h>

#define ALL_BUT 7
#define ALL_JS  8

static const uint32_t buttonMasks[9] = {
  B1_MASK >> 16,
  B2_MASK >> 16,
  JS_LEFT_MASK,
  JS_RIGHT_MASK,
  JS_UP_MASK,
  JS_DOWN_MASK,
  JS_CENTER_MASK,
  B1_MASK | B2_MASK,
  JS_LEFT_MASK | JS_RIGHT_MASK | JS_UP_MASK | JS_DOWN_MASK | JS_CENTER_MASK
};

/* Initialise the interface to the on-board LPC2378 board buttons
 *
 *                         JS_UP
 *                           |
 *   BUT_1    JS_LEFT -- JS_CENTRE -- JS_RIGHT    BUT_2
 *                           |
 *                        JS_DOWN
 */
void buttonsInit(void) {

/* Define inputs for buttons using Fast GPIO interface
 */
   B1_FDIR &= ~B1_MASK;
   B2_FDIR &= ~B2_MASK;
   JS_FDIR &= ~JS_LEFT_MASK;
   JS_FDIR &= ~JS_RIGHT_MASK;
   JS_FDIR &= ~JS_UP_MASK;
   JS_FDIR &= ~JS_DOWN_MASK;
   JS_FDIR &= ~JS_CENTER_MASK;
}


/* Read the state of the buttons and joystick
 * return result with buttons in bottom half of
 * 32 bit int and joystick in top half
 */
uint32_t buttonsRead(void) {
  uint32_t state;

  state = (JS_FIO & buttonMasks[ALL_JS]);
  state |= ((B1_FIO & buttonMasks[ALL_BUT]) >> 16);
  return state;
}


/* Reads the state of an on-board button
 * button in { BUT_1, BUT_2, JS_LEFT, JS_RIGHT, JS_UP, JS_DOWN, JS_CENTRE }
 * Returns true if button is currently pressed and false otherwise
 */
bool isButtonPressed(uint8_t button ) {
  return isButtonPressedInState(buttonsRead(), button);
}

bool isButtonPressedInState(uint32_t state, uint8_t button) {
  return (state & buttonMasks[button] ? false : true);
}


buttonState_t updateButtonState(uint32_t state, uint8_t button) {
  static buttonState_t buttonStates[7] = {B_IDLE, B_IDLE, B_IDLE, B_IDLE,
                                          B_IDLE, B_IDLE, B_IDLE};
  switch (buttonStates[button]) {
  case B_IDLE:
    buttonStates[button] = (isButtonPressedInState(state, button) ? B_PRESSED : B_IDLE);
    break;
  case B_PRESSED:
    buttonStates[button] = (isButtonPressedInState(state, button) ? B_PRESSED : B_PRESSED_RELEASED);
    break;
  case B_PRESSED_RELEASED:
    buttonStates[button] = (isButtonPressedInState(state, button) ? B_PRESSED : B_IDLE);
    break;
  default:
    break;
  }
  return buttonStates[button];                     
}


