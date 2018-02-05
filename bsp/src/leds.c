#include <stdint.h>
#include <iolpc2378.h>
#include <board.h>
#include <leds.h>

/* Initialise the interface to the USB_LINK_LED
 */
void ledsInit( void ) {
/* Assign output to P0.13 and P0.14
 * Notice USB_LINK_LED_MASK is defined in board.h as (1UL << 13)
 *        USB_CONNECT_LED_MASK is defined as (1UL << 14)
 */
  FIO0DIR |= USB_LINK_LED_MASK;
  FIO0DIR |= USB_CONNECT_LED_MASK;
/* LED off initially */
  ledSetState(USB_LINK_LED, LED_OFF);
  ledSetState(USB_CONNECT_LED, LED_OFF);
}

/* Return the state of the USB_LINK_LED
*/
ledState_t ledGetState(ledIdentifier_t led) {
  ledState_t state = LED_OFF;
  uint32_t mask;
  
  if (led == USB_LINK_LED) {
    mask = USB_LINK_LED_MASK; 
  } else {
    mask = USB_CONNECT_LED_MASK;
  }
  
  if (FIO0PIN & mask) {
    state = LED_OFF;
  } else {
    state = LED_ON;
  }
  return state;
}
 
/* Set the state of the USB_LINK_LED
 * 
 *
 * Notice the pin is CLEARED to turn the led ON
 * and    the pin is SET     to turn the led OFF
 */
void ledSetState(ledIdentifier_t led, ledState_t state) {
  uint32_t mask;
  
  if (led == USB_LINK_LED) {
    mask = USB_LINK_LED_MASK; 
  } else {
    mask = USB_CONNECT_LED_MASK;
  }

/* Set LED state */
  if ( state == LED_ON ) {
    FIO0CLR = mask;  // writing zeroes to this register has no effect so simple assignment is ok
  } else {
    FIO0SET = mask;  // writing zeroes to this register has no effect so simple assignment is ok
  }
}

/* Turn on the USB_LINK_LED if it's off 
 * Turn off the USB_LINK_LED if it's on
 */
void ledToggle(ledIdentifier_t led) {
  if (ledGetState(led) == LED_ON) {
    ledSetState(led, LED_OFF);
  } else {
    ledSetState(led, LED_ON);
  }
}




