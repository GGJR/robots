#include <stdint.h>
#include <iolpc2378.h>
#include <interface.h>

void interfaceInit(interface_t ifType) {
  PINSEL4 = 0x00000000;   // GPIO selected on all pins
  FIO2DIR |= 0x000000F0;  // outputs on P2.7-P2.4 for the data leds.
  switch (ifType) {
  case CONTROL:
    PINSEL4 |= 0x00000010;  // PWM selected on P2.2 (PWM1.3)
    FIO2DIR &= ~0x0000000B; // inputs on P2.3,P2.1,P2.0; button, sensor in and out 
    break;
  case CONVEYOR:
    FIO2DIR |= 0x00000003;  // outputs on P2.1-P2.0; motor enable and direction
    FIO2DIR &= ~0x0000000C; // inputs on P2.3-P2.2; sensor out and in 
    break;
  case ROBOT:
    PINSEL4 |= 0x00000055; // PWM selected on P2.3-P2.0
    break;
  default:
    break;
  }
  interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
}

/* Set the state of the INTERFACE LEDS
 * 
 *
 * Notice the pin is SET     to turn the led ON
 * and    the pin is CLEARED to turn the led OFF
 */
void interfaceLedSetState(uint32_t leds, ledState_t state) {

/* Set LED state */
  if ( state == LED_ON ) {
    FIO2SET = leds;  // writing zeroes to this register has no effect so simple assignment is ok
  } else {
    FIO2CLR = leds;  // writing zeroes to this register has no effect so simple assignment is ok
  }
}

void interfaceLedToggle(uint32_t leds) {
  FIO2PIN ^= leds;
}
  