#ifndef __LEDS_H
#define __LEDS_H

#include <stdint.h>

typedef enum ledIdentifier {USB_LINK_LED, USB_CONNECT_LED} ledIdentifier_t;
typedef enum ledState {LED_OFF, LED_ON} ledState_t;

void ledsInit(void);
ledState_t ledGetState(ledIdentifier_t led);
void ledSetState(ledIdentifier_t led, ledState_t state);
void ledToggle(ledIdentifier_t led);

#endif