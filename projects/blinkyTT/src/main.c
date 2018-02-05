/*
 * Simple program to flash the LEDs 
 *
 * Implemented using a time-triggered scheduler.
 *
 * DK - 08-March-2011
 */


#include <stdbool.h>
#include <stdint.h>
#include <bsp.h>
#include <leds.h>
#include <scheduler.h>

void linkLedToggleTask(void);
void connectLedToggleTask(void);

int main () {
  
  bspInit();
  schInit();
  schAddTask(linkLedToggleTask, 0, 250);
  schAddTask(connectLedToggleTask, 0, 500);


  schStart();
  
  while (true) {
    schDispatch();
  }
}


void linkLedToggleTask(void) {
    ledToggle(USB_LINK_LED);
}

void connectLedToggleTask(void) {
    ledToggle(USB_CONNECT_LED);
}

