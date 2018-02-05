/* Blinky - No RTOS
 *
 */

#include <stdbool.h>
#include <bsp.h>
#include <delay.h>
#include <leds.h>
/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

void main() {
  bool connectLedCycle = true;

  bspInit();
  while (true) {
    ledToggle(USB_LINK_LED);
    if (connectLedCycle) {
      ledToggle(USB_CONNECT_LED);
    }
    connectLedCycle = !connectLedCycle;
    dly100us(2500);
  }
  /* Should never arrive here */     
}

