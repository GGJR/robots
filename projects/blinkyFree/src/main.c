/* blinky - FreeRTOS
 *
 */

#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>

#include <bsp.h>
#include <leds.h>

/* Priorities for the demo application tasks. */
#define USB_LINK_LED_TASK_PRIORITY		(tskIDLE_PRIORITY + 5)
#define USB_CONNECT_LED_TASK_PRIORITY		(tskIDLE_PRIORITY + 3)

/*************************************************************************
*                   LOCAL TASK PROTOTYPES
*************************************************************************/

static void appTaskLinkLed(void *);
static void appTaskConnectLed(void *);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

void main() {
  /* Initialise the hardware */
  bspInit();
  
  /* Create the tasks */
  xTaskCreate(appTaskLinkLed, "LinkLed", configMINIMAL_STACK_SIZE, NULL, USB_LINK_LED_TASK_PRIORITY, NULL);
  xTaskCreate(appTaskConnectLed, "ConnectLed", configMINIMAL_STACK_SIZE, NULL, USB_CONNECT_LED_TASK_PRIORITY, NULL);

  /* Start the scheduler.

     NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
     The processor MUST be in supervisor mode when vTaskStartScheduler is
     called.  The demo applications included in the FreeRTOS.org download switch
     to supervisor mode prior to main being called.  If you are not using one of
     these demo application projects then ensure Supervisor mode is used here.
  */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler. */
  return;
}

static void appTaskLinkLed(void *pdata) {
  while (true) {
    ledToggle(USB_LINK_LED);
    vTaskDelay(250);
  }
}

static void appTaskConnectLed(void *pdata) {
  while (true) {
    ledToggle(USB_CONNECT_LED);
    vTaskDelay(500);
  }
}