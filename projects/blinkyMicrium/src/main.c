/* 
 * Blinky with uC/OS-II
 *
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>


/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_USB_LINK_LED_PRIO = 4,
  APP_TASK_USB_CONNECT_LED_PRIO
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_USB_LINK_LED_STK_SIZE = 256,
  APP_TASK_USB_CONNECT_LED_STK_SIZE = 256
};

static OS_STK appTaskUsbLinkLedStk[APP_TASK_USB_LINK_LED_STK_SIZE];
static OS_STK appTaskUsbConnectLedStk[APP_TASK_USB_CONNECT_LED_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskUsbLinkLed(void *pdata);
static void appTaskUsbConnectLed(void *pdata);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskUsbLinkLed,                               
               (void *)0,
               (OS_STK *)&appTaskUsbLinkLedStk[APP_TASK_USB_LINK_LED_STK_SIZE - 1],
               APP_TASK_USB_LINK_LED_PRIO);
 
  OSTaskCreate(appTaskUsbConnectLed,                               
               (void *)0,
               (OS_STK *)&appTaskUsbConnectLedStk[APP_TASK_USB_CONNECT_LED_STK_SIZE - 1],
               APP_TASK_USB_CONNECT_LED_PRIO);

  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskUsbLinkLed(void *pdata) {
    
  /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();

  
  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    ledToggle(USB_LINK_LED);
    OSTimeDly(250);
  }
}

static void appTaskUsbConnectLed(void *pdata) {

  while ( true ) {
    ledToggle(USB_CONNECT_LED);
    OSTimeDly(500);
  }
}
