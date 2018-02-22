/* Conveyor Test
 * JS UP -> forward
 *    DN -> reverse
 *    L,R -> stop
 * 
 * Conveyor sensor 1 reported  on interface LED 1 and LINK LED
 * Conveyor sensor 2 reported  on interface LED 2 and CONNECT LED
 *
 * item sensed at 1 and moving in reverse -> stop
 * item sensed at 2 and moving forward -> stop
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <interface.h>
#include <conveyor.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_PRIO = 4,
  APP_TASK_CTRL_CONV_PRIO
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_STK_SIZE = 256,
  APP_TASK_CTRL_CONV_STK_SIZE = 256
};

static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];
static OS_STK appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens(void *pdata);
static void appTaskCtrlConv(void *pdata);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
 
  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskMonitorSens,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
   
  OSTaskCreate(appTaskCtrlConv,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE - 1],
               APP_TASK_CTRL_CONV_PRIO);
   
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskMonitorSens(void *pdata) {
    
  /* Start the OS ticker
   * (must be done in the highest priority task)
   */
  osStartTick();
  
  /* 
   * Now execute the main task loop for this task
   */
  while (true) {
    interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
    ledSetState(USB_LINK_LED, LED_OFF);
    ledSetState(USB_CONNECT_LED, LED_OFF);
    
    if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {
        interfaceLedSetState(D1_LED, LED_ON);
        ledSetState(USB_LINK_LED, LED_ON);
    } 
    if (conveyorItemPresent(CONVEYOR_SENSOR_2)) {
        interfaceLedSetState(D2_LED, LED_ON);
        ledSetState(USB_CONNECT_LED, LED_ON);
    } 
    
    OSTimeDly(20);
  }
}

static void appTaskCtrlConv(void *pdata) {
  uint32_t btnState;
  
  while (true) {
    static bool jsLRPressed = false;
    static bool jsUpPressed = false;
    static bool jsDownPressed = false;
    btnState = buttonsRead();

    if (isButtonPressedInState(btnState, JS_LEFT) || 
        isButtonPressedInState(btnState, JS_RIGHT)) {
      jsLRPressed = true;
    }
    if (jsLRPressed && (!isButtonPressedInState(btnState, JS_LEFT))
                    && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      jsLRPressed = false;
      conveyorSetState(CONVEYOR_OFF);
    }

    if (isButtonPressedInState(btnState, JS_UP)) {
      jsUpPressed = true;
    }
    if (jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
      jsUpPressed = false;
      conveyorSetState(CONVEYOR_OFF);
      OSTimeDly(250);
      conveyorSetState(CONVEYOR_FORWARD);
    }
    
    if (isButtonPressedInState(btnState, JS_DOWN)) {
      jsDownPressed = true;
    }
    if (jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
      jsDownPressed = false;
      conveyorSetState(CONVEYOR_OFF);
      OSTimeDly(250);
      conveyorSetState(CONVEYOR_REVERSE);
    }
    
    if (conveyorItemPresent(CONVEYOR_SENSOR_1) &&
        conveyorGetState() == CONVEYOR_REVERSE) {
      conveyorSetState(CONVEYOR_OFF);
    } 

    if (conveyorItemPresent(CONVEYOR_SENSOR_2) &&
        conveyorGetState() == CONVEYOR_FORWARD) {
      conveyorSetState(CONVEYOR_OFF);
    } 
  } 
}

