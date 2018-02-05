/* Control Panel Test
 * Emergency stop -> toggle Alarm State
 *   Data LED 3 lit while emergency stop button pressed
 *
 * Pedestal sensor 1 reported  on interface LED 1 and LINK LED
 * Pedestal sensor 2 reported  on interface LED 2 and CONNECT LED
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <interface.h>
#include <control.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_PRIO = 4,
  APP_TASK_CTRL_PRIO
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_MONITOR_SENS_STK_SIZE = 256,
  APP_TASK_CTRL_STK_SIZE = 256
};

static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];
static OS_STK appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens(void *pdata);
static void appTaskCtrl(void *pdata);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  /* Initialise the hardware */
  bspInit();
  controlInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskMonitorSens,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
   
  OSTaskCreate(appTaskCtrl,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE - 1],
               APP_TASK_CTRL_PRIO);
   
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
    interfaceLedSetState(D1_LED | D2_LED, LED_OFF);
    ledSetState(USB_LINK_LED, LED_OFF);
    ledSetState(USB_CONNECT_LED, LED_OFF);
    
    if (controlItemPresent(CONTROL_SENSOR_1)) {
        interfaceLedSetState(D1_LED, LED_ON);
        ledSetState(USB_LINK_LED, LED_ON);
    } 
    if (controlItemPresent(CONTROL_SENSOR_2)) {
        interfaceLedSetState(D2_LED, LED_ON);
        ledSetState(USB_CONNECT_LED, LED_ON);
    } 
    
    OSTimeDly(20);
  }
}

static void appTaskCtrl(void *pdata) {
  static bool emergency = false;
  interfaceLedSetState(D3_LED | D4_LED, LED_OFF);
  
  while (true) {
    emergency = controlEmergencyStopButtonPressed();
    if (emergency) {
      controlAlarmToggleState();
      interfaceLedSetState(D4_LED, LED_ON);
      while (controlEmergencyStopButtonPressed()) {
        OSTimeDly(20);
      }
    } else {
      interfaceLedSetState(D4_LED, LED_OFF);
    }
    OSTimeDly(20);
  } 
}

