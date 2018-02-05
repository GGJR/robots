/* CAN Receiver: Interrupt-driven
 *
 * Process CAN Rx interrupts and toggle interface LED D1 on reception
 *
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <can.h>
#include <leds.h>
#include <lcd.h>
#include <interface.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  DISPLAY_MUTEX_PRIO = 4,
  APP_TASK_CAN_RECEIVE_PRIO,
  APP_TASK_CAN_MONITOR_PRIO
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_CAN_RECEIVE_STK_SIZE = 256,
  APP_TASK_CAN_MONITOR_STK_SIZE = 256
};

static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];
static OS_STK appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskCanReceive(void *pdata);
static void appTaskCanMonitor(void *pdata);

/*
*************************************************************************
*                 LOCAL FUNCTION PROTOTYPES
*************************************************************************
*/

static void canHandler(void);

/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *displayMutex;
static canMessage_t can1RxBuf;


/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

int main() {
  uint8_t error;

  /* Initialise the hardware */ 
  bspInit();
  interfaceInit(NO_DEVICE);

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskCanReceive,                               
               (void *)0,
               (OS_STK *)&appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE - 1],
               APP_TASK_CAN_RECEIVE_PRIO);
 
  OSTaskCreate(appTaskCanMonitor,                               
               (void *)0,
               (OS_STK *)&appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE - 1],
               APP_TASK_CAN_MONITOR_PRIO);

  /* Create Semaphores and Mutexes */
  can1RxSem = OSSemCreate(0);
  displayMutex = OSMutexCreate(DISPLAY_MUTEX_PRIO, &error);
      
  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskCanReceive(void *pdata) {
  uint8_t error;
  canMessage_t msg;
  
  /* Install the CAN interrupt handler and start the OS ticker
   * (must be done in the highest priority task)
   */
  canRxInterrupt(canHandler);
  osStartTick();

  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    interfaceLedToggle(D1_LED);
    OSMutexPend(displayMutex, 0, &error);
    lcdSetTextPos(2,1);
    lcdWrite("ID     : %08x", msg.id); 
    lcdSetTextPos(2,2);
    lcdWrite("LEN    : %08x", msg.len); 
    lcdSetTextPos(2,3);
    lcdWrite("DATA_A : %08x", msg.dataA); 
    lcdSetTextPos(2,4);
    lcdWrite("DATA_B : %08x", msg.dataB); 
    error = OSMutexPost(displayMutex);
  }
}

static void appTaskCanMonitor(void *pdata) {
  uint8_t error;
  
  while (true) {
    OSMutexPend(displayMutex, 0, &error);
    lcdSetTextPos(2,5);
    lcdWrite("CAN1GSR: %08x", canStatus(CAN_PORT_1));
    error = OSMutexPost(displayMutex);
    OSTimeDly(20);
  }
}


/*
 * A simple interrupt handler for CAN message reception on CAN1
 */
static void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    OSSemPost(can1RxSem);
  }
}
