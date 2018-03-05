/* Test Robot
 * Joystick UP    -> inc current joint coordinate
 * Joystick DOWN  -> dec current joint coordinate
 * Joystick RIGHT -> cycle joint selection HAND -> WRIST -> ELBOW -> WAIST
 * Joystick left  -> cycle joint selection HAND <- WRIST <- ELBOW <- WAIST
 */
#include <stdbool.h>
#include <ucos_ii.h>
#include <osutils.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <can.h>
#include <buttons.h>
#include <lcd.h>
#include <interface.h>
#include <robot.h>
#include <messageTable.h>

#define BUTTONS_TASK_ID 0
#define N_JOINTS 4

/***************************************************************************
*                       PRIORITIES
***************************************************************************/

enum {DISPLAY_MUTEX_PRIO = 5,
      APP_TASK_CAN_RECEIVE_PRIO,
      APP_TASK_CAN_MONITOR_PRIO,
      APP_TASK_BUTTONS_PRIO};

/****************************************************************************
*                  APPLICATION TASK STACKS
****************************************************************************/

enum {APP_TASK_BUTTONS_STK_SIZE = 256,
      APP_TASK_CAN_RECEIVE_STK_SIZE = 256,
      APP_TASK_CAN_MONITOR_STK_SIZE = 256};

static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];
static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];
static OS_STK appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE];

/*****************************************************************************
*                APPLICATION FUNCTION PROTOTYPES
*****************************************************************************/

static void appTaskButtons(void *pdata);
static void appTaskCanReceive(void *pdata);
static void appTaskCanMonitor(void *pdata);

/*
*************************************************************************
*                 LOCAL FUNCTION PROTOTYPES
*************************************************************************
*/
static void canHandler(void);
void robotMoveJointTo(robotJoint_t, uint32_t);

/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *LCDsem;
static canMessage_t can1RxBuf;
INT8U error;

/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/


int main() {
  uint8_t error;
  
  /* Initialise the hardware */
  bspInit();
  robotInit();

  /* Initialise the OS */
  OSInit();                                                   

  /* Create Tasks */
  OSTaskCreate(appTaskButtons,                               
               (void *)0,
               (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
               APP_TASK_BUTTONS_PRIO);
 
  OSTaskCreate(appTaskCanReceive,                               
               (void *)0,
               (OS_STK *)&appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE - 1],
               APP_TASK_CAN_RECEIVE_PRIO);
 
  OSTaskCreate(appTaskCanMonitor,                               
               (void *)0,
               (OS_STK *)&appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE - 1],
               APP_TASK_CAN_MONITOR_PRIO);
  
  /* Create Semaphores and Mutexes */
  can1RxSem = OSSemCreate(1);
  LCDsem = OSSemCreate(1);

  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/**************************************************************************
*                             APPLICATION TASK DEFINITIONS
****************************************************************************/


static void appTaskButtons(void *pdata) {
  
  uint32_t btnState;
  static uint8_t joint = ROBOT_HAND;
  static uint32_t leds[5] = {D1_LED, D1_LED, D2_LED, D3_LED, D4_LED};
  static bool jsRightPressed = false;
  static bool jsLeftPressed = false;
  


  
  /* the main task loop for this task  */
  while (true) {
    btnState = buttonsRead();
    if (isButtonPressedInState(btnState, JS_RIGHT)) {
      jsRightPressed = true;
    }
    if (jsRightPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      jsRightPressed = false;
      interfaceLedSetState(leds[joint], LED_OFF);
      joint += 1;
      if (joint > N_JOINTS) {
        joint = 1;
      }
    }
    if (isButtonPressedInState(btnState, JS_LEFT)) {
      jsLeftPressed = true;
    }
    if (jsLeftPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
      jsLeftPressed = false;
      interfaceLedSetState(leds[joint], LED_OFF);
      joint -= 1;
      if (joint == 0) {
        joint = N_JOINTS;
      }
    }
    interfaceLedSetState(leds[joint], LED_ON);
    
    if (isButtonPressedInState(btnState, JS_UP)) {
      
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 86950);  
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500); 
      robotMoveJointTo(ROBOT_ELBOW, 99500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 71000);
      OSTimeDly(500);
      
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    } else if (isButtonPressedInState(btnState, JS_DOWN)) {
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      robotMoveJointTo(ROBOT_WAIST, 45000);
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      robotMoveJointTo(ROBOT_HAND, 45000);
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    }
    OSTimeDly(20);
  }
}
  

  static void appTaskCanReceive(void *pdata) {
  uint8_t error;
  canMessage_t msg;
  
  /* Install the CAN interrupt handler and start the OS ticker
   * (must be done in the highest priority task)
   */
  canRxInterrupt(canHandler);
  osStartTick();
  
   //Local variables
uint32_t counter=0;
    uint32_t RUNNING = 0;
    uint32_t PAD_WAITING = 0;
    uint32_t BLOCK_IN_TRANSIT = 0;
   // bool WAITING_ON_MESSAGE = false;
    

  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
      OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    
    if(msg.id == START && !RUNNING)
    {
      RUNNING = 11;//true
      robotMoveJointTo(ROBOT_ELBOW, 87500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 67250);  
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 82250);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 68750);
      
      msg.id = START_ACK_ROB1;
      canWrite(CAN_PORT_1, &msg);
      
    }
    
    if(msg.id == REQ_PICKUP_PAD1)
    {
      PAD_WAITING = 22;//true
      
      msg.id = ACK_PICKUP_PAD1;
      canWrite(CAN_PORT_1, &msg);
    }
    
 
    
    
    if(PAD_WAITING && !BLOCK_IN_TRANSIT)
    {
            counter++;

      PAD_WAITING = 0;//false
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 85000); //left 
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500); 
      robotMoveJointTo(ROBOT_ELBOW, 99500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 68750);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      
      
      msg.id = CHK_PAD1_PICKUP;
      canWrite(CAN_PORT_1, &msg); 
      //WAITING_ON_MESSAGE = true;
      OSTimeDly(500);
    }
    
    if(msg.id == NACK_CHK_PAD1_PICKUP) 
    {
      PAD_WAITING = 23;//true
      BLOCK_IN_TRANSIT = 0; //false
    }
   
    
    //successful pickup 
    if(msg.id == ACK_CHK_PAD1_PICKUP) 
    {
      //WAITING_ON_MESSAGE = false;
      BLOCK_IN_TRANSIT = 44;  
      
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 71000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 45000);
      OSTimeDly(500);
      
      msg.id = REQ_DROP_CONV;
      canWrite(CAN_PORT_1, &msg);
    }
    
   
    if(msg.id == ACK_DROP_CONV && BLOCK_IN_TRANSIT) 
    {

      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      BLOCK_IN_TRANSIT = 0;
      
      msg.id = CHK_CONV_DROP;
      canWrite(CAN_PORT_1, &msg);
    }
    
    if(msg.id == ACK_CHK_CONV_DROP && BLOCK_IN_TRANSIT)
    {
      robotMoveJointTo(ROBOT_ELBOW, 87500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 67250);  
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 82250);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 68750);
    }

    
    interfaceLedToggle(D1_LED);
    OSSemPend(LCDsem, 0, &error);
    lcdSetTextPos(2,1);
    lcdWrite(displayMessageContents[msg.id]); 
    lcdSetTextPos(2,2);
    lcdWrite("msg.id : %08d", msg.id); 
    lcdSetTextPos(2,3);
    lcdWrite("LEN    : %08x", msg.len); 
    lcdSetTextPos(2,4);
    lcdWrite("DATA_A : %08x", msg.dataA); 
    lcdSetTextPos(2,5);
    lcdWrite("DATA_B : %08x", msg.dataB);
    
    
    lcdSetTextPos(2,7);  
    lcdWrite("RUNNIN : %08d", RUNNING); 
    lcdSetTextPos(2,8);
    lcdWrite("PAD_WG : %08d", PAD_WAITING); 
    lcdSetTextPos(2,9);
    lcdWrite("BLkTran: %08d", BLOCK_IN_TRANSIT); 
    lcdSetTextPos(2,9);
    lcdWrite("counter: %08d", counter);
    error = OSSemPost(LCDsem);
  }
}


static void appTaskCanMonitor(void *pdata) {
  uint8_t error;
  
  while (true) {
    OSSemPend(LCDsem, 0, &error);
    lcdSetTextPos(2,5);
    lcdWrite("CAN1GSR: %08x", canStatus(CAN_PORT_1));
    error = OSSemPost(LCDsem);
    OSTimeDly(20);
  }
}



/*
 * A simple interrupt handler for CAN message reception on CAN1
 */
static void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    error = OSSemPost(can1RxSem);
  }
}

void robotMoveJointTo(robotJoint_t joint, uint32_t newPos) {
  
  uint32_t targetPos = newPos;
  robotJointStep_t direction;
  
  if(robotJointGetState(joint) > targetPos) {
   direction = ROBOT_JOINT_POS_DEC;
  }
  else {
    direction = ROBOT_JOINT_POS_INC;
  }
  
  while(robotJointGetState(joint) != targetPos) {
    robotJointSetState(joint, direction);
    OSTimeDly(10);
    
  }

};

