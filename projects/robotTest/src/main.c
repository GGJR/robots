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
void pickUpPad1(void);
void moveAboveConveyor(void);
void dropBlockConveyor(void);
void setRobotStart(void);

/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *LCDsem;
static canMessage_t can1RxBuf;
static uint32_t STATE;
INT8U error;
static int retries = 0;


/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/


int main() {

  
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
  
    

  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    OSSemPend(can1RxSem, 0, &error);//Pending on access to the CAN semaphore
	
    msg = can1RxBuf;//The can message to be checked is taken from the CAN buffer
    
    //Start Message Response Code//
    if(msg.id == START && STATE < START)
    { 
      msg.id = START_ACK_ROB1;//Assigns the message START_ACK_ROB1 to the can message ID
      canWrite(CAN_PORT_1, &msg);//Acknowledge Start command
	  STATE = START;//Set robot to start state
    }
    
    if(msg.id == REQ_PICKUP_PAD1 && STATE < REQ_PICKUP_PAD1)
    {
      msg.id = ACK_PICKUP_PAD1;//Assigns the message ACK_PICKUP_PAD1 to the can message ID
      canWrite(CAN_PORT_1, &msg);//Acknowledge the pickup request
      
      pickUpPad1();//Pick up block
      
      msg.id = CHK_PAD1_PICKUP;//Assigns the message CHK_PAD1_PICKUP to the can message ID
      canWrite(CAN_PORT_1, &msg);//Send controller the check for block pickup
      
      STATE = CHK_PAD1_PICKUP;//Set robot to the state of checking the success of Pad 1 pickup
     
    }
   
    //Unsuccessful Pickup
    if(msg.id == NACK_CHK_PAD1_PICKUP && STATE == CHK_PAD1_PICKUP)
    { 
		if(retries == 3)
		{
			STATE = ERR_ROB1;//In the event of the retries exceeding 3 the Robot enters an error state
			msg.id = ERR_ROB1;//Assigns the message ERR_ROB1 to the can message ID
			canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
		}
		else{
		  pickUpPad1();//Helper function to pick up the block from Pad 1
		  
		  msg.id = CHK_PAD1_PICKUP;//Assigns the message CHK_PAD1_PICKUP to the can message ID
		  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
		  retries++;//Increments the number of retries
		  STATE = CHK_PAD1_PICKUP;//Robot is in the state of 'check the block has been picked up successfully'
		  
		  OSTimeDly(1000);//System time delay in milliseconds
		}
    }
   
    //Successful Pickup 
    if(msg.id == ACK_CHK_PAD1_PICKUP && STATE == CHK_PAD1_PICKUP) 
    { 
      moveAboveConveyor();//Helper function to move the robot in position to drop the block on conveyor
	  
      msg.id = REQ_DROP_CONV;//Assigns the message REQ_DROP_CONV to the can message ID
      canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
	  STATE = REQ_DROP_CONV;//Robot has requested a drop on conveyor, and is awaiting a reply
    }
    
	//Acknowledgment of conveyor, ready for drop off
    if(msg.id == ACK_DROP_CONV && STATE == REQ_DROP_CONV) 
    {
      dropBlockConveyor();//Helper function to drop the block onto the conveyor
 
      msg.id = CHK_CONV_DROP;//Assigns the message CHK_CONV_DROP to the can message ID
      canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
	  STATE = CHK_CONV_DROP;//Robot believes the block is dropped and polls the conveyor
    }
    
	//Acknowledgment of conveyor that block drop-off was successful
    if(msg.id == ACK_CHK_CONV_DROP && STATE == CHK_CONV_DROP)
    {
      setRobotStart();//Helper function sets joints to neutral
	  STATE = START;//Resets the robot to it's starting state
    }
	//End Message Response Code//

    
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
}
  
void pickUpPad1(void){
  
        robotMoveJointTo(ROBOT_ELBOW, 83500);
        OSTimeDly(500);
        robotMoveJointTo(ROBOT_WAIST, 85000);
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
}

void moveAboveConveyor(void){
        
        robotMoveJointTo(ROBOT_HAND, 71000);
        robotMoveJointTo(ROBOT_ELBOW, 83500);
        robotMoveJointTo(ROBOT_WAIST, 45000);
       
}

void dropBlockConveyor(void){
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      
}

void setRobotStart(void){
      robotMoveJointTo(ROBOT_ELBOW, 87500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 67250);  
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 82250);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 68750);
}

;

