/* Robot 1 (Input Robot)
 * Joel O'Halleron (w14012537)
 * Last Edited: 29/4/2018
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

enum {
      APP_TASK_CAN_RECEIVE_PRIO = 5,
      APP_TASK_ROBOT_CONTROL_PRIO,
      APP_TASK_CAN_SEND_PRIO,
      APP_TASK_CAN_MONITOR_PRIO};

/****************************************************************************
*                  APPLICATION TASK STACKS
****************************************************************************/

enum {
      APP_TASK_CAN_RECEIVE_STK_SIZE = 256,
      APP_TASK_ROBOT_CONTROL_STK_SIZE = 256,
      APP_TASK_CAN_MONITOR_STK_SIZE = 256,
      APP_TASK_CAN_SEND_STK_SIZE = 256};


static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];
static OS_STK appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE];
static OS_STK appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE];
static OS_STK appTaskRobotControlStk[APP_TASK_ROBOT_CONTROL_STK_SIZE];

/*****************************************************************************
*                APPLICATION FUNCTION PROTOTYPES
*****************************************************************************/


static void appTaskCanReceive(void *pdata);
static void appTaskCanMonitor(void *pdata);
static void appTaskRobotControl(void *pdata);
static void appTaskCanSend(void *pdata);


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
void sendMessage(uint32_t);
void setState(uint32_t NEWSTATE);
void processMessage(uint32_t msgID);

/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *STATEsem;
static OS_EVENT *MessageBufSem;
static canMessage_t can1RxBuf;
static uint32_t STATE;
static bool ROBOT_MOVING;
static uint32_t canMessageBuf[5];
INT8U error;
static bool nack_conv_recieved;
static int retries = 1;
static bool PAD1_WAITING;
static bool ROBOT_HAS_BLOCK;
static uint32_t LASTSTATE;


/*****************************************************************************
*                        GLOBAL FUNCTION DEFINITIONS
*****************************************************************************/


int main() {

  
  /* Initialise the hardware */
  bspInit();
  robotInit();

  /* Initialise the OS */
  OSInit();                                                   

 
  OSTaskCreate(appTaskCanReceive,                               
               (void *)0,
               (OS_STK *)&appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE - 1],
               APP_TASK_CAN_RECEIVE_PRIO);
 
  OSTaskCreate(appTaskCanMonitor,                               
               (void *)0,
               (OS_STK *)&appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE - 1],
               APP_TASK_CAN_MONITOR_PRIO);
  
  OSTaskCreate(appTaskRobotControl,                               
               (void *)0,
               (OS_STK *)&appTaskRobotControlStk[APP_TASK_ROBOT_CONTROL_STK_SIZE - 1],
               APP_TASK_ROBOT_CONTROL_PRIO);
  
  OSTaskCreate(appTaskCanSend,                               
               (void *)0,
               (OS_STK *)&appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE - 1],
               APP_TASK_CAN_SEND_PRIO);
  
  
  /* Create Semaphores and Mutexes */
  can1RxSem = OSSemCreate(0);

  STATEsem = OSSemCreate(1);
  MessageBufSem = OSSemCreate(1);

  /* Start the OS */
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/**************************************************************************
*                             APPLICATION TASK DEFINITIONS
****************************************************************************/


  /* 
   * Main CANread task which reads the CAN bus and processes messages
   * based on the message Enumeration
  */
  static void appTaskCanReceive(void *pdata) {
  uint8_t error;
  canMessage_t msg;
  
  /* Install the CAN interrupt handler and start the OS ticker
   * (must be done in the highest priority task)*/
   
  canRxInterrupt(canHandler);
  osStartTick();
    
  /* 
   * Now execute the main task loop for this task*/
        
  while ( true ) {
    OSSemPend(can1RxSem, 0, &error);//Pending on access to the CAN semaphore
     
    msg = can1RxBuf;//The can message to be checked is taken from the CAN buffer

      processMessage(msg.id);
    
  }
 }

 /*
  * Modular task to send messages during the run of the system dependent on
  * the current STATE of the system
 */
static void appTaskCanSend(void *pdata) {
  
  while(true){
    
    if(STATE == EM_STOP)
    {
      sendMessage(EM_STOP_ACK_ROB1);
    }else
    if(STATE == RESET)
    {
	  retries = 1;
      sendMessage(RESET_ACK_ROB1);
    }else
    if(STATE == ERR_ROB1)
    {
      sendMessage(ERR_ROB1);
    } else
    if(STATE == PAUSE)
    {
      sendMessage(PAUSE_ACK_ROB1);
    }else
     if(STATE == RESUME)
    {
      sendMessage(RESUME_ACK_ROB1);
      setState(LASTSTATE);
    }else
      if(!ROBOT_MOVING)
      {
        if(STATE == REQ_PICKUP_PAD1 && !ROBOT_MOVING)
        {
         sendMessage(ACK_PICKUP_PAD1);
        } else
        if(STATE == CHK_PAD1_PICKUP && !ROBOT_MOVING)
        {
          sendMessage(CHK_PAD1_PICKUP);
          OSTimeDly(500);
        } else
        if(STATE == REQ_DROP_CONV && !ROBOT_MOVING)
        {
          sendMessage(REQ_DROP_CONV);
        } else
        if(STATE == CHK_CONV_DROP && !ROBOT_MOVING)
        {
          sendMessage(CHK_CONV_DROP);
        } else
        if(STATE == NACK_CHK_CONV_DROP && !ROBOT_MOVING)
        {
          sendMessage(ERR_ROB1);
        }
      }
    
    OSTimeDly(50);
  }
  
}


/*
 * Modular task to manage the physical actions of the robot depending on the system 
 * STATE
*/
static void appTaskRobotControl(void *pdata)
{
 while (true) {
   if(STATE != EM_STOP)
   {
    //Start State:
    switch(STATE)
    {
      case RESET               :  ROBOT_MOVING = true;
                                  setRobotStart(); //Helper function to move robot joints into neutral position
                                  ROBOT_MOVING = false;
                                  OSTimeDly(20);
                                       break;
      case START               :  ROBOT_MOVING = true;
                                  setRobotStart();//Helper function to move robot joints into neutral position
                                  ROBOT_MOVING = false;
                                       break;
      case REQ_PICKUP_PAD1     :  if(!ROBOT_HAS_BLOCK || !EM_STOP || !PAUSE)
                                  {
                                    ROBOT_MOVING = true;
                                    pickUpPad1();//Pick up block
                                    ROBOT_MOVING = false;
                                  }
                                  if(STATE == EM_STOP || STATE == PAUSE)
                                      {
                                        break;
                                      }else
                                    {setState(CHK_PAD1_PICKUP);} //Sets the state of the subsystem
                                   break;
      case ACK_CHK_PAD1_PICKUP :  ROBOT_MOVING = true;
                                  moveAboveConveyor();//Helper function to move the robot in position to drop the block on conveyor
                                  ROBOT_MOVING = false;
                                  if(STATE == EM_STOP || STATE == PAUSE)
                                  {
                                    break;
                                  }else
                                  {setState(REQ_DROP_CONV);}
                                       break;
      case NACK_CHK_PAD1_PICKUP:  ROBOT_MOVING = true;
                                  if(retries==4)
                                  {
                                    sendMessage(ERR_ROB1); //Function to send a message on the CAN
                                  }
                                  else{
                                    retries++;
                                    pickUpPad1();//Pick up block
                                    setState(CHK_PAD1_PICKUP);
                                  }
                                  ROBOT_MOVING = false;
                                       break;
      case ACK_DROP_CONV       :  if(ROBOT_HAS_BLOCK)
                                  {
                                    ROBOT_MOVING = true;
                                    dropBlockConveyor();//Helper function to drop the block onto the conveyor
                                    ROBOT_MOVING = false;
                                  }
                                  if(STATE == EM_STOP || STATE == PAUSE)
                                  {
                                    break;
                                  }else{
                                  OSTimeDly(100);
                                  setState(CHK_CONV_DROP);}
                                       break;
      default: break;
    }
   }
   OSTimeDly(50);
  }
}


/*
 * Modular task to monitor the system STATE and the current messages being received 
 * on the CAN bus. Provides useful debug and system information.
*/
static void appTaskCanMonitor(void *pdata) {
  canMessage_t msg;
  
  while (true) {

    //Debug code displayed on LCD
    interfaceLedToggle(D1_LED);
    msg = can1RxBuf;
    lcdSetTextPos(2,1);
    lcdWrite(displayMessageContents[msg.id]);
    lcdSetTextPos(2,2);
    lcdWrite("msg.id : %08d", msg.id);
    lcdSetTextPos(2,3);
    lcdWrite("State: ");
    lcdSetTextPos(2,4);
    lcdWrite(displayMessageContents[STATE]);
    lcdSetTextPos(2,5);
    lcdWrite("PAD1_WAITING : %d", PAD1_WAITING);
    lcdSetTextPos(2,6);
    lcdWrite("ROBOT_HAS_BLOCK : %d", ROBOT_HAS_BLOCK);
    lcdSetTextPos(2,7);
    lcdWrite("Retries: %d", retries);
    lcdSetTextPos(2,8);
    lcdWrite("Conv Nack: %d", nack_conv_recieved);
    
    OSTimeDly(500);
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

/*
 * Useful function allowing input of a joint and the desired position
 * allowing movement of the robot to these specified positions
 *
 * @param robotJoint_t joint Joint value based on the joint enumeration
 *                           in Robot.h 
 *
 * @param uint32_t newPos    The desired position value for the joint in question
*/
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
      
         /*This code assists in the pause function
		 * If pause is pressed as the robot has almost fully closed it's hand, the Robot will continue 
		 * as if it had picked up the block and run a check with the controller to see if it has indeed.
		 * This stops an unnecessary run through the pickUpBlock() function.
		 */
		 if(robotJointGetState(ROBOT_HAND) >= 66000)
        {
          ROBOT_HAS_BLOCK = true;
        }
        else 
        {
          ROBOT_HAS_BLOCK = false;
        }
        
        if(STATE == EM_STOP || STATE == PAUSE)
        {

          break;
        }
        else{  
          robotJointSetState(joint, direction);
          OSTimeDly(10);
        }
      
    }
  
}
  
// Moves the robot to pre-determined positions for a pick up of a block on Pad1
void pickUpPad1(void){
  
       /*robotMoveJointTo(ROBOT_ELBOW, 83500);
        OSTimeDly(500);*/
		robotMoveJointTo(ROBOT_HAND, 45000);
        OSTimeDly(500);
        robotMoveJointTo(ROBOT_WAIST, 86500);
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

//Moves the robot to pre-determined positions moving it above the conveyor sensor1
void moveAboveConveyor(void){
        
        robotMoveJointTo(ROBOT_HAND, 71000);
        robotMoveJointTo(ROBOT_ELBOW, 83500);
        robotMoveJointTo(ROBOT_WAIST, 45000);
       
}

//Moves the robot to pre-determined positions to drop a block on the conveyor
void dropBlockConveyor(void){
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      
}

//Moves the robot to pre-determined positions, resseting it to a neutral position
void setRobotStart(void){
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 87500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 67250);  
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 82250);
      OSTimeDly(500);
}

/* Helper function to allow the system STATE to be set 
 * Use of a semaphore ensures that the STATE can't be set
 * by two tasks at the same time, removing the possibility of bugs
 * where the STATE advances prematurely without the necessary operations
 * for the previous STATE having been completed
 *
 * @param uint32_t NEWSTATE Takes this state and assigns the STATE value to
 *                          it's value
 * 
*/
void setState(uint32_t NEWSTATE)
{
    OSSemPend(STATEsem, 0, &error);
    STATE = NEWSTATE;
    error = OSSemPost(STATEsem); 
}

/*Function to send a message
 * 
 * @param uint32_t message Takes a CAN Message ID as parameter to be broadcast on CAN
 */
void sendMessage(uint32_t message)
{
  canMessage_t msg;
  msg.id = message;//Assigns the message to the can message ID
  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
}

/* Function to process a received message
 *
 * @param uint32_t msgID  Takes the message ID of the received message
 *                        and based on how the ID corresponds to the 
 *                        message Enumeration, sets the system state
 *                        or in urgent cases sends a message out on the CAN
 */
void processMessage(uint32_t msgID)
{
  if(msgID == EM_STOP)
  {
    setState(EM_STOP);
  }else
  if(msgID == PAUSE)
  {
    LASTSTATE = STATE;
    setState(PAUSE);
  } else
  if(msgID == RESET)
  {
    setState(RESET);
  } else
   if(msgID == RESUME)
   {
    setState(RESUME);
   }
  else
    if(msgID == CTRL_STOP)
    {
      sendMessage(CTRL_STOP_ACK_ROB1);
    }
  //Start message response code
  if(msgID == START && STATE < REQ_PICKUP_PAD1)
  {
    setState(START);
    sendMessage(START_ACK_ROB1);
  }
  else if(msgID == REQ_PICKUP_PAD1 && STATE < REQ_PICKUP_PAD1)
  {
    setState(REQ_PICKUP_PAD1);
  }
  else if(msgID == ACK_CHK_PAD1_PICKUP)
  {
      setState(ACK_CHK_PAD1_PICKUP);
      retries = 1;
  }
  else if(msgID == NACK_CHK_PAD1_PICKUP)
  {
        setState(NACK_CHK_PAD1_PICKUP);
  }
  else if(msgID == ACK_DROP_CONV && STATE < CHK_CONV_DROP && !ROBOT_MOVING)
  {
    setState(ACK_DROP_CONV);
  }
  else if(msgID == NACK_DROP_CONV)
  {
    OSTimeDly(500);
    setState(REQ_DROP_CONV);
  }
  else if(msgID == ACK_CHK_CONV_DROP)
  {
    if(PAD1_WAITING && !ROBOT_HAS_BLOCK)
    {
      setState(REQ_PICKUP_PAD1);
      PAD1_WAITING = false;
    }else{setState(START);} 
  }
  else if(msgID == NACK_CHK_CONV_DROP)
  {
    nack_conv_recieved = true;
    setState(ERR_ROB1);
  }
  
}

;
  
