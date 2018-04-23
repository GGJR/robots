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
#include <robot.h> //<-- put position coordinates here
#include <messageTable.h>

#define BUTTONS_TASK_ID 0
#define N_JOINTS 4

/***************************************************************************
*                       POSITION COORDINATES
***************************************************************************/
// Data structure robotCoordinates 
typedef struct robotCoordinates{
	
	uint32_t elbow;
	uint32_t wrist;
	uint32_t waist;
	uint32_t hand;
	
}robotCoordinates_t;

// Definitions for elbow's position coordinates (from lower to higher order)(^)
#define elbow_82000  82000
#define elbow_83500  83500
#define elbow_87500  87500
#define elbow_100000 100000

// Definition for wrist's position coordinate
#define wrist_90000  90000

// Definitions for waist's position coordinates (from lower to higher order) (^)
#define waist_45000  45000
#define waist_67250  67250
#define waist_68750  68750
#define waist_82250  82250
#define waist_84750  84750

// Definitions for hand's position coordinates (from lower to higher order) (^)
#define hand_45000  45000
#define hand_69000  69000
#define hand_87500  87500

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
void robotStartPos(void);
void pickUpConv(void);

/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *LCDsem;
static canMessage_t can1RxBuf;
INT8U error;
static uint32_t STATE;
static uint32_t LASTSTATE;
static int retries = 0;
static int retriesPad2 = 0;
static bool CONV_WAITING;

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
      robotMoveJointTo(ROBOT_WAIST, 84750);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    } else if (isButtonPressedInState(btnState, JS_DOWN)) {
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WAIST, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 100000); 
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 69000);
      OSTimeDly(500);
      lcdSetTextPos(8, 6+joint);
      lcdWrite("%08u", robotJointGetState((robotJoint_t)joint));
    }
    OSTimeDly(20);
  }
}
  

  static void appTaskCanReceive(void *pdata) {
  uint8_t error;
  canMessage_t msg;
  robotCoordinates_t coordinate;
  
  /* Install the CAN interrupt handler and start the OS ticker
   * (must be done in the highest priority task)
   */
  canRxInterrupt(canHandler);
  osStartTick();
  
  bool running = false;
  bool block_in_transit = false;
  bool conv_waiting = false;
  bool pick_up_success = false;
  bool drop_success = false;

  /* 
   * Now execute the main task loop for this task
   */     
  while ( true ) {
    OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    
    /*if(msg.id == START && !running)
    {
      running = true;
      
      //robot start position
      coordinate.hand = hand_87500;
      robotMoveJointTo(ROBOT_HAND, coordinate.hand); 
      OSTimeDly(500);
      
      coordinate.elbow = elbow_87500;
      robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow); 
      OSTimeDly(500);
      
      coordinate.waist = waist_67250;
      robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
      OSTimeDly(500);
      
      coordinate.waist = waist_67250;
      robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
      OSTimeDly(500);
      
      coordinate.waist = waist_68750;
      robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
      OSTimeDly(500);
      
      msg.id = START_ACK_ROB2;
      canWrite(CAN_PORT_1, &msg);
    }
    
    if(msg.id == PAUSE)
    {
      msg.id = PAUSE_ACK_ROB2;
      canWrite(CAN_PORT_1, &msg);
    }
    
    if(msg.id == REQ_PICKUP_CONV && !block_in_transit)
    {
      
      msg.id = ACK_PICKUP_CONV;
      canWrite(CAN_PORT_1, &msg);
      
      block_in_transit = true;
    
      //Pick up from conveyor
      coordinate.elbow = elbow_83500;
      robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
      OSTimeDly(500);
      
      coordinate.waist = waist_45000;
      robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
      OSTimeDly(500);
      
      coordinate.hand = hand_45000;
      robotMoveJointTo(ROBOT_HAND, coordinate.hand);
      OSTimeDly(500);
      
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500);
      
      robotMoveJointTo(ROBOT_HAND, 69000);
      OSTimeDly(500);
      
      robotMoveJointTo(ROBOT_ELBOW, 83500);
      OSTimeDly(500);
      
      msg.id = CHK_CONV_PICKUP;
      canWrite(CAN_PORT_1, &msg);
      
      lcdSetTextPos(2,10);
      lcdWrite("HERE1 : %08d", block_in_transit);
    }
    
    //Unsuccessful pickup
    if(msg.id == NACK_CHK_CONV_PICKUP)
    {
      block_in_transit = false;
    }
    
    //Successful pickup
    if(msg.id == ACK_CHK_CONV_PICKUP)
    {
      //Move above the conveyor

      robotMoveJointTo(ROBOT_WAIST, 84750);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_WRIST, 90000);
      OSTimeDly(500);
      
      msg.id = REQ_DROP_PAD2;
      canWrite(CAN_PORT_1, &msg);
    }
    
    //DONT drop
    if(msg.id == NACK_DROP_PAD2)
    {
      block_in_transit = true;
    }
    
    //DO drop
    if(msg.id == ACK_DROP_PAD2)
    {
      //Drops to PAD2
      robotMoveJointTo(ROBOT_ELBOW, 100000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_HAND, 45000);
      OSTimeDly(500);
      robotMoveJointTo(ROBOT_ELBOW, 60000);
      OSTimeDly(500);
      
      block_in_transit = false;
      
      msg.id = CHK_PAD2_DROP;
      canWrite(CAN_PORT_1, &msg);
    }
    
    if(msg.id == NACK_CHK_PAD2_DROP)
    {
      drop_success = false;
    }
    
    if(msg.id == ACK_CHK_PAD2_DROP)
    {
      drop_success = true;
    }*/
    if(msg.id == EM_STOP)
    {
        STATE = EM_STOP;//Set state to reset state
        msg.id = EM_STOP_ACK_ROB2;//Assigns the message RESET_ACK_ROB1 to the can message ID
        canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
        OSTimeDly(1000);
    }
    
    //Make sure the Robot is not in an error state
	if(STATE != EM_STOP){
		//Check Robot is not paused
		if(STATE != PAUSE){
                  
                        /////////Start, Pause, Resume and Stop Code////////////////////////////////////////
			//Start command received from the controller
			if(msg.id == START && STATE <= RESET_ACK_ROB2)
			{ 
			  msg.id = START_ACK_ROB2;//Assigns the message START_ACK_ROB1 to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Acknowledge Start command
			  
                          // robot start position
                          robotStartPos();
                          
                          STATE = START;//Set robot to start state
			}
			
			//Pause command received from the controller
			if(msg.id == PAUSE)
			{
				msg.id = PAUSE_ACK_ROB2;//Assigns the message PAUSE_ACK_ROB1 to the can message ID
				canWrite(CAN_PORT_1, &msg);//Acknowledge Start command
				LASTSTATE = STATE;
				STATE = PAUSE;
			}	
			
			//If control pause command is received
			if(msg.id == CTRL_STOP)
			{
				 msg.id = CTRL_STOP_ACK_ROB2;//Assigns the message CTRL_STOP_ACK_ROB1 to the can message ID
				 canWrite(CAN_PORT_1, &msg);//Acknowledge Start command
                                 OSTimeDly(1000);
			}
                        
                        ////CONVEYOR PICKUP CODE///////////////////////////////////////
                       //Block detected on Conveyor, conveyor sends request for pick-up
                       
                        if((msg.id == REQ_PICKUP_CONV || CONV_WAITING == true) && STATE < RESET_ACK_ROB2)
			{
                            msg.id = ACK_PICKUP_CONV;
                            canWrite(CAN_PORT_1, &msg);
                            
                            block_in_transit = true;
                          
                            pickUpConv();
                           
                            OSTimeDly(1000);
                            
                            msg.id = CHK_CONV_PICKUP;
                            canWrite(CAN_PORT_1, &msg);
			  
                            //lcdSetTextPos(2,10);
                            //lcdWrite("HERE1 : %08d", block_in_transit);
                            
                            STATE = CHK_CONV_PICKUP;//Set robot to the state of checking the success of Pad 1 pickup
			 
			}
                        
                        //Block detected on Conveyor, conveyor sends request for pick-up, a block is currently in transit, ack the message
                        if(msg.id == REQ_PICKUP_CONV && STATE > REQ_PICKUP_CONV)
                        {
                          msg.id = ACK_PICKUP_CONV;//Assigns the message ACK_PICKUP_PAD1 to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Acknowledge the pickup request
                          
                          CONV_WAITING = true;
                        }
                          
			
		   
			//Successful Pickup 
			if(msg.id == ACK_CHK_CONV_PICKUP && STATE == CHK_CONV_PICKUP) 
			{ 
			  //moveAboveConveyor();//Helper function to move the robot in position to drop the block on conveyor
                          coordinate.waist = waist_84750;
			  robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
                          OSTimeDly(500);
                          
                          coordinate.wrist = wrist_90000;
                          robotMoveJointTo(ROBOT_WRIST, coordinate.wrist);
                          OSTimeDly(500);
                          
			  msg.id = REQ_DROP_PAD2;//Assigns the message REQ_DROP_CONV to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
			  STATE = REQ_DROP_PAD2;//Robot has requested a drop on conveyor, and is awaiting a reply
                          
                          if(CONV_WAITING == true)
                          {
                            CONV_WAITING = false;
                          }
                          
                          retries = 0;
			}
                        
                        //Unsuccessful Pickup
			if(msg.id == NACK_CHK_CONV_PICKUP && STATE == CHK_CONV_PICKUP)
			{ 
				if(retries == 3)
				{
                                  STATE = EM_STOP;//In the event of the retries exceeding 3 the Robot enters an error state
                                  msg.id = ERR_ROB2;//Assigns the message ERR_ROB1 to the can message ID
                                  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
				}
				else{
                                   //Pick up from conveyor
                                  pickUpConv();
                                  
				  OSTimeDly(250);
				  
                                  msg.id = CHK_CONV_PICKUP;//Assigns the message CHK_PAD1_PICKUP to the can message ID
				  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
				  
				  retries++;//Increments the number of retries
				  
				  STATE = CHK_CONV_PICKUP;//Robot is in the state of 'check the block has been picked up successfully'
				  
				  OSTimeDly(500);//System time delay in milliseconds
				}
			}

                        //DONT drop
                        if(msg.id == NACK_DROP_PAD2)
                        {
                          block_in_transit = true;
                        }
			
			//Acknowledgment of PAD 2, ready for drop off
			if(msg.id == ACK_DROP_PAD2 && STATE == REQ_DROP_PAD2) 
			{
			  //dropBlockConveyor();//Helper function to drop the block onto the conveyor
                          //Drops to PAD2
                          coordinate.elbow = elbow_100000;
                          robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
                          OSTimeDly(500);
                          
                          coordinate.hand = hand_45000;
                          robotMoveJointTo(ROBOT_HAND, coordinate.hand);
                          OSTimeDly(500);
                          
                          coordinate.elbow = elbow_82000;
                          robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
                          OSTimeDly(500);
                          
			  msg.id = CHK_PAD2_DROP;//Assigns the message CHK_CONV_DROP to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
			  STATE = CHK_PAD2_DROP;//Robot believes the block is dropped and polls the conveyor
			}
                         //Resends drop request if nothing has been received
                        if(STATE == REQ_DROP_PAD2)
                        {
                          msg.id = REQ_DROP_PAD2;//Assigns the message REQ_DROP_CONV to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
                        }
                        
                        if(msg.id == NACK_CHK_PAD2_DROP)
                        {
                          drop_success = false;
                          msg.id = ERR_ROB2;//Assigns the message ERR_ROB1 to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
                        }
			
			//Acknowledgment of conveyor that block drop-off was successful
			if(msg.id == ACK_CHK_PAD2_DROP && STATE == CHK_PAD2_DROP)
			{

                          //robot start position
                          robotStartPos();
			  STATE = START;//Resets the robot to it's starting state
                          if(CONV_WAITING == true)
                          {
                            STATE = START;
                          }
			}
                        
                        //Resend if conveyor doesn't respond to check pick up
                        if(STATE == CHK_CONV_PICKUP)
                        {
                          msg.id = CHK_CONV_PICKUP;//Assigns the message CHK_CONV_DROP to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
                        }
                        
                        //Resend if no reply from Pad2
                        if(STATE == CHK_PAD2_DROP)
                        {
                          msg.id = CHK_PAD2_DROP;//Assigns the message CHK_CONV_DROP to the can message ID
			  canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
                        }
		}//End of non-paused state code
                
		//If controller sends a resume message
		if(msg.id == RESUME && STATE == PAUSE)
		{
			STATE = LASTSTATE;
			OSTimeDly(20);
			msg.id = RESUME_ACK_ROB2;//Assigns the message RESUME_ACK_ROB1 to the can message ID
			canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can
		}//End of paused state code
		
	}//End of non emergency stop state code
        
        if(msg.id == EM_STOP)
        {
            STATE = EM_STOP;//Set state to reset state
            msg.id = EM_STOP_ACK_ROB2;//Assigns the message RESET_ACK_ROB1 to the can message ID
            canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
            OSTimeDly(1000);
        }

	//If the controller sends the reset signal, go back to starting position.
	if(msg.id == RESET)
	{
		STATE = RESET_ACK_ROB2;//Set state to reset state
		
		msg.id = RESET_ACK_ROB2;//Assigns the message RESET_ACK_ROB1 to the can message ID
		canWrite(CAN_PORT_1, &msg);//Writes the can message with new ID to can 
		
                //robot start position
                robotStartPos();
                //STATE = START;
	}//End of emergency stop state code

       /* interfaceLedToggle(D1_LED);
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
        lcdWrite("CONV_W : %08d", conv_waiting);
        lcdSetTextPos(2,8);
        lcdWrite("BLOCK_IN_TR : %08d", block_in_transit);
        
        error = OSSemPost(LCDsem);*/
        
        //Debug code displayed on LCD
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
        lcdSetTextPos(2,6);
        lcdWrite("CONV_WAITING : %d", CONV_WAITING);
        lcdSetTextPos(2,7);
        lcdWrite(displayMessageContents[STATE]);
    
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

void pickUpConv(void){
    robotCoordinates_t coordinate;
  
    //Pick up from conveyor
    coordinate.elbow = elbow_83500;
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    OSTimeDly(500);
    
    coordinate.waist = waist_45000;
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    OSTimeDly(500);
    
    coordinate.hand = hand_45000;
    robotMoveJointTo(ROBOT_HAND, coordinate.hand);
    OSTimeDly(500);
    
    coordinate.elbow = elbow_100000;
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    OSTimeDly(500);
    
    coordinate.wrist = wrist_90000;
    robotMoveJointTo(ROBOT_WRIST, coordinate.wrist);
    OSTimeDly(500);
    
    coordinate.hand = hand_69000;
    robotMoveJointTo(ROBOT_HAND, coordinate.hand);
    OSTimeDly(500);
    
    coordinate.elbow = elbow_83500;
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    OSTimeDly(500);
}

void robotStartPos(void){ 
   robotCoordinates_t coordinate;
   
   //robot start position
    coordinate.hand = hand_87500;
    robotMoveJointTo(ROBOT_HAND, coordinate.hand); 
    OSTimeDly(500);
    
    coordinate.elbow = elbow_87500;
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow); 
    OSTimeDly(500);
    
    /*coordinate.waist = waist_67250;
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    OSTimeDly(500);
    
    coordinate.waist = waist_67250;
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    OSTimeDly(500);*/
    
    coordinate.waist = waist_68750;
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    OSTimeDly(500);
}

void robotMoveJointTo(robotJoint_t joint, uint32_t newPos) {
  
  uint32_t targetPos = newPos;
  robotJointStep_t direction;
  // if statement for emergency stop
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