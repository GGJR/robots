/*****************************************************************************
* 
* main.c
*
* ANSI C source code for OUTPUT ROBOT of the production cell project.
* uC/OS-II RTOS utilized for an event-triggered and multi-tasking functionality.
*
* Overview:
*
* Communicating mainly with both Controller and Conveyor (send and receive
* CAN messages) for safely picking-up a work-piece from the Conveyor and unload 
* it to the far end Pad 2 of the production cell. Board's LCD and LEDs have
* been utilized for reporting system status. For easy replacement of robot's 
* position coordinates, robotCoordinates data structure have been constructed.
*
* Software Architecture:
*
* 1. Waits for a request to pick-up a work-piece from the Conveyor.
* 2. Moves the work-piece above the Conveyor asking whether pick-up
* operation was successful.
* 3. Moves the work-piece above the unload Pad 2 asking Controller
* for a permission to deposit.
* 4. Deposits the work-piece to the unload Pad 2.
* 5. Returns to its starting position, waiting for the next pick-up
* request from the Conveyor.
*    
* Error Handling: 
*
* 1. If Output Robot fails to pick-up a work-piece from the Conveyor it retries
* up to 3 times before it sends an error to the Controller, leading to 
* emergency stop of the system.
* 2. If Output Robot fails to drop-off a work-piece to the Pad 2 it sends an error to
* the Controller, leading to emergency stop of the system. 
* 
* Performance Analysis:
*
* 1. Time elapsed of picking-up work-piece from the Conveyor.
* 2. Time elapsed of moving work-piece above pad2.
* 3. Time elpased of dropping work-piece to the pad2.
* 4. Time elpased of sending a CAN message (canWitre). START_ACK_ROB2 ( to the Controller) and ACK_PICKUP_CONV (to the Conveyor) messages.
* 5. Time elpased of receiving a CAN mesasge (canRead through canHandler).
* 6. Time elpased of displaying messages on the LCD monitor.
*
* @Date: 26 March 2018
* @Author: Giorgos Tsapparellas
* @Group ID: 4
* @Version: 1.0 (Complete) 
*                
*****************************************************************************/

/*****************************************************************************
*                       	INCLUDE FILES
*****************************************************************************/

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
// Containing robot coordinates data structure for easy replacement.
#include <robot.h> 
// Containing CAN IDs, CAN messages, System State IDs and System State messages
// shared with Input Robot, Controller and Conveyor components of the 
// production cell.
#include <messageTable.h> 
#include <timers.h>

/******************************************************************************
*                 DATA STRUCTURES OF PRODUCTION CELL SYSTEM
******************************************************************************/

// Data structure robotCoordinates holding uint32_t elbow, wrist, waist 
// and hand values.  
typedef struct robotCoordinates{
	
	uint32_t elbow;
	uint32_t wrist;
	uint32_t waist;
	uint32_t hand;
	
}robotCoordinates_t;

/******************************************************************************
*                 DEFINITIONS OF PRODUCTION CELL SYSTEM
******************************************************************************/

// Definitions for elbow's position coordinates (from lower to higher order)(^).
#define elbow_82000  82000
#define elbow_83500  83500
#define elbow_87500  87500
#define elbow_100000 100000

// Definition for wrist's position coordinate.
#define wrist_90000  90000

// Definitions for waist's position coordinates (from lower to higher order) (^).
#define waist_45000  45000
#define waist_67250  67250
#define waist_68750  68750
#define waist_82250  82250
#define waist_84750  84750

// Definitions for hand's position coordinates (from lower to higher order) (^).
#define hand_45000  45000
#define hand_69000  69000
#define hand_87500  87500

// Number of joints definitions used by Buttons task.
#define N_JOINTS 4	
// Maximum Pick-up Retries from Conveyor before sending an error message to the Controller. 
#define MAX_PICKUP_RETRIES 3

/*********************************************************************************
*          APPLICATION TASK PRIORITIES OF PRODUCTION CELL SYSTEM
**********************************************************************************/

// appTaskCanSendReceive task is running at higher priority,
// while appTaskCanMonitor task is the mid priority task and 
// appTaskButtons is the lowest priority task.
enum {APP_TASK_CAN_SEND_RECEIVE_PRIO = 5,
      APP_TASK_CAN_MONITOR_PRIO,
      APP_TASK_BUTTONS_PRIO};

/*********************************************************************************
*          APPLICATION TASK STACKS OF PRODUCTION CELL SYSTEM
**********************************************************************************/

// Task Buttons stack size of 256.
// Task Can SendReceive stack size of 256.
// Task Can Monitor stack size of 256.
enum {APP_TASK_BUTTONS_STK_SIZE = 256,		
      APP_TASK_CAN_SEND_RECEIVE_STK_SIZE = 256,	
      APP_TASK_CAN_MONITOR_STK_SIZE = 256};	

// OS_STK appTaskCanSendReceiveStk variable of size 256.			
static OS_STK appTaskCanSendReceiveStk[APP_TASK_CAN_SEND_RECEIVE_STK_SIZE];
// OS_STK appTaskCanMonitorStk variable of size 256.
static OS_STK appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE];
// OS_STK appTaskButtonsStk variable of size 256. 
static OS_STK appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE];

/*********************************************************************************
*          APPLICATION FUNCTION PROTOTYPES OF PRODUCTION CELL SYSTEM
**********************************************************************************/

// void CanSendReceive application function prototype.
static void appTaskCanSendReceive(void *pdata);
// void CanMonitor application function prototype.
static void appTaskCanMonitor(void *pdata);
// void TaskButtons application function prototype.
static void appTaskButtons(void *pdata);

/**********************************************************************************
*          LOCAL FUNCTION PROTOTYPES OF PRODUCTION CELL SYSTEM
**********************************************************************************/

// void canHandler local function prototype for interrupt handling for 
// CAN message reception.
static void canHandler(void);
// void robotMoveJointTo local function prototype for moving 
// desired robot joint (wrist, waist, elbow or hand) to the set coordinate. 
void robotMoveJointTo(robotJoint_t, uint32_t);
// void robotStartPos local function prototype for reseting robot to the 
// initial position (robot coordinates).
void robotStartPos(void);
// void pickUpConv local function prototype for picking-up the work-piece from 
// the Conveyor (robot coordinates). 
void pickUpConv(void);
// void moveAbovePad2 local function prototype for moving the work-piece 
// above the Pad2 (robot coordinates).
void moveAbovePad2(void);
// void dropPad2 local function prototype for dropping-off the work-piece to 
// the Pad 2 (robot coordinates).
void dropPad2(void);

/*****************************************************************************
*          GLOBAL VARIABLES OF PRODUCTION CELL SYSTEM
*****************************************************************************/

// CAN Receiver Semaphore.  
static OS_EVENT *can1RxSemaphore; 
// LCD Semaphore. 
static OS_EVENT *LCDSemaphore;
// CAN message receive buffer for CAN port 1.
static canMessage_t can1RxBuffer;
// error variable used by canHandler. 
INT8U error; 
// System state ID. 
static uint32_t systemStateID;
// Current state of the system. 
static uint32_t state;
// Last saved state of the system.
static uint32_t lastState;
// Failure retries for monitoring work-piece pick-up from the Conveyor.
static uint32_t retries = 1;
// Conveyor waiting for picking-up a work-piece (TRUE or FALSE).
static bool conveyor_waiting;

// Performance analysis of Output Robot operation.
// 1. Time elapsed of picking-up work-piece from the Conveyor.
// 2. Time elapsed of moving work-piece above pad2.
// 3. Time elpased of dropping work-piece to the pad2.
// 4. Time elpased of sending a CAN message (canWitre). START_ACK_ROB2 (Send to Controller) and ACK_PICKUP_CONV (Send to Conveyor) messages.
// 5. Time elpased of receiving a CAN mesasge (canRead through canHandler).
// 6. Time elpased of displaying debugging messages on the LCD monitor.
static uint32_t timeElapsedPickUpConv;
static uint32_t timeElapsedMoveAbovePad2;
static uint32_t timeElapsedDropPad2;
static uint32_t timeElapsedCanWriteCtrl;
static uint32_t timeElapsedCanWriteConv;
static uint32_t timeElapsedCanRead;
static uint32_t timeElapsedLCDMonitor;

/****************************************************************************
*          GLOBAL FUNCTION DEFINITIONS OF PRODUCTION CELL SYSTEM
*****************************************************************************/

/* Main function of the system. */
int main() {

	// Initialise the board support package.
	bspInit();
	// Initialise the robot support package.
	robotInit();  
	//Initialise the OS.
	OSInit();                                                   

	// Create CanSendReceive task.
	OSTaskCreate(appTaskCanSendReceive,                               
			   (void *)0,
			   (OS_STK *)&appTaskCanSendReceiveStk[APP_TASK_CAN_SEND_RECEIVE_STK_SIZE - 1],
			   APP_TASK_CAN_SEND_RECEIVE_PRIO);
	// Create CanMonitor task.
	OSTaskCreate(appTaskCanMonitor,                               
			   (void *)0,
			   (OS_STK *)&appTaskCanMonitorStk[APP_TASK_CAN_MONITOR_STK_SIZE - 1],
			   APP_TASK_CAN_MONITOR_PRIO);
    // Create Button task.
	OSTaskCreate(appTaskButtons,                
			   (void *)0, 
			   (OS_STK *)&appTaskButtonsStk[APP_TASK_BUTTONS_STK_SIZE - 1],
			   APP_TASK_BUTTONS_PRIO);

	// Initialise can1RxSemaphore.
	can1RxSemaphore = OSSemCreate(0);
	// Initialise LCDSemaphore.
	LCDSemaphore = OSSemCreate(1);
	
	// Initialise the Timer Watch.
	initWatch();

	// Start the OS.
	OSStart();                                                  

	// Should never arrive here. 
	return 0;      
} // end of main () function.

/********************************************************************************
*             APPLICATION TASKS OF PRODUCTION CELL SYSTEM
********************************************************************************/

/*
 *
 * Lowest-priority task appTaskButtons.
 * Used to identify exact robot coordinates and initial test
 * of picking-up a work-piece from the Conveyor and place it on 
 * Pad 2. 
 * 
 * LEDs are also being flashed according to joystick state.
 *
 * Joystick UP    -> pick-up a work-piece from the Conveyor.
 * Joystick DOWN  -> drop a work-piece on Pad 2.
 * Joystick RIGHT -> cycle joint selection HAND -> WRIST -> ELBOW -> WAIST
 * Joystick left  -> cycle joint selection HAND <- WRIST <- ELBOW <- WAIST
 *
 */ 
static void appTaskButtons(void *pdata) {
  
  // Initialise btnState uint32_t number.
  uint32_t btnState;
  // Set uint8_t joint equals to ROBOT_HAND (default).
  static uint8_t joint = ROBOT_HAND;
  // Initialise uint32_t leds array of size 5.
  static uint32_t leds[5] = {D1_LED, D1_LED, D2_LED, D3_LED, D4_LED};
  // Set boolean joystick right pressed to false. 
  static bool jsRightPressed = false;
  // Set boolean joystick left pressed to false.
  static bool jsLeftPressed = false;
  
  // Main loop of button task.
  while (true) {
	// Read the buttons into the btnState variable.
	btnState = buttonsRead();
	// If the button state indicates a joystick right 
	// update jsRightPressed variable accordignly.
	if (isButtonPressedInState(btnState, JS_RIGHT)) {
	  jsRightPressed = true;
	}
	// If the joystick right is pressed move to the next joint.
	// Update flashing led accordignly.
	if (jsRightPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
	  jsRightPressed = false;
	  interfaceLedSetState(leds[joint], LED_OFF);
	  joint += 1;
	  if (joint > N_JOINTS) {
		joint = 1;
	  }
	}
	// If the button state indicates a joystick left
	// update jsLeftPressed variable accordignly.
	if (isButtonPressedInState(btnState, JS_LEFT)) {
	  jsLeftPressed = true;
	}
	// If the joystick left is pressed move to the previous joint.
	// Update flashing led accordignly.
	if (jsLeftPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
	  jsLeftPressed = false;
	  interfaceLedSetState(leds[joint], LED_OFF);
	  joint -= 1;
	  if (joint == 0) {
		joint = N_JOINTS;
	  }
	}
	// Set state of interface LED.
	interfaceLedSetState(leds[joint], LED_ON);
	
	// If the joystick up is pressed pick-up the work-piece from the Conveyor.
	if (isButtonPressedInState(btnState, JS_UP)) {
	  pickUpConv();
	// else if the joystick down is pressed drop the work-piece to the Pad2.  
	} else if (isButtonPressedInState(btnState, JS_DOWN)) {
      moveAbovePad2();
	  dropPad2();
	}
	// Dealy of 20 ms.
	OSTimeDly(20);
  }
} // end of appTaskButtons (void *pdata) function.

/*
 *
 * High-priority task appTaskCanSendReceive.
 * Communicating and synchronising with Controller
 * and Conveyor by either sending or receiving CAN 
 * messages. 
 * Robot is operating according to these CAN messages
 * by calling local functions such as robotStartPos(),
 * pickUpConv(), dropPad2() and canWrite().
 * LCD is updated with CAN message,CAN msg id,
 * CAN message len, CAN msg dataA, CAN msg dataB,
 * conv_waiting boolean value, current state, CAN status
 * and system state by utilizing LCDSemaphore.  
 *
 */
static void appTaskCanSendReceive(void *pdata) {
  // Local variable error used by can1RxSemaphore.
  uint8_t error;
  // Local variable msg of type canMessage_t.
  canMessage_t msg;
  // Calling CAN Handler to interrupt on message reception.
  canRxInterrupt(canHandler);
  // Start the OS ticker.
  // Must be done on the highest-priority task.
  osStartTick();
  
  // Local boolean value indicating that work-piece is currently in transit.
  static bool block_in_transit = false;
  // Local boolean value indicating that pick-up was successful.
  static bool pick_up_success = false;
  // Local boolean value indicating that drop was successful.
  static bool drop_success = false;

  // Main loop of CanSendReceive task.  
  while ( true ) {
    
    // Pend can1RxSemaphore at the start of the procedure.
    OSSemPend(can1RxSemaphore, 0, &error);
    // Acquire can1RxBuffer to the msg variable.
    msg = can1RxBuffer;
    
    /* Communications of Output Robot with the Controller. 
       All subsystems have to sync with these CAN messages at any time. 
       EM_STOP, PAUSE, CTRL_STOP, RESUME and RESET CAN messages reception. */
    
    // If Emergency Stop CAN message has been received from the Controller...
    if(msg.id == EM_STOP)
    {
	// Assign message EM_STOP_ACK_ROB2 to the CAN message ID. 
        msg.id = EM_STOP_ACK_ROB2;
	// Send EM_STOP_ACK_ROB2 acknowledgment to the Controller. 
        canWrite(CAN_PORT_1, &msg);
        // Set the current state to EM_STOP.
        state = EM_STOP;
        // Set the system state ID to SYSTEM_EMERGENCY_STOP.
        systemStateID = SYSTEM_EMERGENCY_STOP;
    }
    
    // If Pause CAN message has been received from the Controller... 
    if(msg.id == PAUSE)
    {
        // Assign message PAUSE_ACK_ROB2 to the CAN message ID. 
        msg.id = PAUSE_ACK_ROB2;
        // Send PAUSE_ACK_ROB2 acknowledgment to the Controller. 
        canWrite(CAN_PORT_1, &msg);
        // Set the current state as the last state for Resume purposes.
        lastState = state;
        // Set the current state to PAUSE. 
        state = PAUSE;
        // Set the system state ID to SYSTEM_PAUSING.
        systemStateID = SYSTEM_PAUSING;
    }
    
    // If Control Stop CAN message has been received from the Controller... 
    if(msg.id == CTRL_STOP)
    {
        // Assign message CTRL_STOP_ACK_ROB2 to the CAN message ID.  
        msg.id = CTRL_STOP_ACK_ROB2;
        // Send CTRL_STOP_ACK_ROB2 acknowledgment to the Controller. 
        canWrite(CAN_PORT_1, &msg);
        // Set the current state to CTRL_STOP.
        state = CTRL_STOP;
        // Set the system state ID to SYSTEM_STOPPING.
        systemStateID = SYSTEM_STOPPING;
    }
    
    // If Resume CAN message has been received from the Controller... 
    if(msg.id == RESUME)
    {
        // Assign message RESUME_ACK_ROB2 to the CAN message ID. 
        msg.id = RESUME_ACK_ROB2;
        // Send RESUME_ACK_ROB2 to the Controller.
        canWrite(CAN_PORT_1, &msg);
        // Set the current state as the last state for Resume purposes.
        state = lastState;
        // Set the system state ID to SYSTEM_RESUMING.
        systemStateID = SYSTEM_RESUMING;
    }
    
    // If Reset CAN message has been received from the Controller... 
    if(msg.id == RESET)
    {
        // Assign message RESET_ACK_ROB2 to the CAN message ID. 
        msg.id = RESET_ACK_ROB2;
        // Send RESET_ACK_ROB2 to the Controller.
        canWrite(CAN_PORT_1, &msg);
        // Set robot to the start position.
        robotStartPos();
        // Set the current state to RESET_ACK_ROB2.
        state = RESET_ACK_ROB2;
        // Set the system state ID to SYSTEM_RESETTING.
        systemStateID = SYSTEM_RESETTING;
    }
    
    /* Communications of Output Robot with the Controller and Conveyor under 
       Production cell's normal operation. Resends CHK_CONV_PICKUP, REQ_DROP_PAD2
       and CHK_PAD2_DROP CAN messages if did not arrive.*/
    
    // If the current state is not equal to EM_STOP...
	if(state != EM_STOP){
	  // If the current state is not equal to CTRL_STOP... 
	  if(state != CTRL_STOP){
		// If the current state is not equal to PAUSE... 
		if(state != PAUSE){
		
			// If Emergency Stop CAN message has been received from the Controller...
			if(msg.id == EM_STOP)
			{
				// Assign message EM_STOP_ACK_ROB2 to the CAN message ID. 
				msg.id = EM_STOP_ACK_ROB2;
				// Send EM_STOP_ACK_ROB2 acknowledgment to the Controller. 
				canWrite(CAN_PORT_1, &msg);
				// Set the current state to EM_STOP.
				state = EM_STOP;
				// Set the system state ID to SYSTEM_EMERGENCY_STOP.
				systemStateID = SYSTEM_EMERGENCY_STOP;
			}

			//---Start, Pause, Stop, Resume and Reset operations---//
			
			// If Start CAN message has been received from the Controller... 
			if(msg.id == START && state <= RESET_ACK_ROB2)
			{ 
				// Assign message START_ACK_ROB2 to the CAN message ID. 
				msg.id = START_ACK_ROB2;
				// Measure computation time of sending CAN message START_ACK_ROB2 to the Controller.
				timeElapsedCanWriteCtrl = 0;
				// Start the timer.
				startWatch();
				// Send START_ACK_ROB2 acknowledgment to the Controller. 
				canWrite(CAN_PORT_1, &msg);
				// Stop the timer.
                                timeElapsedCanWriteCtrl = stopWatch();
				// Set robot to the start position.
				robotStartPos();
				// Set the current state to START.
				state = START;
				// Set the system state ID to SYSTEM_STARTING.
				systemStateID = SYSTEM_STARTING;
			}
			
            // If Control Stop CAN message has been received from the Controller... 
			if(msg.id == CTRL_STOP)
			{
				// Assign message CTRL_STOP_ACK_ROB2 to the CAN message ID.  
				msg.id = CTRL_STOP_ACK_ROB2;
				// Send CTRL_STOP_ACK_ROB2 acknowledgment to the Controller. 
				canWrite(CAN_PORT_1, &msg);
				// Set the current state to CTRL_STOP.
				state = CTRL_STOP;
				// Set the system state ID to SYSTEM_STOPPING.
				systemStateID = SYSTEM_STOPPING;
			}
                        
			// If Pause CAN message has been received from the Controller... 
			if(msg.id == PAUSE)
			{
				// Assign message PAUSE_ACK_ROB2 to the CAN message ID. 
				msg.id = PAUSE_ACK_ROB2;
				// Send PAUSE_ACK_ROB2 acknowledgment to the Controller. 
				canWrite(CAN_PORT_1, &msg);
				// Set the current state as the last state for Resume purposes.
				lastState = state;
                                // Set the current state to PAUSE. 
				state = PAUSE;
				// Set the system state ID to SYSTEM_PAUSING.
				systemStateID = SYSTEM_PAUSING;
			}	
                        
			// If Resume CAN message has been received from the Controller... 
			if(msg.id == RESUME)
			{
				// Assign message RESUME_ACK_ROB2 to the CAN message ID. 
				msg.id = RESUME_ACK_ROB2;
				// Send RESUME_ACK_ROB2 to the Controller.
				canWrite(CAN_PORT_1, &msg);
				// Set the current state as the last state for Resume purposes.
				state = lastState;
				// Set the system state ID to SYSTEM_RESUMING.
				systemStateID = SYSTEM_RESUMING;
			}
                        
			// If Reset CAN message has been received from the Controller... 
			if(msg.id == RESET)
			{
				// Assign message RESET_ACK_ROB2 to the CAN message ID. 
				msg.id = RESET_ACK_ROB2;
				// Send RESET_ACK_ROB2 to the Controller.
				canWrite(CAN_PORT_1, &msg);
				// Set robot to the start position.
				robotStartPos();
				// Set the current state to RESET_ACK_ROB2.
				state = RESET_ACK_ROB2;
				// Set the system state ID to SYSTEM_RESETTING.
				systemStateID = SYSTEM_RESETTING;
			}
                        
			//---Conveyor Pick-up operation---//
			
			// If Request Pick-Up CAN message has been received from the Conveyor or if the Convetor is waiting...
			if((msg.id == REQ_PICKUP_CONV || conveyor_waiting == true) && state < RESET_ACK_ROB2)
			{               
				// Assign message ACK_PICKUP_CONV to the CAN message ID. 
				msg.id = ACK_PICKUP_CONV;
				// Measure computation time of sending CAN message ACK_PICK_UP_CONV to the Conveyor.
				timeElapsedCanWriteConv = 0;
				// Start the timer.
				startWatch();
				// Send ACK_PICKUP_CONV acknowledgment to the Conveyor. 
				canWrite(CAN_PORT_1, &msg);
				// Stop the timer.
				timeElapsedCanWriteConv = stopWatch();
				// Measure computation time of picking-up a work-piece from to the Conveyor.
				// Notice that, delays have also been included in pickUpConv() function 
				// for Robot's movements adequate functionality.
				timeElapsedPickUpConv = 0;
				// Start the timer.
				startWatch();
				// Pick-up a work-piece from the Conveyor.
				pickUpConv();
				// Stop the timer.
				timeElapsedPickUpConv = stopWatch();
				// Delay for 1000 ms.
				OSTimeDly(1000);
				// Assign message CHK_CONV_PICKUP to the CAN message ID. 
				msg.id = CHK_CONV_PICKUP;
				// Send CHK_CONV_PICKUP to the Conveyor.  
				canWrite(CAN_PORT_1, &msg);           
				// Set the current state to CHK_CONV_PICKUP.
				state = CHK_CONV_PICKUP;
				// Set the system state ID to SYSTEM_RUNNING.
				systemStateID = SYSTEM_RUNNING;
			}
			
                        // If Request Pick-Up CAN message has been received from the Conveyor and work-piece is currently in transit...            
			if(msg.id == REQ_PICKUP_CONV && state > REQ_PICKUP_CONV)
			{
				// Assign message ACK_PICKUP_CONV to the CAN message ID. 
				msg.id = ACK_PICKUP_CONV;
				// Send ACK_PICKUP_CONV acknowledgment to the Conveyor. 
				canWrite(CAN_PORT_1, &msg);
				// Set boolean block_in_transit value to true.
				block_in_transit = true;  
				// Set boolean conveyor_waiting value to true.
				conveyor_waiting = true;
			}
                          
			// If Acknowledgment Check Conveyor Pick-Up CAN message has been received from the Conveyor... 
			// Successful Pick-Up.
			if(msg.id == ACK_CHK_CONV_PICKUP && state == CHK_CONV_PICKUP) 
			{ 
				// Measure computation time of moving a work-piece above the Pad2.
				// Notice that, delays have also been included in moveAbovePad2() function 
				// for Robot's movements adequate functionality.
                                timeElapsedMoveAbovePad2 = 0;
				// Start the timer.
                                startWatch();
				// Move the work-piece above the Pad2 ready for drop-off.
				moveAbovePad2();
				// Stop the timer.
                                timeElapsedMoveAbovePad2 = stopWatch();
				// Assign message REQ_DROP_PAD2 to the CAN message ID. 
				msg.id = REQ_DROP_PAD2;
				// Send REQ_DROP_PAD2 to the Controller. 
				canWrite(CAN_PORT_1, &msg);
				// Set the current state to REQ_DROP_PAD2. 
				state = REQ_DROP_PAD2;
                                // If boolean conveyor_waiting value is true...
				if(conveyor_waiting == true)
				{
					// Set boolean CONV_WAITING value back to false.
					conveyor_waiting = false;
				}
				// Set retries value to 1.
				retries = 1;
				// Set boolean pick_up_success value to true.
				pick_up_success = true;
			}
                        
			// If Negative Acknowledgment Check Conveyor Pick-Up CAN message has been received from the Conveyor... 
			// Unsuccessful Pick-Up.
			if(msg.id == NACK_CHK_CONV_PICKUP && state == CHK_CONV_PICKUP)
			{ 
				// Set boolean pick_up_success value to false.
				pick_up_success = false;
				// If retries reach the maximum number which is 3...
				if(retries == MAX_PICKUP_RETRIES)
				{
					// Assign message ERR_ROB2 to the CAN message ID. 
					msg.id = ERR_ROB2;
					// Send ERR_ROB2 to the Controller. 
					canWrite(CAN_PORT_1, &msg);
					// Set the current state to EM_STOP.  
					state = EM_STOP;
					// Set the system state ID to SYSTEM_EMERGENCY_STOP.
					systemStateID = SYSTEM_EMERGENCY_STOP;
				}
				else // Else...
				{
					// Pick-up work-piece from the Conveyor.
					pickUpConv();   
					// Delay of 250 ms.
                                        OSTimeDly(250);
					// Assign message CHK_CONV_PICKUP to the CAN message ID. 
                                        msg.id = CHK_CONV_PICKUP;
					// Send CHK_CONV_PICKUP to the Conveyor. 
                                        canWrite(CAN_PORT_1, &msg);
					// Increment the number of retries by 1.
                                        retries++;
					// Set the current state to CHK_CONV_PICKUP. 
                                        state = CHK_CONV_PICKUP;
					// Delay of 500 ms. 
                                        OSTimeDly(500);
				}
			}
			
			// If current state is CHK_CONV_PICKUP...
			// Resend Check Conveyor Pickup CAN message if nothing has been received.
			if(state == CHK_CONV_PICKUP)
			{
				// Assign message CHK_CONV_PICKUP to the CAN message ID.
				msg.id = CHK_CONV_PICKUP;
				// Send CHK_CONV_PICKUP to the Conveyor. 
				canWrite(CAN_PORT_1, &msg);
			}

			// If Negative Drop Pad 2 Permission CAN message has been received from the Controller... 
			// DON'T DROP.
			// Waiting for the Pad2 to become free.
			if(msg.id == NACK_DROP_PAD2)
			{
				// Set boolean block_in_transit value to true.
				block_in_transit = true;
			}
			
			// If Acknowledgment Drop Pad 2 Permission CAN message has been received from the Controller... 
			// DO DROP.
			if(msg.id == ACK_DROP_PAD2 && state == REQ_DROP_PAD2) 
			{
				// Measure computation time of dropping-off a work-piece to the Pad2.
				// Notice that, delays have also been included in dropPad2() function 
				// for Robot's movements adequate functionality.
                                timeElapsedDropPad2 = 0;
				// Start the timer.
				startWatch();
                                // Drop work-piece to Pad2.
				dropPad2();        
				// Stop the timer.
                                timeElapsedDropPad2 = stopWatch();
				// Assign message CHK_PAD2_DROP to the CAN message ID. 
				msg.id = CHK_PAD2_DROP;
				// Send CHK_PAD2_DROP to the Controller.  
				canWrite(CAN_PORT_1, &msg);
				// Set the current state to CHK_PAD2_DROP.
				state = CHK_PAD2_DROP;
			}
			
			// If current state is REQ_DROP_PAD2...
			// Resend Request Drop Pad2 CAN message if nothing has been received.
			if(state == REQ_DROP_PAD2)
			{
				// Assign message REQ_DROP_PAD2 to the CAN message ID. 
				msg.id = REQ_DROP_PAD2;
				// Send REQ_DROP_PAD2 to the Controller. 
				canWrite(CAN_PORT_1, &msg); 
			}
                        
			// If Negative Check Drop Pad 2 CAN message has been received from the Controller... 
			if(msg.id == NACK_CHK_PAD2_DROP)
			{
				// Set boolean drop_success value to false.
				drop_success = false;
				// Assign message ERR_ROB2 to the CAN message ID. 
				msg.id = ERR_ROB2;
				// Send ERR_ROB2 to the Controller.
				canWrite(CAN_PORT_1, &msg);
			}
			
			// If Acknowledgment Check Drop Pad 2 CAN message has been received from the Controller... 
			if(msg.id == ACK_CHK_PAD2_DROP && state == CHK_PAD2_DROP)
			{
				// Set boolean drop_success value to true.
				drop_success = true;
				// Set robot to the start position.
				robotStartPos();
				// Set the current state to START.
				state = START;
			}         
                        
			// If current state is CHK_PAD2_DROP...
			// Resend Check Pad2 Drop CAN message if nothing has been received.
			if(state == CHK_PAD2_DROP)
			{
				// Assign message CHK_PAD2_DROP to the CAN message ID.
				msg.id = CHK_PAD2_DROP;
				// Send CHK_PAD2_DROP to the Controller.
				canWrite(CAN_PORT_1, &msg);
			}
		}// end of Non-Paused if statement.
                
		// If Resume CAN message has been received from the Controller... 
		if(msg.id == RESUME && state == PAUSE)
		{
			// Assign message RESUME_ACK_ROB2 to the CAN message ID. 
			msg.id = RESUME_ACK_ROB2;
			// Send RESUME_ACK_ROB2 to the Controller.
			canWrite(CAN_PORT_1, &msg);
                        // Set the current state as the last state for Resume purposes.
			state = lastState;
			// Set the system state ID to SYSTEM_RESUMING.
			systemStateID = SYSTEM_RESUMING;
		}
	  } // end of Ctrl stop if statement.
	}// end of Non-Emergency stop if statement.
  } // end of while loop of appTaskCanSendReceive () function.
} // end of appTaskCanSendReceive () function.

/*
 *
 * Mid-priority task appTaskMonitor.
 * Updating CAN status on LCD by pending 
 * and posting LCDSemaphore, sequentially.
 *
 */
static void appTaskCanMonitor(void *pdata) {

  // Local variable error used by LCDSemaphore.
  uint8_t error;
  // Local variable msg of type canMessage_t.
  canMessage_t msg;
  
  // Main loop of Can Monitor task.
  while (true) {
    // Measure computation time of time taken to display debugging
    // messages on the LCD monitor of LPC2378-STK evaluation board.
    timeElapsedLCDMonitor = 0;
    // Pend LCDSemaphore.
    OSSemPend(LCDSemaphore, 0, &error);
    // Start the timer.
    startWatch();
    // Acquire can1RxBuffer to the msg variable for CAN related debugging messages.
    msg = can1RxBuffer;
    // Toggle the interface LED OF LPC2378-STK evaluation board.
    interfaceLedToggle(D1_LED);
    
    // For debugging purposes LCD is updated with CAN message,CAN msg id,
    // CAN message len, CAN msg dataA, CAN msg dataB,
    // conv_waiting boolean value, current state, CAN status and
    // system state by utilizing LCDSemaphore.  
	
    // Set LCD text position to 2,1.
   /* lcdSetTextPos(2,1);
    // Display CAN message.
    lcdWrite(displayMessageContents[msg.id]); 
    // Set LCD text position to 2,2.
    lcdSetTextPos(2,2);
    // Display CAN msg id.
    lcdWrite("msg.id : %08d", msg.id);
    // Set LCD text position to 2,3.	
    lcdSetTextPos(2,3);
    // Display CAN msg length.
    lcdWrite("LEN    : %08x", msg.len); 
    // Set LCD text position to 2,4.
    lcdSetTextPos(2,4);
    // Display CAN msg dataA.
    lcdWrite("DATA_A : %08x", msg.dataA); 
    // Set LCD text position to 2,5.
    lcdSetTextPos(2,5);
    // Display CAN msg dataB.
    lcdWrite("DATA_B : %08x", msg.dataB);
    // Set LCD text position to 2,6.
    lcdSetTextPos(2,6);
    // Display if conveyor is waiting for picking-up a work-piece (boolean value).
    lcdWrite("CONV_WAITING : %d", conveyor_waiting);
    // Set LCD text position to 2,7.
    lcdSetTextPos(2,7);
    // Display current state of the system.
    lcdWrite(displayMessageContents[state]);
    // Set LCD text position to 2,8.
    lcdSetTextPos(2,8);
    // Display CAN status.
    lcdWrite("CAN1GSR: %08x", canStatus(CAN_PORT_1));
    // Set LCD text position to 2,9.
    lcdSetTextPos(2,9);
    // Display system state id.
    lcdWrite(displaySystemState[systemStateID]);*/
    
    // Uncomment below source code for debugging 
    // computation times of picking-up a work-piece
    // from the conveyor, moving a work-piece above the
    // pad2, dropping-off a work-piece to the pad2, 
    // reading a CAN message (through canHandler), 
    // writting a CAN message (to the Controller and Conveyor),
    // and displaying messages on the LCD monitor.
    // Rememeber that, above displaying source code has to be commented 
    // for ignoring LCD monitor's confliction.
	
    
    // Set LCD text position to 2,4.
    lcdSetTextPos(2,4);
    // Display time taken to pick-up a work-piece from the Conveyor.
    lcdWrite("tPUConv: %u", timeElapsedPickUpConv);
    // Set LCD text position to 2,5.
    lcdSetTextPos(2,5);
    // Display time taken to move work-piece above the Pad2.
    lcdWrite("tMAPad2: %u", timeElapsedMoveAbovePad2);
    // Set LCD text position to 2,6.
    lcdSetTextPos(2,6);
    // Display time taken to drop-off a work-piece on Pad2.
    lcdWrite("tDPad2: %u", timeElapsedDropPad2);
    // Set LCD text position to 2,7.
    lcdSetTextPos(2,7);
    // Display time taken to read CAN message from the bus.
    lcdWrite("tCanRead: %u", timeElapsedCanRead);
    // Set LCD text position to 2,8.
    lcdSetTextPos(2,8);
    // Display time taken to send CAN message to the Controller.
    lcdWrite("tCanWriteCtrl: %u", timeElapsedCanWriteCtrl);
    // Set LCD text position to 2,9.
    lcdSetTextPos(2,9);
    // Display time taken to send CAN message to the Conveyor.
    lcdWrite("tCanWriteConv: %u", timeElapsedCanWriteConv);
    // Stop the timer.
    timeElapsedLCDMonitor = stopWatch();
    // Set LCD text position to 2,10.
    lcdSetTextPos(2,10);
    // Display time taken to write values on LCD monitor.
    lcdWrite("tLCD: %u", timeElapsedLCDMonitor);
	
    // Post the LCDSemaphore.
    error = OSSemPost(LCDSemaphore);
    // Delay of 20 ms.
    OSTimeDly(20);
  }
} // end of appTaskCanMonitor (void *pdata) function.

/********************************************************************************
*             LOCAL FUNCTIONS OF PRODUCTION CELL SYSTEM
********************************************************************************/

/*
 *
 * A simple interrupt handler for CAN message reception on CAN1.
 * Use of can1RxSemaphore to synchronise between the CAN interrupt
 * handler and the appTaskCanSendReceive task. 
 *
 */
static void canHandler(void) {
  // If CAN port 1 is ready...
  if (canReady(CAN_PORT_1)) {
    // Measure computation time of reading a CAN message from the
    // bus using can interrupt handler.
    timeElapsedCanRead = 0;
    // Start the timer.
    startWatch();
    // Read CAN port 1 reception buffer.
    canRead(CAN_PORT_1, &can1RxBuffer);
    // Stop the timer.
    timeElapsedCanRead = stopWatch();
    // Post can1RxSemaphore semaphore.
    error = OSSemPost(can1RxSemaphore);
  }
} // end of canHandler () function. 

/*
 *
 * Guide the Output Robot to start position.
 * For easy replacement of position coordinates, 
 * robotCoordinates data structure have been constructed.
 * robotMoveJointTo function is being called,
 * specifying joint to be moved as well as the new coordinate
 * position. Delay between each robot movement have been
 * considered for adequate functionality. 
 *
 */
void robotStartPos(void){ 
   // Local variable coordinate of type robotCoordinates_t. 
   robotCoordinates_t coordinate;
   
   // Set hand variable of coordinate data structure to 87500.
   coordinate.hand = hand_87500;
   // Move ROBOT_HAND joint to specified coordinate.
   robotMoveJointTo(ROBOT_HAND, coordinate.hand); 
   // Delay of 500 ms.
   OSTimeDly(500);
   
   // Set elbow variable of coordinate data structure to 87500.
   coordinate.elbow = elbow_87500;
   // Move ROBOT_ELBOW joint to specified coordinate.
   robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow); 
   // Delay of 500 ms.
   OSTimeDly(500);
    
   // Set waist variable of coordinate data structure to 68750.
   coordinate.waist = waist_68750;
   // Move ROBOT_WAIST to specified coordinate.
   robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
   // Delay of 500 ms.
   OSTimeDly(500);
} // end of robotStartPos () function.

/*
 *
 * Guide the Output Robot to pick-up work-piece from the Conveyor.
 * For easy replacement of position coordinates, 
 * robotCoordinates data structure have been constructed.
 * robotMoveJointTo function is being called,
 * specifying joint to be moved as well as the new coordinate
 * position. Delay between each robot movement have been
 * considered for adequate functionality. 
 *
 */
void pickUpConv(void){
    // Local variable coordinate of type robotCoordinates_t. 
    robotCoordinates_t coordinate;
  
    // Set elbow variable of coordinate data structure to 83500.
    coordinate.elbow = elbow_83500;
    // Move ROBOT_ELBOW joint to specified coordinate.
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set waist variable of coordinate data structure to 83500. 
    coordinate.waist = waist_45000;
    // Move ROBOT_WAIST joint to specified coordinate.
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    // Delay  of 500 ms.
    OSTimeDly(500);
    
    // Set hand variable of coordinate data structure to 45000.
    coordinate.hand = hand_45000;
    // Move ROBOT_HAND joint to specified coordinate.
    robotMoveJointTo(ROBOT_HAND, coordinate.hand);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set elbow variable of coordinate data structure to 100000.
    coordinate.elbow = elbow_100000;
    // Move ROBOT_ELBOW joint to specified coordinate.
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set wrist variable of coordinate data structure to 90000.
    coordinate.wrist = wrist_90000;
    // Move ROBOT_WRIST joint to specified coordinate.
    robotMoveJointTo(ROBOT_WRIST, coordinate.wrist);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set hand variable of coordinate data structure to 69000.
    coordinate.hand = hand_69000;
    // Move ROBOT_HAND joint to specified coordinate.
    robotMoveJointTo(ROBOT_HAND, coordinate.hand);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set elbow variable of coordinate data structure to 83500.
    coordinate.elbow = elbow_83500;
    // Move ROBOT_ELBOW joint to specified coordinate.
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    // Delay of 500 ms.
    OSTimeDly(500);
} // end of pickUpConv () function.

/*
 *
 * Guide the Output Robot to move the work-piece above the Conveyor.
 * For easy replacement of position coordinates, 
 * robotCoordinates data structure have been constructed.
 * robotMoveJointTo function is being called,
 * specifying joint to be moved as well as the new coordinate
 * position. Delay between each robot movement have been
 * considered for adequate functionality. 
 *
 */
void moveAbovePad2(void){
    // Local variable coordinate of type robotCoordinates_t. 
    robotCoordinates_t coordinate;
    // Set waist variable of coordinate data structure to 84750.
    coordinate.waist = waist_84750;
    // Move ROBOT_WAIST joint to specified coordinate.
    robotMoveJointTo(ROBOT_WAIST, coordinate.waist);
    // Delay of 500 ms.
    OSTimeDly(500);
    
    // Set wrist variable of coordinate data structure to 90000.
    coordinate.wrist = wrist_90000;
    // Move ROBOT_WRIST joint to specified coordinate.
    robotMoveJointTo(ROBOT_WRIST, coordinate.wrist);
    // Delay of 500 ms.
    OSTimeDly(500);	
} // end of moveAbove Conveyor () function. 

/*
 *
 * Guide the Output Robot to drop a work-piece to the Pad2.
 * For easy replacement of position coordinates, 
 * robotCoordinates data structure have been constructed.
 * robotMoveJointTo function is being called,
 * specifying joint to be moved as well as the new coordinate
 * position. Delay between each robot movement have been
 * considered for adequate functionality. 
 *
 */
void dropPad2(void){
    // Local variable coordinate of type robotCoordinates_t. 
    robotCoordinates_t coordinate;
  
    // Set elbow variable of coordinate data structure to 100000.
    coordinate.elbow = elbow_100000;
    // Move ROBOT_ELBOW joint to specified coordinate.
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    // Delay of 500 ms.
    OSTimeDly(500);
	
    // Set hand variable of coordinate data structure to 45000. 	
    coordinate.hand = hand_45000;
    // Move ROBOT_HAND joint to specified coordinate.
    robotMoveJointTo(ROBOT_HAND, coordinate.hand);
    // Delay of 500 ms.
    OSTimeDly(500);
	
    // Set elbow variable of coordinate data structure to 82000.   
    coordinate.elbow = elbow_82000;
    // Move ROBOT_ELBOW joint to specified coordinate.
    robotMoveJointTo(ROBOT_ELBOW, coordinate.elbow);
    // Delay of 500 ms.
    OSTimeDly(500);
} // end of dropPad2 () function.

/*
 *
 * Guide the Output Robot specified joint to a new 
 * coordinate position.
 * @parameters: 
 *		robotJoint_t joint   -> Ouptut Robot joint. 
 *		uint32_t newPosition -> New coordinate position.
 *
 */
void robotMoveJointTo(robotJoint_t joint, uint32_t newPosition) {
  
  // Assign new position to the target postion uint32_t variable.
  uint32_t targetPosition = newPosition;
  // Local variable direction of type robotJointStep_t.
  robotJointStep_t direction;
  
  // If the system is not in emergency stop, stopping or pausing state...
  if (systemStateID != SYSTEM_EMERGENCY_STOP || systemStateID != SYSTEM_STOPPING || systemStateID != SYSTEM_PAUSING)
  {
    // If the current state of inputed joint is greater
    // than the target coordinate position...
    if(robotJointGetState(joint) > targetPosition) {
        // Decrease the joint coordinate position into direction variable.
        direction = ROBOT_JOINT_POS_DEC;
    }
    // Else...
    else { 
        // Increase the joint coordinate position into direction variable. 
        direction = ROBOT_JOINT_POS_INC;
    }
    // While current state of inputed joint is not equal 
    // to the target posistion, set the robot joint state
    // according to specified joint and direction.  
    while(robotJointGetState(joint) != targetPosition) {
        robotJointSetState(joint, direction);
        // Delay of 10 ms.
        OSTimeDly(10);
    }
  }
}; // end of robotMoveJointTo (robotJoint_t joint, uint32_t newPos) function.