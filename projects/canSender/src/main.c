/* Control Panel (built upon Control Panel Test)
 * Robert Small w13007276 (2018)
 *
 * Built for assignment EN0617 Professionalism & Industrial Case Project - Computer Science - Northumbria University
 */

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <buttons.h>
#include <interface.h>
#include <control.h>
#include <messageTable.h>
#include <can.h>
#include <lcd.h>

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_CAN_RECEIVE_PRIO = 4,
  APP_TASK_CAN_SEND_PRIO,
  APP_TASK_CTRL_PRIO,
  APP_TASK_LCD_PRIO,
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_CAN_RECEIVE_STK_SIZE = 256,
  APP_TASK_CAN_SEND_STK_SIZE = 256,
  APP_TASK_CTRL_STK_SIZE = 256,
  APP_TASK_LCD_STK_SIZE = 256,
};

static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];
static OS_STK appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE];
static OS_STK appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE];
static OS_STK appTaskLCDStk[APP_TASK_LCD_STK_SIZE];


/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskCanReceive(void *pdata);
static void appTaskCanSend(void *pdata);
static void appTaskCtrl(void *pdata);
static void appTaskLCD(void *pdata);

/*
*************************************************************************
*                 LOCAL FUNCTION PROTOTYPES
*************************************************************************
*/

static void sendMessage(int message);
static void canHandler(void);
 
/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

static OS_EVENT *can1RxSem;
static OS_EVENT *canSendSem;
static canMessage_t can1RxBuf;
static int messageDisplay = -1;
static int messageSend = -1;
static int selectedMessage = 0;

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
  OSTaskCreate(appTaskCanReceive,
               (void *)0,
               (OS_STK *)&appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE - 1],
               APP_TASK_CAN_RECEIVE_PRIO);
  
  OSTaskCreate(appTaskCanSend,
               (void *)0,
               (OS_STK *)&appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE - 1],
               APP_TASK_CAN_SEND_PRIO);
   
  OSTaskCreate(appTaskCtrl,
               (void *)0,
               (OS_STK *)&appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE - 1],
               APP_TASK_CTRL_PRIO);
  
  OSTaskCreate(appTaskLCD,
               (void *)0,
               (OS_STK *)&appTaskLCDStk[APP_TASK_LCD_STK_SIZE - 1],
               APP_TASK_LCD_PRIO);
  
  /* Create Semaphores and Mutexes */
  can1RxSem = OSSemCreate(0);
  canSendSem = OSSemCreate(0);
   
  /* Start the OS */
  OSStart();
  
  /* Should never arrive here */ 
  return 0;
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

/*
 * Event driven task that receives messages on CAN1.
 * This task calls the function processRecievedMessage(int message) to correctly process a message.
 */
static void appTaskCanReceive(void *pdata) {  
  // Install the CAN interrupt handler and start the OS ticker (must be done in the highest priority task)
  canRxInterrupt(canHandler);
  osStartTick();
  
  // Initialise local task variables.
  uint8_t error;
  canMessage_t msg;
  
  // Main loop for this task.
  while ( true ) {
    OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    
    // Updates the message to display on LCD (for info)
    messageDisplay = msg.id;
  }
}

/*
 * Event driven task that sends messages on CAN1.
 * The function sendMessage(int message) should be called to begin the sending process.
 */
static void appTaskCanSend(void *pdata) {
  // Initialise local task variables.
  canMessage_t msg = {0, 0, 0, 0};
  uint8_t error;

  // Initialise the CAN message structure
  msg.id = 0x07;
  msg.len = 4;
  msg.dataA = 0;
  msg.dataB = 0;
  
  // Main loop for this task.
  // Event driven task to send messages on CAN1.
  while ( true ) {
    // Wait for semaphore.
    OSSemPend(canSendSem, 0, &error);
    
    // Set message ID to new value.
    msg.id = messageSend;
    
    // Transmit message on CAN 1
    canWrite(CAN_PORT_1, &msg);
  }
}

/*
 * Task that handles the control of the system.
 * Button input is handled here.
 */
static void appTaskCtrl(void *pdata) {
  uint32_t btnState;
  bool JoyLeft = false;
  bool JoyRight = false;
  bool JoyMiddle = false;
  
  // Main loop for this task.
  while (true) {
    // Read the button state.
    btnState = buttonsRead();
    
    // Handles joystick left.
    if (isButtonPressedInState(btnState, JS_LEFT)) {
      JoyLeft = true;
    }
    if (JoyLeft && (!isButtonPressedInState(btnState, JS_LEFT))) {
      // Joystick up has been pressed and released.
      JoyLeft = false;
      selectedMessage = (selectedMessage - 1) % 50;
    }
    
    // Handles joystick right.
    if (isButtonPressedInState(btnState, JS_RIGHT)) {
      JoyRight = true;
    }
    if (JoyRight && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      // Joystick up has been pressed and released.
      JoyRight = false;
      selectedMessage = (selectedMessage + 1) % 50;
    }
    
    // Handles joystick middle.
    if (isButtonPressedInState(btnState, JS_CENTRE)) {
      JoyMiddle = true;
    }
    if (JoyMiddle && (!isButtonPressedInState(btnState, JS_CENTRE))) {
      // Joystick up has been pressed and released.
      JoyMiddle = false;
      sendMessage(selectedMessage);
    }
    
    // Loop delay.
    OSTimeDly(50);
  }
}

/*
 * Task to handle all LCD output.
 * No semaphores used, this task is the only task interacting with the LCD screen.
 */
static void appTaskLCD(void *pdata) {
  // Main loop for this task.
  // LCD display loop. Updates LCD screen to show the current system state.
  while ( true ) {    
    // Show title.
    lcdSetTextPos(1, 1);
    lcdWrite("Message Test Unit");
    
    // Show selected message.
    lcdSetTextPos(1, 3);
    lcdWrite("Selected:");
    lcdSetTextPos(1, 4);
    lcdWrite(displayMessageContents[selectedMessage]);
    
    // Show last message recieved.
    lcdSetTextPos(1, 6);
    lcdWrite("Last:");
    lcdSetTextPos(1, 7);
    if (messageDisplay != -1) {
      lcdWrite(displayMessageContents[messageDisplay]);
    }
    
    //Loop delay.
    OSTimeDly(100);
  }
}


/*************************************************************************
*                   APPLICATION FUNCTION DEFINITIONS
*************************************************************************/

/*
 * A simple interrupt handler for CAN message reception on CAN1
 */
static void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    OSSemPost(can1RxSem);
  }
}

/*
 * A helper function to assist sending messages on CAN.
 */
static void sendMessage(int message) {    
  // Update mesage to diaplay.
  messageDisplay = message;
  
  // Post the semephore so the sending task can take over.
  messageSend = message;
  OSSemPost(canSendSem);
}