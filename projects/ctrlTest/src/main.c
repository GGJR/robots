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
  APP_TASK_MONITOR_SENS_PRIO,
  APP_TASK_CTRL_PRIO,
  APP_TASK_CAN_RETRY_PRIO,
  APP_TASK_LED_PRIO,
  APP_TASK_LCD_PRIO,
};

/*************************************************************************
*                     APPLICATION TASK STACKS
*************************************************************************/

enum {
  APP_TASK_CAN_RECEIVE_STK_SIZE = 256,
  APP_TASK_CAN_SEND_STK_SIZE = 256,
  APP_TASK_MONITOR_SENS_STK_SIZE = 256,
  APP_TASK_CTRL_STK_SIZE = 256,
  APP_TASK_CAN_RETRY_STK_SIZE = 256,
  APP_TASK_LED_STK_SIZE = 256,
  APP_TASK_LCD_STK_SIZE = 256,
};

static OS_STK appTaskCanReceiveStk[APP_TASK_CAN_RECEIVE_STK_SIZE];
static OS_STK appTaskCanSendStk[APP_TASK_CAN_SEND_STK_SIZE];
static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];
static OS_STK appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE];
static OS_STK appTaskCanRetryStk[APP_TASK_CAN_RETRY_STK_SIZE];
static OS_STK appTaskLEDStk[APP_TASK_LED_STK_SIZE];
static OS_STK appTaskLCDStk[APP_TASK_LCD_STK_SIZE];


/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskCanReceive(void *pdata);
static void appTaskCanSend(void *pdata);
static void appTaskMonitorSens(void *pdata);
static void appTaskCtrl(void *pdata);
static void appTaskCanRetry(void *pdata);
static void appTaskLED(void *pdata);
static void appTaskLCD(void *pdata);

/*
*************************************************************************
*                 LOCAL FUNCTION PROTOTYPES
*************************************************************************
*/

static void sendMessage(int message);
static void canHandler(void);
static void processRecievedMessage(int message);
 
/*
*************************************************************************
*                 GLOBAL VARIABLE DEFINITIONS
*************************************************************************
*/

char pendingAckMessageTitle[7][21] = {
  "EM Stop:             ",
  "Pause  :             ",
  "Resume :             ",
  "Start  :             ",
  "Stop   :             ",
  "Reset  :             ",
  "Pad1Chk:             ",};
static OS_EVENT *can1RxSem;
static OS_EVENT *canSendSem;
static canMessage_t can1RxBuf;
static int messageDisplay = -1;
static int errorDetected = -1;
static int messageSend = -1;
static int systemState = 0;
static bool emStopAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool pauseAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool resumeAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool startAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool stopAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool resetAck[3] = {false, false, false}; // {conveyor, robot1, robot2}
static bool pad1BlockAck = false; // {robot1}
static bool pad2WaitingAck = false; // Stores whether robot2 is waiting for ack.
static bool responsesWaiting[7] = {false, false, false, false, false, false, false}; // {EmergencyStop, Pause, Resume, Start, Stop, Reset, Pad1Block}
static bool ignorePadInput = false;


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
  
  OSTaskCreate(appTaskMonitorSens,
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
   
  OSTaskCreate(appTaskCtrl,
               (void *)0,
               (OS_STK *)&appTaskCtrlStk[APP_TASK_CTRL_STK_SIZE - 1],
               APP_TASK_CTRL_PRIO);
  
  OSTaskCreate(appTaskCanRetry,
               (void *)0,
               (OS_STK *)&appTaskCanRetryStk[APP_TASK_CAN_RETRY_STK_SIZE - 1],
               APP_TASK_CAN_RETRY_PRIO);
  
  OSTaskCreate(appTaskLED,
               (void *)0,
               (OS_STK *)&appTaskLEDStk[APP_TASK_LED_STK_SIZE - 1],
               APP_TASK_LED_PRIO);
  
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
    
    // Process message.
    processRecievedMessage(msg.id);
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
 * Task that moniters the two sensory inputs.
 */
static void appTaskMonitorSens(void *pdata) {
  // Main loop for this task.
  while (true) {
    // Set the state of both on board LEDs to off.
    ledSetState(USB_LINK_LED, LED_OFF);
    ledSetState(USB_CONNECT_LED, LED_OFF);
    
    // Check if Sensor 1 is currently detecting a block.
    if (controlItemPresent(CONTROL_SENSOR_1)) {
      // Turn LED on.
      ledSetState(USB_LINK_LED, LED_ON);
      
      // If a message to Robot1 hasn't already been sent.
      if (!ignorePadInput && systemState == SYSTEM_RUNNING) {
        // Send message to robot1. (informing that there is block)
        sendMessage(REQ_PICKUP_PAD1);
      }
    }
    // Check if sensor 2 is currently detecting a block.
    if (controlItemPresent(CONTROL_SENSOR_2)) {
      // Turn LED on.
      ledSetState(USB_CONNECT_LED, LED_ON);
    } else if (pad2WaitingAck) {
      // Robot 2 is holding a block and pad2 has just become free.
      sendMessage(ACK_DROP_PAD2);
      pad2WaitingAck = false;
    }
    
    // Loop delay.
    OSTimeDly(20);
  }
}

/*
 * Task that handles the control of the system.
 * Button input is handled here.
 */
static void appTaskCtrl(void *pdata) {
  // Initialise local task variables.
  static bool emergency = false;
  //interfaceLedSetState(D3_LED | D4_LED, LED_OFF);
  uint32_t btnState;
  bool Btn1 = false;
  bool Btn2 = false;
  bool JoyUp = false;
  
  // Main loop for this task.
  while (true) {
    // Read the button state.
    btnState = buttonsRead();
       
    // Handles button1. (start/stop)
    if (isButtonPressedInState(btnState, BUT_1)) {
      Btn1 = true;
    }
    if (Btn1 && (!isButtonPressedInState(btnState, BUT_1))) {
      // Button1 has been pressed and released.
      Btn1 = false;
      if (systemState == SYSTEM_NOT_STARTED || systemState == SYSTEM_STOPPED) {
      //if (true) {
        sendMessage(START);
      } else if (systemState == SYSTEM_RUNNING) {
        sendMessage(CTRL_STOP);
      }
    }
    
    // Handles button2. (pause/resume)
    if (isButtonPressedInState(btnState, BUT_2)) {
      Btn2 = true;
    }
    if (Btn2 && (!isButtonPressedInState(btnState, BUT_2))) {
      // Button2 has been pressed and released.
      Btn2 = false;
      if (systemState == SYSTEM_RUNNING) {
        sendMessage(PAUSE);
      } else if (systemState == SYSTEM_PAUSED){
        sendMessage(RESUME);
      }
    }
    
    // Handles joystick up. (reset)
    if (isButtonPressedInState(btnState, JS_UP)) {
      JoyUp = true;
    }
    if (JoyUp && (!isButtonPressedInState(btnState, JS_UP))) {
      // Joystick up has been pressed and released.
      JoyUp = false;
      sendMessage(RESET);
    }
    
    // Handles Emergency Stop Button
    emergency = controlEmergencyStopButtonPressed();
    if (emergency) {
      sendMessage(EM_STOP);
      while (controlEmergencyStopButtonPressed()) {
        OSTimeDly(20);
      }
    }
    
    // Loop delay.
    OSTimeDly(50);
  }
}

/*
 * Task that retries sending messages until the expected response is recieved.
 * If the number of retries excedes the RETRY_COUNT constant, then an error is detected and the system signals emergency stop.
 */
static void appTaskCanRetry(void *pdata) {
  // Initialise local task variables.
  const int RETRY_COUNT = 50;
  bool waitingForLastTick[7] = {false, false, false, false, false, false, false};
  int retryAttempts[7] = {RETRY_COUNT, RETRY_COUNT, RETRY_COUNT, RETRY_COUNT, RETRY_COUNT, RETRY_COUNT, RETRY_COUNT};
  
  // Main loop for this task.
  // Handles resending messages if responses are not recieved.
  while (true) {
    // Check for emergency stop.
    if (responsesWaiting[0]) {
      if (waitingForLastTick[0]) {
        // If all responses recieved...
        if (emStopAck[0] && emStopAck[1] && emStopAck[2]) {
          // Reset values back to default.
          emStopAck[0] = false;
          emStopAck[1] = false;
          emStopAck[2] = false;
          responsesWaiting[0] = false;
          waitingForLastTick[0] = false;
          retryAttempts[0] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_EMERGENCY_STOP;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[0]--;
          sendMessage(EM_STOP);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[0] = true;
      }
    }
    
    // Check for pause.
    if (responsesWaiting[1]) {
      if (waitingForLastTick[1]) {
        // If all responses recieved...
        if (pauseAck[0] && pauseAck[1] && pauseAck[2]) {
          // Reset values back to default.
          pauseAck[0] = false;
          pauseAck[1] = false;
          pauseAck[2] = false;
          responsesWaiting[1] = false;
          waitingForLastTick[1] = false;
          retryAttempts[1] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_PAUSED;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[1]--;
          sendMessage(PAUSE);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[1] = true;
      }
    }
    
    // Check for resume.
    if (responsesWaiting[2]) {
      if (waitingForLastTick[2]) {
        // If all responses recieved...
        if (resumeAck[0] && resumeAck[1] && resumeAck[2]) {
          // Reset values b to default.
          resumeAck[0] = false;
          resumeAck[1] = false;
          resumeAck[2] = false;
          responsesWaiting[2] = false;
          waitingForLastTick[2] = false;
          retryAttempts[2] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_RUNNING;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[2]--;
          sendMessage(RESUME);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[2] = true;
      }
    }
    
    // Check for start.
    if (responsesWaiting[3]) {
      if (waitingForLastTick[3]) {
        // If all responses recieved...
        if (startAck[0] && startAck[1] && startAck[2]) {
          // Reset values back to default.
          startAck[0] = false;
          startAck[1] = false;
          startAck[2] = false;
          responsesWaiting[3] = false;
          waitingForLastTick[3] = false;
          retryAttempts[3] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_RUNNING;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[3]--;
          sendMessage(START);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[3] = true;
      }
    }
    
    // Check for Stop.
    if (responsesWaiting[4]) {
      if (waitingForLastTick[4]) {
        // If all responses recieved...
        if (stopAck[0] && stopAck[1] && stopAck[2]) {
          // Reset values back to default.
          stopAck[0] = false;
          stopAck[1] = false;
          stopAck[2] = false;
          responsesWaiting[4] = false;
          waitingForLastTick[4] = false;
          retryAttempts[4] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_STOPPED;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[4]--;
          sendMessage(CTRL_STOP);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[4] = true;
      }
    }
    
    // Check for Reset.
    if (responsesWaiting[5]) {
      if (waitingForLastTick[5]) {
        // If all responses recieved...
        if (resetAck[0] && resetAck[1] && resetAck[2]) {
          // Reset values back to default.
          resetAck[0] = false;
          resetAck[1] = false;
          resetAck[2] = false;
          responsesWaiting[5] = false;
          waitingForLastTick[5] = false;
          retryAttempts[5] = RETRY_COUNT;
          
          // Update system state.
          systemState = SYSTEM_NOT_STARTED;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[5]--;
          sendMessage(RESET);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[5] = true;
      }
    }
    
    // Check for PAD1 pickup.
    if (responsesWaiting[6]) {
      if (waitingForLastTick[6]) {
        // If all responses recieved...
        if (pad1BlockAck) {
          // Reset values back to default.
          pad1BlockAck = false;
          responsesWaiting[6] = false;
          waitingForLastTick[6] = false;
          retryAttempts[6] = RETRY_COUNT;
        } else {
          // Resend message if not all responses are recieved.
          retryAttempts[6]--;
          sendMessage(REQ_PICKUP_PAD1);
        }
      } else {
        // Helps to identify which messages have recently been sent.
        waitingForLastTick[6] = true;
      }
    }
    
    // Check if retry attepts have hit 0.
    for (int i = 0; i < 7; i++) {
      if (retryAttempts[i] <= 0) {
        // If retry attepts are expired, signal emergency stop.
        sendMessage(EM_STOP);
      }
    }
    
    // Loop delay.
    OSTimeDly(500);
  }
}

/*
 * Task to manage the state of the LEDs.
 * The LEDs D1, D2, D3 & D4 are used to represent the system state.
 */
static void appTaskLED(void *pdata) {
  // Main loop for this task.
  // Handles updating of the periferal LEDs to show system state.
  while (true) {      
    // LED 1 (Ready & Readying)
    if (systemState == SYSTEM_NOT_STARTED || systemState == SYSTEM_STOPPED) {
      interfaceLedSetState(D1_LED, LED_ON);
    } else if (systemState == SYSTEM_RESETTING) {
      interfaceLedToggle(D1_LED);
    } else {
      interfaceLedSetState(D1_LED, LED_OFF);
    }
    
    // LED 2 (Running & Starting/Stopping)
    if (systemState == SYSTEM_RUNNING) {
      interfaceLedSetState(D2_LED, LED_ON);
    } else if (systemState == SYSTEM_STARTING || systemState == SYSTEM_STOPPING) {
      interfaceLedToggle(D2_LED);
    } else {
      interfaceLedSetState(D2_LED, LED_OFF);
    }
    
    // LED 3 (Paused & Pausing/Resuming)
    if (systemState == SYSTEM_PAUSED) {
      interfaceLedSetState(D3_LED, LED_ON);
    } else if (systemState == SYSTEM_PAUSING || systemState == SYSTEM_RESUMING) {
      interfaceLedToggle(D3_LED);
    } else {
      interfaceLedSetState(D3_LED, LED_OFF);
    }
    
    // LED 4 (Emergency)
    if (systemState == SYSTEM_EMERGENCY_STOP) {
      interfaceLedSetState(D4_LED, LED_ON);
    } else if (systemState == SYSTEM_ERROR_DETECTED) {
      interfaceLedToggle(D4_LED);
    } else {
      interfaceLedSetState(D4_LED, LED_OFF);
    }
    
    // Loop delay.
    OSTimeDly(250);
  }
}

/*
 * Task to handle all LCD output.
 * No semaphores used, this task is the only task interacting with the LCD screen.
 */
static void appTaskLCD(void *pdata) {
  // Initialise local task variables.
  int ackRow;
  
  // Main loop for this task.
  // LCD display loop. Updates LCD screen to show the current system state.
  while ( true ) {    
    // Show system state.
    lcdSetTextPos(1, 1);
    lcdWrite(displaySystemState[systemState]);
    
    // Show state of sensor 1.
    lcdSetTextPos(1, 2);
    lcdWrite("Pad 1");
    lcdSetTextPos(7, 2);
    if (controlItemPresent(CONTROL_SENSOR_1)) {
      lcdWrite("Y");
    } else {
      lcdWrite("N");
    }
    
    // Show state of sensor 2.
    lcdSetTextPos(1, 3);
    lcdWrite("Pad 2");
    lcdSetTextPos(7, 3);
    if (controlItemPresent(CONTROL_SENSOR_2)) {
      lcdWrite("Y");
    } else {
      lcdWrite("N");
    }
    
    // Show if ignoring pad input
    lcdSetTextPos(10, 2);
    lcdWrite("Ignor");
    lcdSetTextPos(17, 2);
    if (ignorePadInput) {
      lcdWrite("Y");
    } else {
      lcdWrite("N");
    }
    
    // Show pressed state of alarm button.
    lcdSetTextPos(10, 3);
    lcdWrite("AlBtn");
    lcdSetTextPos(17, 3);
    if (controlEmergencyStopButtonPressed()) {
      lcdWrite("Y");
    } else {
      lcdWrite("N");
    }
    
    // Show last message recieved.
    lcdSetTextPos(1, 5);
    if (messageDisplay != -1) {
      lcdWrite(displayMessageContents[messageDisplay]);
    }
    
      
    // Show which component reported an error (if any)
    lcdSetTextPos(1, 6);
    if (errorDetected != -1) {
      if (errorDetected == 0) {
        lcdWrite("Controller");
      } else if (errorDetected == 1) {
        lcdWrite("Robot 1");
      } else if (errorDetected == 2) {
        lcdWrite("Conveyor");
      } else {
        lcdWrite("Robot 2");
      }
    } else {
        lcdWrite("          ");
    }
    
    // Show status of responses.
    ackRow = 7;
    lcdSetTextPos(1, ackRow);
    lcdWrite("Responses  C  R1 R2");
    for (int y = 0; y < 7; y++) {
      if (responsesWaiting[y]) {
        ackRow++;
        
        // Display discription text.
        lcdSetTextPos(1, ackRow);
        lcdWrite(pendingAckMessageTitle[y]);
        
        // Display response info.
        for (int x = 0; x < 3; x++) {
          lcdSetTextPos(12 + (x*3), ackRow);
          
          // EM Stop
          if (y == 0) {
            if (emStopAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          } 
          // Pause
          else if (y == 1) {
            if (pauseAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          }
          // Resume
          else if (y == 2) {
            if (resumeAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          } 
          // Start
          else if (y == 3) {
            if (startAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          }
          // Stop
          else if (y == 4) {
            if (stopAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          }
          // Reset
          else if (y == 5) {
            if (resetAck[x]) {
              lcdWrite("Y");
            } else {
              lcdWrite("N");
            }
          }
          // Pad1 Pickup Request
          else if (y == 6) {
            if (x == 1) {
              if (pad1BlockAck) {
                lcdWrite("Y");
              } else {
                lcdWrite("N");
              }
            } else {
                lcdWrite("-");
            }
          }
        }
      }
    }
    // Fills the remaining rows with white space (only relevent messages are shown)
    while (ackRow < 12) {
      ackRow++;
      lcdSetTextPos(1, ackRow);
      lcdWrite("                     ");
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
 * This task updates the system state according to the new message being sent.
 * It updates the message to be sent and then releases the send semaphore. (Sending will be picked up by the appTaskCanSend task)
 */
static void sendMessage(int message) {  
  // Update system state if needed.
  if (message == EM_STOP && !responsesWaiting[0]) {
    // Emergency stop
    systemState = SYSTEM_ERROR_DETECTED;
    for (int i = 0; i < sizeof(responsesWaiting); i++) {
      responsesWaiting[i] = false;
    }
    controlAlarmSetState(CONTROL_ALARM_ON);
    for (int i = 0; i < 3; i++) {
      emStopAck[i] = false;
      pauseAck[i] = false;
      resumeAck[i] = false;
      startAck[i] = false;
      stopAck[i] = false;
      resetAck[i] = false;
    }
    responsesWaiting[0] = true;
    if (errorDetected == -1) {
      errorDetected = 0;
    }
  } else if (message == PAUSE && !responsesWaiting[1]) {
    // Pause
    systemState = SYSTEM_PAUSING;
    responsesWaiting[1] = true;
  } else if (message == RESUME && !responsesWaiting[2]) {
    // Resume
    systemState = SYSTEM_RESUMING;
    responsesWaiting[2] = true;
  } else if (message == START && !responsesWaiting[3]) {
    // Start
    systemState = SYSTEM_STARTING;
    responsesWaiting[3] = true;
  } else if (message == CTRL_STOP && !responsesWaiting[4]) {
    // CTRL stop
    systemState = SYSTEM_STOPPING;
    responsesWaiting[4] = true;
  } else if (message == RESET && !responsesWaiting[5]) {
    // Reset
    systemState = SYSTEM_RESETTING;
    for (int i = 0; i < sizeof(responsesWaiting); i++) {
      responsesWaiting[i] = false;
    }
    for (int i = 0; i < 3; i++) {
      emStopAck[i] = false;
      pauseAck[i] = false;
      resumeAck[i] = false;
      startAck[i] = false;
      stopAck[i] = false;
      resetAck[i] = false;
    }
    pad1BlockAck = false;
    responsesWaiting[5] = true;
    errorDetected = -1;
    controlAlarmSetState(CONTROL_ALARM_OFF);
  } else if (message == REQ_PICKUP_PAD1 && !responsesWaiting[6]) {
    // Request pickup from pad 1
    responsesWaiting[6] = true;
    pad1BlockAck = false;
    ignorePadInput = true;
  }
  
  // Update mesage to diaplay.
  messageDisplay = message;
  
  // Post the semephore so the sending task can take over.
  messageSend = message;
  OSSemPost(canSendSem);
}

/*
 * A helper function to decomparmentalise the processing of a message.
 * The control unit needs to handle many messages so most of this task is reacting to each message case.
 */
static void processRecievedMessage(int message) {
  messageDisplay = message;
  
  // Reacts appropriatly to each message.
  if (message == EM_STOP_ACK_CONV) {
    // Conveyor responding to emergency stop.
    emStopAck[0] = true;
  } else if (message == EM_STOP_ACK_ROB1) {
    // Robot1 responding to emergency stop.
    emStopAck[1] = true;
  } else if (message == EM_STOP_ACK_ROB2) {
    // Robot2 responding to emergency stop.
    emStopAck[2] = true;
  } else if (message == ERR_CONV) {
    // Conveyor is reporting an error. Emergency stop.
    sendMessage(EM_STOP);
    errorDetected = 2;
  } else if (message == ERR_ROB1) {
    // Robot 1 is reporting an error. Emergency stop.
    sendMessage(EM_STOP);
    errorDetected = 1;
  } else if (message == ERR_ROB2) {
    // Robot 2 is reporting an error. Emergency stop.
    sendMessage(EM_STOP);
    errorDetected = 3;
  } else if (message == PAUSE_ACK_CONV) {
    // Conveyor responding to pause instruction.
    pauseAck[0] = true;
  } else if (message == PAUSE_ACK_ROB1) {
    // Robot1 responding to pause instruction.
    pauseAck[1] = true;
  } else if (message == PAUSE_ACK_ROB2) {
    // Robot2 responding to pause instruction.
    pauseAck[2] = true;
  }  else if (message == RESUME_ACK_CONV) {
    // Conveyor responding to resume instruction.
    resumeAck[0] = true;
  } else if (message == RESUME_ACK_ROB1) {
    // Robot1 responding to resume instruction.
    resumeAck[1] = true;
  } else if (message == RESUME_ACK_ROB2) {
    // Robot2 responding to resume instruction.
    resumeAck[2] = true;
  } else if (message == START_ACK_CONV) {
    // Conveyor responding to start instruction.
    startAck[0] = true;
  } else if (message == START_ACK_ROB1) {
    // Robot1 responding to start instruction.
    startAck[1] = true;
  } else if (message == START_ACK_ROB2) {
    // Robot2 responding to start instruction.
    startAck[2] = true;  
  } else if (message == CTRL_STOP_ACK_CONV) {
    // Conveyor responding to stop instruction.
    stopAck[0] = true;
  } else if (message == CTRL_STOP_ACK_ROB1) {
    // Robot1 responding to stop instruction.
    stopAck[1] = true;
  } else if (message == CTRL_STOP_ACK_ROB2) {
    // Robot2 responding to stop instruction.
    stopAck[2] = true;
  } else if (message == RESET_ACK_CONV) {
    // Conveyor responding to reset instruction.
    resetAck[0] = true;
  } else if (message == RESET_ACK_ROB1) {
    // Robot1 responding to reset instruction.
    resetAck[1] = true;
  } else if (message == RESET_ACK_ROB2) {
    // Robot2 responding to reset instruction.
    resetAck[2] = true;
  } else if (message == ACK_PICKUP_PAD1) {
    // Robot1 responding to pad1 picup request.
    pad1BlockAck = true;
  } else if (message == CHK_PAD1_PICKUP) {
    // Robot1 is enquiring about the status of pad 1.
    if (!controlItemPresent(CONTROL_SENSOR_1)) {
      // The pickup was a success
      sendMessage(ACK_CHK_PAD1_PICKUP);
      ignorePadInput = false;
    } else {
      // The block is still there 
      sendMessage(NACK_CHK_PAD1_PICKUP);
    }
  } else if (message == REQ_DROP_PAD2) {
    // Robot2 is enquiring about the status of pad 2.
    if (controlItemPresent(CONTROL_SENSOR_2)) {
      // A block is present.
      sendMessage(NACK_DROP_PAD2);
      pad2WaitingAck = true;
    } else {
      // No block detected.
      sendMessage(ACK_DROP_PAD2);
    } 
  } else if (message == CHK_PAD2_DROP) {
      // Robot2 is enquiring if the drp on pad2 was a success.
      if (controlItemPresent(CONTROL_SENSOR_2)) {
        // A block is present.
        sendMessage(ACK_CHK_PAD2_DROP);
      } else {
        // No block detected.
        sendMessage(NACK_CHK_PAD2_DROP);
      }
  }
}