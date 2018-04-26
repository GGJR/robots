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

Gruff James
Date-26/04/18 //nearly done


*/

#include <stdbool.h>
#include <ucos_ii.h>
#include <bsp.h>
#include <osutils.h>
#include <leds.h>
#include <lcd.h>
#include <buttons.h>
#include <interface.h>
#include <conveyor.h>
#include <can.h>
#include <timers.h>
#include <iolpc2378.h>
#include <messageTable.h>

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

static OS_EVENT *can1RxSem;
static OS_EVENT *LCDsem;

//the last message sent on the CAN bus

/**
Struct ConvVars
This struct is used to hold the general system information which is shared between tasks
*/
struct ConvVars{
  uint8_t stopConv;//if the conveyer has stopped for a block
  uint8_t convS1;//if sensor 1 is blocked
  uint8_t convS2;//if sensor 2 is blocked
  uint8_t start;//if in running state
  uint8_t emStop;//if in an emergency stop from controller
  uint8_t emStopConv;//if in an emergency stop from conveyer
  uint8_t blockCount;//the number of blocks on the conveyer
  uint32_t rob2ACK;//when robot 2 returns an ACK after the request for a block to be picked up
  uint8_t blockEndWaiting;//when a block is waiting on the end sensor
  uint8_t successDrop;//if a block has been successfully dropped on the conveyer
  uint8_t successPick;//if a block has been succesfully picked up from the conveyer
  
  uint8_t pause;//if the system is paused
  uint8_t resume;//if the system has been resumed
  
  uint8_t reset;//if the system has been reset after an emergency stop
  uint8_t ctrlStop;//if the system has had a controlled stop
  uint8_t lastRecieved;//the last message recieved from the CAN
  uint32_t lastSentMsg;//the last message sent on the CAN
  
  
  uint32_t test0;
  uint32_t performance;
  
};
//This struct is used to hold information used in to create timers
struct Counter{
  uint32_t y1;
  uint32_t y2;
  uint32_t y3;
  INT32U blockTimeOut1;
  INT32U blockTimeOut2;
};
//this struct is used to hold button states
struct ButtonBool{
  uint8_t jsRTPressed;// right
  uint8_t jsLTPressed;// left
  uint8_t jsUpPressed;// up
  uint8_t jsDownPressed;// down
  uint8_t jsCentPressed;// centre
  uint8_t but1Pressed;// but1
  uint8_t but2Pressed;// but2
};//7 members




static canMessage_t can1RxBuf;
static struct ButtonBool btnBool={0,0,0,0,0,0,0};

static struct ConvVars convVars={0,0,0,0,0,0,0,0,0,0};

INT8U error;
/*************************************************************************
*                  PRIORITIES
*************************************************************************/
enum {
  APP_TASK_CAN_READ_PRIO= 4,
  APP_TASK_CTRL_CONV_PRIO,
  APP_TASK_BTNS_PRIO
};
/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/
enum {
  APP_TASK_CAN_READ_STK_SIZE = 256,
  APP_TASK_CTRL_CONV_STK_SIZE = 256,
  APP_TASK_BTNS_STK_SIZE = 256
};
static OS_STK appTaskCanReadStk[APP_TASK_CAN_READ_STK_SIZE];
static OS_STK appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE];
static OS_STK appTaskBtnsStk[APP_TASK_BTNS_STK_SIZE];
/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

//The system tasks
static void appTaskCanRead(void *pdata);
static void appTaskCtrlConv(void *pdata);
static void appTaskBtns(void *pdata);

//the CAN message interuppt

static void canHandler(void);
//This function is used to display information to the LCD
static void displayInfo(struct Counter performance,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct ConvVars convVars);

//
uint32_t readWatch(void);                 //Reads the watch and returns it's value

//
static uint32_t sendMes(uint32_t mesID);
//emergency stops
static struct ConvVars emConveyerTimeout(struct ConvVars convVars);
static struct ConvVars emController(struct ConvVars convVars);
//start system
static struct ConvVars resetSystem(struct ConvVars convVars);
static struct ConvVars startSystem(struct ConvVars convVars);
//
static struct ConvVars successfulDrop(struct ConvVars convVars);
//counter
static struct Counter countUp(struct Counter counter);
//btns
static struct ConvVars readSensors(struct ConvVars convVars);

static struct ConvVars readyForBlock(struct ConvVars convVars);

static struct ConvVars pauseSystem(struct ConvVars convVars);
static struct ConvVars resumeSystem(struct ConvVars convVars);

static void processMessage(canMessage_t msg);


uint32_t executionTime=0;
uint32_t executionStart=0;
uint32_t executionEnd=0;

/**
This is the main function of the program
*/
int main() {
  uint8_t error;
  
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
  
  /* Initialise the OS */
  OSInit();                                                   
  
  /* Create Tasks
  appTaskCanRead
  */
  /* appTaskCanRead
  This task is used to read from the CAN bus
  */
  OSTaskCreate(appTaskCanRead,                               
               (void *)0,
               (OS_STK *)&appTaskCanReadStk[APP_TASK_CAN_READ_STK_SIZE - 1],
               APP_TASK_CAN_READ_PRIO);
  /* appTaskCtrlConv
  This task is used to operate the Conveyer belt and its sensors
  */
  OSTaskCreate(appTaskCtrlConv,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE - 1],
               APP_TASK_CTRL_CONV_PRIO);
  /* appTaskBtns
  This task is used to operate the buttons
  */
  OSTaskCreate(appTaskBtns,                               
               (void *)0,
               (OS_STK *)&appTaskBtnsStk[APP_TASK_BTNS_STK_SIZE - 1],
               APP_TASK_BTNS_PRIO);  
  
  
  /* Start the OS */
  can1RxSem = OSSemCreate(0);//Event-Driven
  LCDsem = OSSemCreate(1);
  
  
  
  
  initWatch();
  startWatch();
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
} 


/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/





static void appTaskCanRead(void *pdata) {
  uint8_t error;
  canRxInterrupt(canHandler);//Event-Driven
  osStartTick();
  // uint32_t counter=0;
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  
  //this makes the LEDs toggle in a cool way
  ledToggle(USB_LINK_LED);
  interfaceLedToggle(D1_LED);
  interfaceLedToggle(D3_LED);
  /* 
  * Now execute the main task loop for this task
  */
  canMessage_t msg={0,0,0,0};
  while (true) {
    /**
    The LEDs toggle whenever a message is recieved
    */
    ledToggle(USB_LINK_LED);
    ledToggle(USB_CONNECT_LED);
    OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    processMessage(msg);
    executionEnd = readWatch();
  }
}


/**
This task is used to operate the buttons. 
It is the lowest priority and only runs when the others are not being run.
It runs in a infinite while loop where the buttons are continuously polled.
*/
static void appTaskBtns(void *pdata) {
  uint32_t btnState;
  while(true){    
    btnState = buttonsRead();   
    if (isButtonPressedInState(btnState, JS_LEFT)) {
      btnBool.jsLTPressed = 2;
    }else if (isButtonPressedInState(btnState, JS_RIGHT)) {
      btnBool.jsRTPressed = 2;
    }else if (isButtonPressedInState(btnState, JS_UP)) {
      btnBool.jsUpPressed = 2;
    }else if (isButtonPressedInState(btnState, JS_DOWN)) {
      btnBool.jsDownPressed = 2;
    }else if (isButtonPressedInState(btnState, JS_CENTRE)) {
      btnBool.jsCentPressed = 2;
    }else if (isButtonPressedInState(btnState, BUT_1)) {//left button
      btnBool.but1Pressed = 2;
    }else if (isButtonPressedInState(btnState, BUT_2)) {//right button
      btnBool.but2Pressed = 2;
    }  
    if (btnBool.jsLTPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
      btnBool.jsLTPressed = 0;
      convVars.stopConv=77;
    }else if (btnBool.jsRTPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      btnBool.jsRTPressed = 0;
      convVars.stopConv=77;
    }else if (btnBool.jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
      btnBool.jsUpPressed = 0;
      convVars.stopConv=77;
      
    }else if (btnBool.jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
      btnBool.jsDownPressed = 0;
      convVars.stopConv=77;
      
    }else if (btnBool.jsCentPressed && (!isButtonPressedInState(btnState, JS_CENTRE))) {
      btnBool.jsCentPressed = 0;
      convVars.stopConv=77;
      
    }else if (btnBool.but1Pressed && (!isButtonPressedInState(btnState, BUT_1))) {//left button
      btnBool.but1Pressed = 0;
      conveyorSetState(CONVEYOR_OFF);   
      convVars.blockCount=0;        
      
    }else if (btnBool.but2Pressed && (!isButtonPressedInState(btnState, BUT_2))) {//right button
      btnBool.but2Pressed = 0;
      convVars.start=49;
    }
  }
}

/**
This task is used to operate the conveyer belt and its sensors
*/
static void appTaskCtrlConv(void *pdata) {
  //performance analysis
  struct Counter performanceC={0,0,0,0,0};
  
  //timer counters
  struct Counter emCounter={0,0,0,0,0};
  struct Counter dropCounter={0,0,0,0,0};
  struct Counter pickUpCounter={0,0,0,0,0};
  
  
  //static struct ButtonBool btnBool={0,0,0,0,0,0,0};
  uint32_t emTimeOut=50000;//conveyer timeout for emergency stop
  uint32_t dropDelay=2000;//The delay for when robot1 picks a block up
  uint32_t pickUpDelay=1000;
  while (true) {
    
    
    //starting loop & in emergency stop
    /**
    while(convVars.emStopConv){
    if(!msg.id==EM_STOP||!msg.id==EM_STOP_ACK_ROB1||!msg.id==EM_STOP_ACK_ROB1){
    lastSentMsg=sendMes( ERR_CONV);
  }
    if(msg.id!=RESET){
    break;
  }
  }
    **/
    while(convVars.emStopConv||convVars.emStop){
      displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  convVars);
      OSTimeDly(50);
    }
    /**
    This is the while loop for the idle state.
    **/
    while(!convVars.start){
      //msg=msgGlobal;
      
      //moved
      //if(msg.id==RESET){
      //  lastSentMsg=sendMes( RESET_ACK_CONV);
      //}      
      //**/
      //if(msg.id==CTRL_STOP){
      //  lastSentMsg=sendMes( CTRL_STOP_ACK_CONV);
      //}
      //moved
      displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  convVars);
      OSTimeDly(50);
    }
    /**
    This is the while loop for the running state
    */
    while (true) {
      /**
      This if statement is triggered when an emergency stop is called by the controller. it stops the conveyer belt motor and breaks the running state while loop to move to the emergency stop state
      */
      if(convVars.emStop){
        conveyorSetState(CONVEYOR_OFF); 
        break;
      }              
      /**
      This if statement is for the conveyer to trigger an emergency stop. It works by incrementing a timer while the conveyer is moving a block. 
      If the conveyer runs for too long without a block being removed then an emergency stop is called and a message is sent to the controller and the running while loop is broken and the state changes to emergency stop.
      */
      if(convVars.blockCount&&!convVars.blockEndWaiting){
        emCounter=countUp(emCounter);
        emCounter.blockTimeOut1+=emCounter.y3;
        if(emCounter.blockTimeOut1>emTimeOut &&!convVars.stopConv ){
          convVars.lastSentMsg=sendMes( ERR_CONV);
          convVars=emConveyerTimeout(convVars);
          conveyorSetState(CONVEYOR_OFF);    
          break;
        }
      }else{
        emCounter.blockTimeOut1=0;
        emCounter=countUp(emCounter);
      }
      emCounter.blockTimeOut2+=emCounter.y3;
      /**
      This state is used to trigger a controlled stop. This means that once the conveyer has finished trans... may need more code timer etc...
      */
      if(convVars.ctrlStop){
        conveyorSetState(CONVEYOR_OFF); 
        if(!convVars.stopConv&&!convVars.blockCount){
          convVars.start=0;
          break;     
        }
      }
      //system pause
      /**
      This if statement is used to move the system into the pause state. in the pause state the conveyer motor is turned off and a while loop is ran idefinitely until a resume message is recieved on the CAN bus.
      */
      if(convVars.pause){
        conveyorSetState(CONVEYOR_OFF);  
        while(convVars.pause){
          displayInfo( performanceC, emCounter,dropCounter,pickUpCounter,  convVars);
        }
      }
      //read sensors
      convVars=readSensors(convVars);    
      /**
      This if statement is used to stop the conveyer when a message has been recieved from robot1 that a block is ready.
      */
      if(convVars.stopConv){
        conveyorSetState(CONVEYOR_OFF);
      }
      /**
      This if statement is used to increment a counter so that when robot
      */
      if(convVars.blockEndWaiting && !convVars.convS2){     
        pickUpCounter=countUp(pickUpCounter);
        pickUpCounter.blockTimeOut1+=pickUpCounter.y3;
      }else{
        pickUpCounter=countUp(pickUpCounter);
        pickUpCounter.blockTimeOut1=0;
      }
      pickUpCounter.blockTimeOut2+=pickUpCounter.y3;    
   
      if(!convVars.blockEndWaiting && !convVars.stopConv && convVars.blockCount && !convVars.convS2){
        if(conveyorGetState() !=CONVEYOR_REVERSE){
          conveyorSetState(CONVEYOR_REVERSE);
        }
        /**
        This if statement is triggered when a block reaches the end of the conveyer belt
        */
      }else if(convVars.blockCount  && convVars.convS2){//&& !convVars.blockEndWaiting
        conveyorSetState(CONVEYOR_OFF);
        convVars.blockEndWaiting=6;
        /**
        This if statement is used to send a message to robot2 to request they pickup a block. if they have responded with an ACK then the if statement wont be triggered.
        */
        if(!convVars.rob2ACK){
          convVars.lastSentMsg=sendMes(REQ_PICKUP_CONV);
          if(!convVars.performance){            
          }
          convVars.performance=55;
        }
        
        
        
        /** 
        This if statement is triggered when a block is sucessfully removed
        */
      }else if(convVars.blockEndWaiting && !convVars.convS2 && (pickUpCounter.blockTimeOut1>pickUpDelay)){
        //successful removal
        //recieve ACK
        pickUpCounter.blockTimeOut1=0;
        convVars.blockEndWaiting=0;
        if(convVars.blockCount>0){
          convVars.blockCount--;
        }    
        //OSTimeDly(1500);
      }
      //counter
      
      /** moved
      
    }
      **/ //moved
      displayInfo(performanceC, emCounter,dropCounter,pickUpCounter,  convVars);
      OSTimeDly(50);
    }
    displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  convVars);
  } 
}

/**
This function is used to process the message recieved from the CAN bus
*/
static void processMessage(canMessage_t msg){
  convVars.lastRecieved=msg.id;
  if(msg.id==START){
    convVars.lastSentMsg=sendMes( START_ACK_CONV);
    if(convVars.emStop){
      convVars=resetSystem(convVars);
    }else{
      convVars=startSystem(convVars);
    }
  }else if(msg.id==PAUSE){
    convVars.lastSentMsg=sendMes( PAUSE_ACK_CONV);
    convVars=pauseSystem(convVars);
  }else if(msg.id==RESUME){
    convVars.lastSentMsg=sendMes( RESUME_ACK_CONV);
    convVars=resumeSystem(convVars);
  }else if(msg.id==ACK_PICKUP_CONV){
    convVars.rob2ACK=78;
    convVars.performance=0;
  }else if(msg.id==REQ_DROP_CONV && !convVars.stopConv){    
    if(!convVars.convS1){
      convVars.lastSentMsg=sendMes(ACK_DROP_CONV);
    convVars=readyForBlock(convVars);
    }else{
      convVars.lastSentMsg=sendMes(NACK_DROP_CONV);
    }
  } else if(msg.id==CHK_CONV_PICKUP){
    if(!convVars.blockEndWaiting){
      convVars.lastSentMsg=sendMes( ACK_CHK_CONV_PICKUP);  
      convVars.rob2ACK=0;
    }else{
      convVars.lastSentMsg=sendMes( NACK_CHK_CONV_PICKUP);       
    }
  }else if(msg.id==RESET){
    convVars.lastSentMsg=sendMes( RESET_ACK_CONV);
    resetSystem(convVars);
  }else if(msg.id==CTRL_STOP){
    convVars.ctrlStop=99;
    convVars.lastSentMsg=sendMes( CTRL_STOP_ACK_CONV);
  }else if(msg.id==EM_STOP){
    convVars.lastSentMsg=sendMes(EM_STOP_ACK_CONV);
    convVars=emController(convVars);
  }else if(msg.id==CHK_CONV_DROP){
    if(convVars.convS1){//convVars.successDrop
      convVars.lastSentMsg=sendMes(ACK_CHK_CONV_DROP);  
      convVars=successfulDrop(convVars);
    }else{
      convVars.lastSentMsg=sendMes( NACK_CHK_CONV_DROP); 
    }
  }
}


/**
This function is used to display system information to the LCD screen
*/
static void displayInfo(struct Counter performanceC,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct ConvVars convVars){
  
  ///**
  
  //conveyer display
  // uint8_t error;
  if(executionEnd > executionStart)  {  // Test for overflow of register
    executionTime = (executionEnd - executionStart);
  }else{
    executionTime=666;
  }

  OSSemPend(LCDsem, 0, &error);
  lcdSetTextPos(1, 0);
  lcdWrite("ConvS 1|2  :%02u|%02u     ",convVars.convS1,convVars.convS2);
  lcdSetTextPos(1, 1);
  lcdWrite("blckNo:%2u stoConv:%2u    ",convVars.blockCount,convVars.stopConv);
  lcdSetTextPos(1, 2);
  lcdWrite("CanIn/Out:%02u|%02u       ",convVars.lastRecieved,convVars.lastSentMsg);
  lcdSetTextPos(1, 3);
  lcdWrite("start:%3u stop:%3u           ",convVars.start,convVars.ctrlStop);
  lcdSetTextPos(1, 4); 
  lcdWrite("|| / |>  :%02u|%02u     ",convVars.pause,convVars.resume); 
  //lcdSetTextPos(1, 3);
  //lcdWrite("DrPu:%05u|%05u        ",dropCounter.blockTimeOut2,pickUpCounter.blockTimeOut2);
  
  lcdSetTextPos(1, 5); 
  lcdWrite("eCO/eCV/TO:%2u|%2u|%05d   ",convVars.emStop,convVars.emStopConv,emCounter.blockTimeOut1);  
  lcdSetTextPos(1, 6);
  lcdWrite("s-Drp/Pck:%02u|%02u      ",convVars.successDrop,convVars.successPick); 
  lcdSetTextPos(1, 7); 
  lcdWrite("Rob2 ACK :%3u           ",convVars.rob2ACK);  
  lcdSetTextPos(1, 8); 
  lcdWrite("blkNdWait:%3u           ",convVars.blockEndWaiting);  
  lcdSetTextPos(1, 9); 
  lcdWrite("Delay D/P:%05u|%05u      ",dropCounter.blockTimeOut1,pickUpCounter.blockTimeOut1); 
  
  
  
  lcdSetTextPos(1, 10); 
  lcdWrite("exeTi %010u",executionTime); 
  
  //  **/
  error = OSSemPost(LCDsem);

}
static struct ConvVars readyForBlock(struct ConvVars convVars){
      convVars.successDrop=0;
      convVars.stopConv=76;
return convVars;
}
/**
This function is used to set the system variables in the event of an emergency stop triggered by the conveyer belt
*/
static struct ConvVars emConveyerTimeout(struct ConvVars convVars){
  convVars.emStopConv=81;
  convVars.emStop=82;
  convVars.start=0;
  convVars.blockCount=0;
  convVars.stopConv=0;
  convVars.blockEndWaiting=0;   
  return convVars;
}
/**
This function is used to set the system variables in the event of an emergency stop triggered by the controller
*/
static struct ConvVars emController(struct ConvVars convVars){
  convVars.emStopConv=83;
  convVars.emStop=89;
  convVars.start=0;
  convVars.blockCount=0;
  convVars.stopConv=0;
  convVars.blockEndWaiting=0;
  return convVars;
}


/**
This function is used to set the system variables in the event of a reset after an emergency stop
*/
static struct ConvVars resetSystem(struct ConvVars convVars){
  convVars.emStop=0;
  convVars.emStopConv=0;
  return convVars;
}
/**
This function is used to set the system variables when the system is started
*/
static struct ConvVars startSystem(struct ConvVars convVars){
  convVars.start=48;
  return convVars;
}
/**
This function is used to set the system variables when a block is dropped on the conveyer belt
*/
static struct ConvVars successfulDrop(struct ConvVars convVars){
      convVars.stopConv=0;
      convVars.successDrop=99;
      convVars.blockCount++;
  return convVars;
}
static struct ConvVars pauseSystem(struct ConvVars convVars){
  convVars.pause=55;
  convVars.resume=0;
  return convVars;
}
static struct ConvVars resumeSystem(struct ConvVars convVars){
  convVars.resume=66;
  convVars.pause=0;
  return convVars;
}

static struct Counter countUp(struct Counter counter){
  counter.y1=OSTimeGet();
  counter.y3=counter.y1-counter.y2;
  counter.y2=counter.y1;
  return counter;
}


/**
static struct ConvVars copyPasteMe(struct ConvVars convVars){

return convVars;
}
**/
/**
This function is used to send messages on the CAN bus
*/
static uint32_t sendMes(uint32_t mesID){
  interfaceLedToggle(D1_LED);
  interfaceLedToggle(D2_LED);
  interfaceLedToggle(D3_LED);
  interfaceLedToggle(D4_LED);
  
  canMessage_t msgOut;
  msgOut.id=mesID;
  canWrite(CAN_PORT_1, &msgOut);
  return mesID;
}
/**
This function is the CAN interrupt handler. It is only triggered when a message is recieved on the CAN bus.
It posts the semaphore pended by the system task appTaskCanRead, so that once a message is recieved it is imediately processed by the task.
*/
static void canHandler(void) {
      executionStart = readWatch();
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    OSSemPost(can1RxSem);
  }
}
/**
This function is used to read the sensors of the conveyer belt
*/
static struct ConvVars readSensors(struct ConvVars convVars){
  if(conveyorItemPresent(CONVEYOR_SENSOR_2)){
    convVars.convS1=4;
  }else{
    convVars.convS1=0;
  }
  if(conveyorItemPresent(CONVEYOR_SENSOR_1)){
    convVars.convS2=5;
  }else{
    convVars.convS2=0;
  }
  return convVars;
}
uint32_t readWatch(void){
  uint32_t counter = 0;
  counter = T1TC;  // get the value of the timer counter
  return counter;  // return the value of the timer
}