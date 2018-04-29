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
Date-29/04/18 //nearly done


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
Struct generalVars
This struct is used to hold the general system information which is shared between tasks
*/
struct GeneralVars{
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
  
  uint8_t test0;
  uint8_t test1;
  uint8_t test2;
  uint8_t test3;
  uint32_t performance;
  
};
/**
This struct is used to hold information used in to create timers
*/
struct Counter{
  uint32_t y1;
  uint32_t y2;
  uint32_t y3;
  INT32U blockTimeOut1;
  INT32U blockTimeOut2;
};
/**
this struct is used to hold button states
*/
struct ButtonBool{
  uint8_t jsRTPressed;// right
  uint8_t jsLTPressed;// left
  uint8_t jsUpPressed;// up
  uint8_t jsDownPressed;// down
  uint8_t jsCentPressed;// centre
  uint8_t but1Pressed;// but1
  uint8_t but2Pressed;// but2
};




static canMessage_t can1RxBuf;
static struct ButtonBool btnBool={0,0,0,0,0,0,0};
static struct GeneralVars generalVars={0,0,0,0,0,0,0,0,0,0};
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
static void displayInfo(struct Counter performance,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct GeneralVars generalVars);

uint32_t readWatch(void);//Reads the watch and returns it's value

//
static uint32_t sendMes(uint32_t mesID);//sends a message on the CAN
//emergency stops
static struct GeneralVars emConveyerTimeout(struct GeneralVars generalVars);//sets the variables after the conveyer declares an emergency stop
static struct GeneralVars emController(struct GeneralVars generalVars);//sets the variables after an emergency stop message is recieved from the controller
//start system
static struct GeneralVars resetSystem(struct GeneralVars generalVars);////sets the variables to reset state
static struct GeneralVars startSystem(struct GeneralVars generalVars);//sets the variables to start state
//
static struct GeneralVars successfulDrop(struct GeneralVars generalVars);//sets the variables after a block is succesfully dropped
//counter
static struct Counter countUp(struct Counter counter);//increments a counter

static struct GeneralVars readSensors(struct GeneralVars generalVars);//reads the sensor values

static struct GeneralVars readyForBlock(struct GeneralVars generalVars);//sets the variables to readyForBlock state

static struct GeneralVars pauseSystem(struct GeneralVars generalVars);//sets the variables to pause state
static struct GeneralVars resumeSystem(struct GeneralVars generalVars);//sets the variables to resume state

static void processMessage(canMessage_t msg);//function to process the message recieved


/**
These are the variables used in the performance analysis
*/
uint32_t executionStart=0;
uint32_t executionEnd=0;
uint32_t executionTime=0;
/**
This is the main function of the program
*/
int main() {
  
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
  
  /* Initialise the OS */
  OSInit();                                                   
  
  /* Create Tasks
  
  appTaskCanRead
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



/**
This task is used to recieve messages from the CAN.
It is event driven as it is only run when a message is recieved.
*/
static void appTaskCanRead(void *pdata) {
  uint8_t error;
  //the initialisation of the can interrupt handler
  canRxInterrupt(canHandler);//Event-Driven
  // uint32_t counter=0;
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  osStartTick();
  
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
      generalVars.stopConv=77;
    }else if (btnBool.jsRTPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      btnBool.jsRTPressed = 0;
      generalVars.stopConv=77;
    }else if (btnBool.jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
      btnBool.jsUpPressed = 0;
      generalVars.stopConv=77;      
    }else if (btnBool.jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
      btnBool.jsDownPressed = 0;
      generalVars.stopConv=77;     
    }else if (btnBool.jsCentPressed && (!isButtonPressedInState(btnState, JS_CENTRE))) {
      btnBool.jsCentPressed = 0;
      generalVars.stopConv=77;     
    }else if (btnBool.but1Pressed && (!isButtonPressedInState(btnState, BUT_1))) {//left button
      btnBool.but1Pressed = 0;
      conveyorSetState(CONVEYOR_OFF);   
      generalVars.blockCount=0;            
    }else if (btnBool.but2Pressed && (!isButtonPressedInState(btnState, BUT_2))) {//right button
      btnBool.but2Pressed = 0;
      generalVars.start=49;
    }
    
  }
}

/**
This task is used to operate the conveyer belt and its sensors.
It works by taking the global variables
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
  uint32_t pickUpDelay=1000;
  while (true) {
    while(generalVars.emStopConv||generalVars.emStop){
      generalVars.test2++;
      displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  generalVars);
      OSTimeDly(50);
    }
    /**
    This is the while loop for the idle state.
    **/
    while(!generalVars.start){
      generalVars.test1++;
      displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  generalVars);
      OSTimeDly(50);
    }
    /**
    This is the while loop for the running state
    */
    while (true) {
      generalVars.test0++;
      executionStart=readWatch();
      
      /**
      This if statement is triggered when an emergency stop is called by the controller. it stops the conveyer belt motor and breaks the running state while loop to move to the emergency stop state
      */
      if(generalVars.emStop){
        conveyorSetState(CONVEYOR_OFF); 
        break;
      }              
      /**
      This if statement is for the conveyer to trigger an emergency stop. It works by incrementing a timer while the conveyer is moving a block. 
      If the conveyer runs for too long without a block being removed then an emergency stop is called and a message is sent to the controller and the running while loop is broken and the state changes to emergency stop.
      */
      if(generalVars.blockCount&&!generalVars.blockEndWaiting){
        emCounter=countUp(emCounter);
        emCounter.blockTimeOut1+=emCounter.y3;
        if(emCounter.blockTimeOut1>emTimeOut &&!generalVars.stopConv ){
          generalVars.lastSentMsg=sendMes( ERR_CONV);
          generalVars=emConveyerTimeout(generalVars);
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

      if(generalVars.ctrlStop){
        if(!generalVars.stopConv&&!generalVars.blockCount){
          generalVars.start=0;
          conveyorSetState(CONVEYOR_OFF); 
          break;     
        }
      }
      /**
      This if statement is used to move the system into the pause state. in the pause state the conveyer motor is turned off and a while loop is ran idefinitely until a resume message is recieved on the CAN bus.
      */
      if(generalVars.pause){
        conveyorSetState(CONVEYOR_OFF);  
        while(generalVars.pause){
          generalVars.test3++;
          
          displayInfo( performanceC, emCounter,dropCounter,pickUpCounter,  generalVars);
        }
      }
      //read sensors
      generalVars=readSensors(generalVars);    
      /**
      This if statement is used to stop the conveyer when a message has been recieved from robot1 that a block is ready.
      */
      if(generalVars.stopConv){
        conveyorSetState(CONVEYOR_OFF);
      }
      /**
      This if statement is used to increment a counter so that when robot
      */
      if(generalVars.blockEndWaiting && !generalVars.convS2){     
        pickUpCounter=countUp(pickUpCounter);
        pickUpCounter.blockTimeOut1+=pickUpCounter.y3;
      }else{
        pickUpCounter=countUp(pickUpCounter);
        pickUpCounter.blockTimeOut1=0;
      }
      pickUpCounter.blockTimeOut2+=pickUpCounter.y3;    
      if(!generalVars.blockEndWaiting && !generalVars.stopConv && generalVars.blockCount && !generalVars.convS2){
        if(conveyorGetState() !=CONVEYOR_REVERSE){
          conveyorSetState(CONVEYOR_REVERSE);
        }
        /**
        This if statement is triggered when a block reaches the end of the conveyer belt
        */
      }else if(generalVars.blockCount  && generalVars.convS2){//&& !convVars.blockEndWaiting
        conveyorSetState(CONVEYOR_OFF);
        generalVars.blockEndWaiting=6;
        /** 
        This if statement is used to send a message to robot2 to request they pickup a block. if they have responded with an ACK then the if statement wont be triggered.
        */
        if(!generalVars.rob2ACK){
          generalVars.lastSentMsg=sendMes(REQ_PICKUP_CONV);
          if(!generalVars.performance){            
          }
          generalVars.performance=55;
        }       
        /** 
        This if statement is triggered when a block is sucessfully removed
        */
      }else if(generalVars.blockEndWaiting && !generalVars.convS2 && (pickUpCounter.blockTimeOut1>pickUpDelay)){
        //successful removal
        //recieve ACK
        pickUpCounter.blockTimeOut1=0;
        generalVars.blockEndWaiting=0;
        if(generalVars.blockCount>0){
          generalVars.blockCount--;
        }    
        //OSTimeDly(1500);
      }   
      executionEnd=readWatch();
      
      displayInfo(performanceC, emCounter,dropCounter,pickUpCounter,  generalVars);
      
      /**
      Small delay to allow for the lower priority buttons task to run.
      */
      OSTimeDly(50);
    }
    displayInfo(performanceC,  emCounter,dropCounter,pickUpCounter,  generalVars);
  } 
}

/**
This function is used to process the message recieved from the CAN bus
*/
static void processMessage(canMessage_t msg){
  generalVars.lastRecieved=msg.id;
  if(msg.id==START){//starts the system
    generalVars.lastSentMsg=sendMes( START_ACK_CONV);//acknowledgement that the system has started
    generalVars=startSystem(generalVars);
  }else if(msg.id==EM_STOP){
    generalVars.lastSentMsg=sendMes(EM_STOP_ACK_CONV);//acknowledgement of the emergency stop
    generalVars=emController(generalVars);
  }else if(msg.id==PAUSE){//pauses the system
    generalVars.lastSentMsg=sendMes( PAUSE_ACK_CONV);//acknowledgement of the pause 
    generalVars=pauseSystem(generalVars);
  }else if(msg.id==RESUME){//resumes the system from a pause
    generalVars.lastSentMsg=sendMes( RESUME_ACK_CONV);//acknowledgement of the resume
    generalVars=resumeSystem(generalVars);
  }else if(msg.id==RESET){//resets the system after an emergency stop
    generalVars=resetSystem(generalVars);
    generalVars.lastSentMsg=sendMes( RESET_ACK_CONV);//acknowledgement of the reset
  }else if(msg.id==CTRL_STOP){//performs a controlled stop of the system
    generalVars.ctrlStop=99;
    generalVars.lastSentMsg=sendMes( CTRL_STOP_ACK_CONV);//acknowledgement of the controlled stop
  }else if(msg.id==ACK_PICKUP_CONV){//the acknowledgement of a request to pick a block up from the conveyer
    generalVars.rob2ACK=78;
  }else if(msg.id==REQ_DROP_CONV && !generalVars.stopConv){//robot1 requests to drop a block on the conveyer
    if(!generalVars.convS1){
      generalVars.lastSentMsg=sendMes(ACK_DROP_CONV);//acknowledgement that it is safe to drop
      generalVars=readyForBlock(generalVars);
    }else{
      generalVars.lastSentMsg=sendMes(NACK_DROP_CONV);//acknowledgement that it is not safe to drop
    }
  } else if(msg.id==CHK_CONV_PICKUP){//robot2 checks if the block was successfully picked up
    if(!generalVars.blockEndWaiting){
      generalVars.rob2ACK=0;
      generalVars.lastSentMsg=sendMes( ACK_CHK_CONV_PICKUP); //acknowledgement that it was successfully picked up
    }else{
      generalVars.lastSentMsg=sendMes( NACK_CHK_CONV_PICKUP);//acknowledgement that it was not successfully picked up      
    }
  }else if(msg.id==CHK_CONV_DROP){//robot2 checks if the block was successfully dropped off
    if(generalVars.convS1){
      generalVars.lastSentMsg=sendMes(ACK_CHK_CONV_DROP);  //acknowledgement that it is not picked up
      generalVars=successfulDrop(generalVars);
    }else{
      generalVars.lastSentMsg=sendMes( NACK_CHK_CONV_DROP); //acknowledgement that it is not dropped
    }
  }
  
}


/**
This function is used to display system information to the LCD screen
*/
static void displayInfo(struct Counter performanceC,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct GeneralVars generalVars){
  
  ///**
  
  //conveyer display
  // uint8_t error;
  if(executionEnd > executionStart)  {  // Test for overflow of register
    executionTime = (executionEnd - executionStart);
    
  }else{
    //executionTime=666;
  }
  executionTime = (executionEnd - executionStart);
  
  OSSemPend(LCDsem, 0, &error);//pends a semaphore so that no other tasks can use the LCD
  lcdSetTextPos(1, 0);
  lcdWrite("ConvS 1|2  :%02u|%02u     ",generalVars.convS1,generalVars.convS2);
  lcdSetTextPos(1, 1);
  lcdWrite("blckNo:%2u stoConv:%2u    ",generalVars.blockCount,generalVars.stopConv);
  lcdSetTextPos(1, 2);
  lcdWrite("CanIn/Out:%02u|%02u       ",generalVars.lastRecieved,generalVars.lastSentMsg);
  lcdSetTextPos(1, 3);
  lcdWrite("start:%3u stop:%3u           ",generalVars.start,generalVars.ctrlStop);
  lcdSetTextPos(1, 4); 
  lcdWrite("|| / |>  :%02u|%02u     ",generalVars.pause,generalVars.resume); 
  //lcdSetTextPos(1, 3);
  //lcdWrite("DrPu:%05u|%05u        ",dropCounter.blockTimeOut2,pickUpCounter.blockTimeOut2);
  
  lcdSetTextPos(1, 5); 
  lcdWrite("eCO/eCV/TO:%2u|%2u|%05d   ",generalVars.emStop,generalVars.emStopConv,emCounter.blockTimeOut1);  
  lcdSetTextPos(1, 6);
  lcdWrite("s-Drp/Pck:%02u|%02u      ",generalVars.successDrop,generalVars.successPick); 
  lcdSetTextPos(1, 7); 
  lcdWrite("Rob2 ACK :%3u           ",generalVars.rob2ACK);  
  lcdSetTextPos(1, 8); 
  lcdWrite("blkNdWait:%3u           ",generalVars.blockEndWaiting);  
  lcdSetTextPos(1, 9); 
  // lcdWrite("Delay D/P:%05u|%05u      ",dropCounter.blockTimeOut1,pickUpCounter.blockTimeOut1); 
  lcdWrite("t0:%03u t1:%03u u",generalVars.test0,generalVars.test1); 
  
  
  
  lcdSetTextPos(1, 10); 
  lcdWrite("t2:%03u t3:%03u",generalVars.test2,generalVars.test3); 
  
  //lcdWrite("exeTi %010u",executionTime); 
  //lcdWrite("t0 :%02u t1 :%02u",convVars.test0,convVars.test1);
  //  **/
  error = OSSemPost(LCDsem);//posts a semaphore once the LCD function is finished
  
}
static struct GeneralVars readyForBlock(struct GeneralVars generalVars){
  generalVars.successDrop=0;
  generalVars.stopConv=76;
  return generalVars;
}
/**
This function is used to set the system variables in the event of an emergency stop triggered by the conveyer belt
*/
static struct GeneralVars emConveyerTimeout(struct GeneralVars generalVars){
  generalVars.emStopConv=81;
  generalVars.emStop=87;
  return generalVars;
}
/**
This function is used to set the system variables in the event of an emergency stop triggered by the controller
*/
static struct GeneralVars emController(struct GeneralVars generalVars){
  generalVars.emStopConv=83;
  generalVars.emStop=89;
  
  return generalVars;
}


/**
This function is used to set the system variables in the event of a reset after an emergency stop
*/
static struct GeneralVars resetSystem(struct GeneralVars generalVars){
  generalVars.emStop=0;
  generalVars.emStopConv=0;
  generalVars.start=0;
  generalVars.blockCount=0;
  generalVars.stopConv=0;
  generalVars.blockEndWaiting=0;
  generalVars.successDrop=0;
  return generalVars;
}
/**
This function is used to set the system variables when the system is started
*/
static struct GeneralVars startSystem(struct GeneralVars generalVars){
  generalVars.start=48;
  generalVars.ctrlStop=0;
  return generalVars;
}
/**
This function is used to set the system variables when a block is dropped on the conveyer belt
*/
static struct GeneralVars successfulDrop(struct GeneralVars generalVars){
  generalVars.stopConv=0;
  generalVars.successDrop=99;
  generalVars.blockCount++;
  return generalVars;
}
/**
This function sets the variables for the system pause state
*/
static struct GeneralVars pauseSystem(struct GeneralVars generalVars){
  generalVars.pause=55;
  generalVars.resume=0;
  return generalVars;
}
/**
This function sets the variables for the system resume state
*/
static struct GeneralVars resumeSystem(struct GeneralVars generalVars){
  generalVars.resume=66;
  generalVars.pause=0;
  return generalVars;
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
This function is used to send messages on the CAN bus.
It also toggles LEDs when a message is sent
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
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    OSSemPost(can1RxSem);
  }
}
/**
This function is used to read the sensors of the conveyer belt
*/
static struct GeneralVars readSensors(struct GeneralVars generalVars){
  if(conveyorItemPresent(CONVEYOR_SENSOR_1)){
    generalVars.convS1=4;
  }else{
    generalVars.convS1=0;
  }
  if(conveyorItemPresent(CONVEYOR_SENSOR_2)){
    generalVars.convS2=5;
  }else{
    generalVars.convS2=0;
  }
  return generalVars;
}
/**
This function is used to read a timer for the performance analysis
*/
uint32_t readWatch(void){
  uint32_t counter = 0;
  counter = T1TC;  // get the value of the timer counter
  return counter;  // return the value of the timer
}