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
Date-09/04/18 //nearly done


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


struct ConvVars{
  uint8_t stopConv;//if the conveyer has stopped for a block
  uint8_t convS1;//block on the start sensor
  uint8_t convS2;//block on the end sensor
  uint8_t start;//in nning state
  uint8_t emStop;//in an emergency stop from controller
  uint8_t emStopConv;
  uint8_t blockCount;//the number of blocks on the conveyer
  uint32_t rob2ACK;//
  uint8_t blockEndWaiting;
  uint8_t successDrop;
  uint8_t successPick;
  uint8_t test0;
  uint8_t test1;
  uint8_t test2;
  uint8_t test3;
  uint8_t test4;
  uint8_t pause;
  uint8_t resume;
  
  uint8_t reset;
  uint8_t ctrlStop;
  uint8_t lastRecieved;
  
};//10 members
struct Counter{
  uint32_t y1;
  uint32_t y2;
  uint32_t y3;
  INT32U blockTimeOut1;
  INT32U blockTimeOut2;
};
struct ButtonBool{
  uint8_t jsRTPressed;// right
  uint8_t jsLTPressed;// left
  uint8_t jsUpPressed;// up
  uint8_t jsDownPressed;// down
  uint8_t jsCentPressed;// centre
  uint8_t but1Pressed;// but1
  uint8_t but2Pressed;// but2
};//7 members
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

static void appTaskCanRead(void *pdata);

static void appTaskCtrlConv(void *pdata);
static void appTaskBtns(void *pdata);

uint32_t readWatch(void);                 //Reads the watch and returns it's value
static void canHandler(void);
static void displayInfo(uint32_t lastSentMsg,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct ConvVars convVars);

//
static void displayInfoLight(uint32_t counterb,uint32_t msgGlobal);

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
static struct ConvVars readBtns(struct ConvVars convVars);
static struct ConvVars readSensors(struct ConvVars convVars);


/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

static OS_EVENT *can1RxSem;
static OS_EVENT *canSendSem;


static OS_EVENT *LCDsem;
INT8U error;

static canMessage_t can1RxBuf;
static struct ButtonBool btnBool={0,0,0,0,0,0,0};

static struct ConvVars convVars={0,0,0,0,0,0,0,0,0,0};

int main() {
  uint8_t error;
  
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
  
  /* Initialise the OS */
  OSInit();                                                   
  
  /* Create Tasks */
  OSTaskCreate(appTaskCanRead,                               
               (void *)0,
               (OS_STK *)&appTaskCanReadStk[APP_TASK_CAN_READ_STK_SIZE - 1],
               APP_TASK_CAN_READ_PRIO);
  OSTaskCreate(appTaskCtrlConv,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE - 1],
               APP_TASK_CTRL_CONV_PRIO);
  OSTaskCreate(appTaskBtns,                               
               (void *)0,
               (OS_STK *)&appTaskBtnsStk[APP_TASK_BTNS_STK_SIZE - 1],
               APP_TASK_BTNS_PRIO);  
  /* Start the OS */
  can1RxSem = OSSemCreate(0);//Event-Driven
  canSendSem= OSSemCreate(0);
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
canMessage_t msgSendGlobal;

uint32_t lastSentMsg=0;

static void appTaskCanRead(void *pdata) {
  uint8_t error;
  canRxInterrupt(canHandler);//Event-Driven
  osStartTick();
  uint32_t counter=0;
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  
  /* 
  * Now execute the main task loop for this task
  */
  canMessage_t msgGlobal={0,0,0,0};
  while (true) {
    
    ///////////////////////////////////consider wiping can each time
    //can1RxBuf.id=0;
    // msgGlobal.id=0;
    counter++;
    ledToggle(USB_LINK_LED);
    ledToggle(USB_CONNECT_LED);
    OSSemPend(can1RxSem, 0, &error);
    msgGlobal = can1RxBuf;
    convVars.lastRecieved=msgGlobal.id;
    
    if(msgGlobal.id==START){
              lastSentMsg=sendMes( START_ACK_CONV);
      convVars.start=41;
      if(convVars.emStop){
        convVars=resetSystem(convVars);
      }else{
        convVars=startSystem(convVars);
      }
    }else if(msgGlobal.id==PAUSE){
      lastSentMsg=sendMes( PAUSE_ACK_CONV);
      
      convVars.pause=55;
      convVars.resume=0;
    }else if(msgGlobal.id==RESUME){
      lastSentMsg=sendMes( RESUME_ACK_CONV);
      
      convVars.resume=66;
      convVars.pause=0;
    }else if(msgGlobal.id==ACK_PICKUP_CONV){
      convVars.rob2ACK=78;
    }else if(msgGlobal.id==REQ_DROP_CONV && !convVars.stopConv){    
      if(!convVars.convS1){
        convVars.stopConv=76;
        lastSentMsg=sendMes(ACK_DROP_CONV);
        convVars.successDrop=0;
      }else{
        lastSentMsg=sendMes(NACK_DROP_CONV);
      }
    } else if(msgGlobal.id==CHK_CONV_PICKUP){
      if(!convVars.blockEndWaiting){
        lastSentMsg=sendMes( ACK_CHK_CONV_PICKUP);     
        convVars.rob2ACK=0;
        convVars.test2=83;
      }else{
        lastSentMsg=sendMes( NACK_CHK_CONV_PICKUP);       
        convVars.test2=69;
      }
    }else if(msgGlobal.id==RESET){
      lastSentMsg=sendMes( RESET_ACK_CONV);
    }else if(msgGlobal.id==CTRL_STOP){
      convVars.ctrlStop=99;
      lastSentMsg=sendMes( CTRL_STOP_ACK_CONV);
    }else if(msgGlobal.id==EM_STOP){
      lastSentMsg=sendMes(EM_STOP_ACK_CONV);
      convVars=emController(convVars);
    }else if(msgGlobal.id==CHK_CONV_DROP){
      if(convVars.successDrop){
        lastSentMsg=sendMes(ACK_CHK_CONV_DROP);  
        convVars.test3++;
      }else{
        lastSentMsg=sendMes( NACK_CHK_CONV_DROP); 
      }
      
      ///**
      //**/
      //OSTimeDly(200);
    }
  }
}
static void displayInfoLight(uint32_t counterb,uint32_t msgGlobalID){
  OSSemPend(LCDsem, 0, &error);
  
  lcdSetTextPos(1, 10);
  lcdWrite("msgID %02u Ctr %02u     ",counterb); 
  error = OSSemPost(LCDsem);
  ledToggle(USB_LINK_LED);
  ledToggle(USB_CONNECT_LED);
}



static void appTaskBtns(void *pdata) {
  uint32_t btnState;
  
  while(true){
    ///**
    
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
    //joystick logic
    if (btnBool.jsLTPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
      btnBool.jsLTPressed = 0;
    }else if (btnBool.jsRTPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
      btnBool.jsRTPressed = 0;
    }else if (btnBool.jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
      btnBool.jsUpPressed = 0;
    }else if (btnBool.jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
      btnBool.jsDownPressed = 0;
    }else if (btnBool.jsCentPressed && (!isButtonPressedInState(btnState, JS_CENTRE))) {
      btnBool.jsCentPressed = 0;
    }else if (btnBool.but1Pressed && (!isButtonPressedInState(btnState, BUT_1))) {//left button
      btnBool.but1Pressed = 0;
      conveyorSetState(CONVEYOR_OFF);    
    }else if (btnBool.but2Pressed && (!isButtonPressedInState(btnState, BUT_2))) {//right button
      btnBool.but2Pressed = 0;
    }
    // **/
    
    //OSTimeDly(2000000);
  }
}


static void appTaskCtrlConv(void *pdata) {
  struct Counter emCounter={0,0,0,0,0};
  struct Counter dropCounter={0,0,0,0,0};
  struct Counter pickUpCounter={0,0,0,0,0};
  
  
  //static struct ButtonBool btnBool={0,0,0,0,0,0,0};
  canMessage_t msgx={0,0,0,0};
  uint32_t emTimeOut=50000;//conveyer timeout for emergency stop
  uint32_t dropDelay=3000;//conveyer timeout for emergency stop
  uint32_t pickUpDelay=1000;
  //uint32_t btnState;
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
      displayInfo( lastSentMsg, emCounter,dropCounter,pickUpCounter,  convVars);
      
    }
    //msgGlobal.id=0;
    
    
    
    
    // **/  
    //      convVars.start=41;//move back
    
    
    //main running loop
    while (true) {
      //msg.id=0;
      // msg=msgGlobal;
      //checks for emergency stop
      convVars.test1++;
      if(convVars.emStop){
        conveyorSetState(CONVEYOR_OFF); 
        break;
      }              
      //for the time out emergency stop
      if(convVars.blockCount&&!convVars.blockEndWaiting){
        emCounter=countUp(emCounter);
        emCounter.blockTimeOut1+=emCounter.y3;
        if(emCounter.blockTimeOut1>emTimeOut &&!convVars.stopConv ){
          lastSentMsg=sendMes( ERR_CONV);
          convVars=emConveyerTimeout(convVars);
          conveyorSetState(CONVEYOR_OFF);    
        }
      }else{
        emCounter.blockTimeOut1=0;
        emCounter=countUp(emCounter);
      }
      emCounter.blockTimeOut2+=emCounter.y3;
      //breaks without
      if(convVars.ctrlStop){
        if(!convVars.stopConv&&!convVars.blockCount){
          convVars.start=0;
          break;     
        }
      }
      //system pause
      ///**
      if(convVars.pause){
        conveyorSetState(CONVEYOR_OFF);  
        while(convVars.pause){
          displayInfo( lastSentMsg, emCounter,dropCounter,pickUpCounter,  convVars);
        }
      }
      //**/
      //new buttons
      convVars=readBtns(convVars);
      
      //coveyer recieves drop request
      
      //read sensors
      convVars=readSensors(convVars);
      
      
      /** moved
      if(msg.id==REQ_DROP_CONV && !convVars.stopConv){
      msg.id=0;
      msgGlobal.id=0;       
      //sends ACK saying it is ok to drop
      if(!convVars.convS1){
      convVars.stopConv=76;
      lastSentMsg=sendMes( ACK_DROP_CONV);
      convVars.successDrop=0;
    }else{
      lastSentMsg=sendMes(NACK_DROP_CONV);
    }
    } 
      
      if(msg.id==CHK_CONV_PICKUP){
      if(!convVars.blockEndWaiting){
      lastSentMsg=sendMes( ACK_CHK_CONV_PICKUP);     
      convVars.rob2ACK=0;
      convVars.test2=83;
    }else{
      lastSentMsg=sendMes( NACK_CHK_CONV_PICKUP);       
      convVars.test2=69;
    }
    }
      **/// moved
      
      if(convVars.stopConv){
        conveyorSetState(CONVEYOR_OFF);
      }
      //pickUp counter
      if(convVars.blockEndWaiting && !convVars.convS2){     
        pickUpCounter=countUp(pickUpCounter);
        pickUpCounter.blockTimeOut1+=pickUpCounter.y3;
      }else{
        pickUpCounter.blockTimeOut1=0;
        pickUpCounter=countUp(pickUpCounter);
      }
      pickUpCounter.blockTimeOut2+=pickUpCounter.y3;
      
      
      
      //successful drop counter
      if (convVars.stopConv && convVars.convS1){
        dropCounter=countUp(dropCounter);
        dropCounter.blockTimeOut1+=dropCounter.y3;
      }else{
        dropCounter.blockTimeOut1=0;
        dropCounter=countUp(dropCounter);
      }
      dropCounter.blockTimeOut2+=dropCounter.y3;
      
      //in the event of a succesful drop
      if(convVars.stopConv && convVars.convS1 && (dropCounter.blockTimeOut1>dropDelay)){
        dropCounter.blockTimeOut1=0;
        convVars=successfulDrop(convVars);     
        //normal running with conveyer on
      }else if(!convVars.blockEndWaiting && !convVars.stopConv && convVars.blockCount && !convVars.convS2){
        if(conveyorGetState() !=CONVEYOR_REVERSE){
          conveyorSetState(CONVEYOR_REVERSE);
        }
        //when a block reaches the end
      }else if(convVars.blockCount  && convVars.convS2){//&& !convVars.blockEndWaiting
        conveyorSetState(CONVEYOR_OFF);
        convVars.blockEndWaiting=6;
        convVars.test4++;
        
        if(!convVars.rob2ACK){
          lastSentMsg=sendMes(REQ_PICKUP_CONV);
        }
        // \/this is when a block is removed\/
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
      displayInfo( lastSentMsg,emCounter,dropCounter,pickUpCounter,  convVars);
      //OSTimeDly(50);
    }
    displayInfo( lastSentMsg, emCounter,dropCounter,pickUpCounter,  convVars);
  } 
}
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



static void displayInfo(uint32_t lastSentMsg,struct Counter emCounter,struct Counter dropCounter,struct Counter pickUpCounter,struct ConvVars convVars){
  
  ///**
  static uint32_t canID;

  //conveyer display
  // uint8_t error;
  
  OSSemPend(LCDsem, 0, &error);
  lcdSetTextPos(1, 0);
  lcdWrite("ConvS 1|2  :%02u|%02u     ",convVars.convS1,convVars.convS2);
  lcdSetTextPos(1, 1);
  lcdWrite("blckNo:%2u stoConv:%2u    ",convVars.blockCount,convVars.stopConv);
  lcdSetTextPos(1, 2);
  lcdWrite("CanIn/Out:%02u|%02u       ",convVars.lastRecieved,lastSentMsg);
  lcdSetTextPos(1, 3);
  lcdWrite("start      :%3u           ",convVars.start);
  
  //lcdSetTextPos(1, 3);
  //lcdWrite("DrPu:%05u|%05u        ",dropCounter.blockTimeOut2,pickUpCounter.blockTimeOut2);
  
  lcdSetTextPos(1, 4); 
  lcdWrite("emC/CV/TO:%2u|%2u|%05d   ",convVars.emStop,convVars.emStopConv,emCounter.blockTimeOut1);  
  lcdSetTextPos(1, 5);
  lcdWrite("s-Drp/Pck:%02u|%02u      ",convVars.successDrop,convVars.successPick); 
  lcdSetTextPos(1, 6); 
  lcdWrite("Rob2 ACK :%3u           ",convVars.rob2ACK);  
  lcdSetTextPos(1, 7); 
  lcdWrite("blkNdWait:%3u           ",convVars.blockEndWaiting);  
  lcdSetTextPos(1, 8); 
  lcdWrite("Delay D/P:%05u|%05u      ",dropCounter.blockTimeOut1,pickUpCounter.blockTimeOut1); 
  lcdSetTextPos(1, 9); 
  lcdWrite("|| / |>  :%02u|%02u     ",convVars.pause,convVars.resume); 
  lcdSetTextPos(1, 10); 
  lcdWrite("test :%2u~%2u~%2u~%2u~%2u~ ",convVars.test0,convVars.test1,convVars.test2,convVars.test3,convVars.test4); 
  //  **/
  error = OSSemPost(LCDsem);
  
}
static struct ConvVars emConveyerTimeout(struct ConvVars convVars){
  convVars.emStopConv=81;
  convVars.emStop=82;
  convVars.start=0;
  convVars.blockCount=0;
  convVars.stopConv=0;
  convVars.blockEndWaiting=0;   
  return convVars;
}
static struct ConvVars emController(struct ConvVars convVars){
  convVars.emStop=89;
  convVars.start=0;
  convVars.blockCount=0;
  convVars.stopConv=0;
  convVars.blockEndWaiting=0;
  return convVars;
}

static struct ConvVars resetSystem(struct ConvVars convVars){
  convVars.start=44; 
  convVars.emStop=0;
  return convVars;
}
static struct ConvVars startSystem(struct ConvVars convVars){
  convVars.start=48;
  convVars.emStop=0;
  return convVars;
}
static struct ConvVars successfulDrop(struct ConvVars convVars){
  //OSTimeDly(500);//2500
  convVars.blockCount++;
  convVars.stopConv=0;
  convVars.successDrop=91;
  return convVars;
}
static struct Counter countUp(struct Counter counter){
  counter.y1=OSTimeGet();
  counter.y3=counter.y1-counter.y2;
  counter.y2=counter.y1;
  return counter;
}
static struct ConvVars readBtns(struct ConvVars convVars){
  if(btnBool.jsLTPressed==2){
    convVars.stopConv=77;
  }else if(btnBool.jsRTPressed==2){
    convVars.stopConv=77;
  }else if(btnBool.jsUpPressed==2){
    convVars.stopConv=77;
  }else if(btnBool.jsDownPressed==2){
    convVars.stopConv=77;
  }else if(btnBool.jsCentPressed==2){
    convVars.stopConv=77;
  }else if(btnBool.but1Pressed==2){//left btn
    convVars.blockCount=0;        
  }else if(btnBool.but2Pressed==2){//right btn
  } 
  return convVars;
}

/**
static struct ConvVars copyPasteMe(struct ConvVars convVars){

return convVars;
}
**/

static uint32_t sendMes(uint32_t mesID){
  canMessage_t msgOut;
  msgOut.id=mesID;
  canWrite(CAN_PORT_1, &msgOut);
  return mesID;
}
static void canHandler(void) {
  if (canReady(CAN_PORT_1)) {
    canRead(CAN_PORT_1, &can1RxBuf);
    OSSemPost(can1RxSem);
  }
}
uint32_t readWatch(void){
  uint32_t counter = 0;
  counter = T1TC;  // get the value of the timer counter
  return counter;  // return the value of the timer
}