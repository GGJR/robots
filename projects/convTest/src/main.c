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

/*************************************************************************
*                  PRIORITIES
*************************************************************************/

enum {
  APP_TASK_CTRL_CONV_PRIO = 4,
  APP_TASK_MONITOR_SENS_PRIO
    
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/

enum {
  EM_STOP = 0,
  EM_STOP_ACK_CONV,
  EM_STOP_ACK_ROB1,
  EM_STOP_ACK_ROB2,
  ERR_CONV,
  ERR_ROB1,
  ERR_ROB2,
  PAUSE,
  PAUSE_ACK_CONV,
  PAUSE_ACK_ROB1,
  PAUSE_ACK_ROB2,
  RESUME,
  RESUME_ACK_CONV,
  RESUME_ACK_ROB1,
  RESUME_ACK_ROB2,
  START,
  START_ACK_CONV,
  START_ACK_ROB1,
  START_ACK_ROB2,
  CTRL_STOP,
  CTRL_STOP_ACK_CONV,
  CTRL_STOP_ACK_ROB1,
  CTRL_STOP_ACK_ROB2,
  RESET,
  RESET_ACK_CONV,
  RESET_ACK_ROB1,
  RESET_ACK_ROB2,
  REQ_PICKUP_PAD1,
  ACK_PICKUP_PAD1,
  CHK_PAD1_PICKUP,
  ACK_CHK_PAD1_PICKUP,
  NACK_CHK_PAD1_PICKUP,
  REQ_DROP_CONV,
  ACK_DROP_CONV,
  NACK_DROP_CONV,
  REQ_PICKUP_CONV,
  ACK_PICKUP_CONV,
  CHK_CONV_PICKUP,
  ACK_CHK_CONV_PICKUP,
  NACK_CHK_CONV_PICKUP,
  REQ_DROP_PAD2,
  ACK_DROP_PAD2,
  NACK_DROP_PAD2
};


enum {
  APP_TASK_MONITOR_SENS_STK_SIZE = 256,
  APP_TASK_CTRL_CONV_STK_SIZE = 256
};

static OS_STK appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE];
static OS_STK appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE];

/*************************************************************************
*                  APPLICATION FUNCTION PROTOTYPES
*************************************************************************/

static void appTaskMonitorSens(void *pdata);
static void appTaskCtrlConv(void *pdata);

uint32_t readWatch(void);                 //Reads the watch and returns it's value
static void canHandler(void);
static void displayInfo(canMessage_t mes,uint8_t blockCount,uint8_t stopConv,uint8_t convEnd,uint8_t start,uint8_t emStop,INT32U blockTimeOut,INT32U blockTimeOut2);

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

static OS_EVENT *can1RxSem;
static canMessage_t can1RxBuf;

int main() {
  /* Initialise the hardware */
  bspInit();
  conveyorInit();
  
  /* Initialise the OS */
  OSInit();                                                   
  
  /* Create Tasks */
  OSTaskCreate(appTaskMonitorSens,                               
               (void *)0,
               (OS_STK *)&appTaskMonitorSensStk[APP_TASK_MONITOR_SENS_STK_SIZE - 1],
               APP_TASK_MONITOR_SENS_PRIO);
  
  OSTaskCreate(appTaskCtrlConv,                               
               (void *)0,
               (OS_STK *)&appTaskCtrlConvStk[APP_TASK_CTRL_CONV_STK_SIZE - 1],
               APP_TASK_CTRL_CONV_PRIO);
  
  /* Start the OS */
  //can1RxSem = OSSemCreate(0);//Event-Driven
  initWatch();
  startWatch();
  OSStart();                                                  
  
  /* Should never arrive here */ 
  return 0;      
}

/*************************************************************************
*                   APPLICATION TASK DEFINITIONS
*************************************************************************/

static void appTaskMonitorSens(void *pdata) {
  
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  
  /* 
  * Now execute the main task loop for this task
  */
  while (true) {
    interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
    ledSetState(USB_LINK_LED, LED_OFF);
    ledSetState(USB_CONNECT_LED, LED_OFF);
    
    if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {
      interfaceLedSetState(D1_LED, LED_ON);
      ledSetState(USB_LINK_LED, LED_ON);
    } 
    if (conveyorItemPresent(CONVEYOR_SENSOR_2)) {
      interfaceLedSetState(D2_LED, LED_ON);
      ledSetState(USB_CONNECT_LED, LED_ON);
    } 
    
    OSTimeDly(20);
  }
}
uint32_t y1=0;
uint32_t y2=0;
uint32_t y3=0;
uint32_t z1=0;
uint32_t z2=0;
uint32_t z3=0;
static void appTaskCtrlConv(void *pdata) {
  //CAN
  //read
  uint8_t error;
  canMessage_t msg;
  canRxInterrupt(canHandler);//Event-Driven
  
  //write
  canMessage_t msgOut;
  bool txOk = false;
  
  INT32U blockTimeOut1=0;
  INT32U blockTimeOut2=0;
  
  osStartTick();
  
  //testing vars
  uint32_t x1=98;
  uint32_t x2=99;
  
  uint32_t btnState;
  uint8_t blockCount=0;
  
  uint8_t stopConv=0;
  uint8_t convEnd=0;
  uint8_t start=0;
  uint8_t emStop=0;
  while (true) {
    ///**
    while(msg.id!=START&&(msg.id!=RESET)){
    OSSemPend(can1RxSem, 0, &error);
    msg = can1RxBuf;
    displayInfo( msg, blockCount, stopConv, convEnd, start, emStop,blockTimeOut1, blockTimeOut2);
  }
    //**/
    start=41;
    if(emStop){
      start=44;
      emStop=0;
      msgOut.id=RESET_ACK_CONV;
      msgOut.dataA=blockCount;
      txOk = canWrite(CAN_PORT_1, &msgOut);
    }else{
      start=48;
      emStop=0;
      msgOut.id=START_ACK_CONV;
      msgOut.dataA=blockCount;
      txOk = canWrite(CAN_PORT_1, &msgOut);
    }
    while (true) {
      ///**  //Event-Driven
      OSSemPend(can1RxSem, 0, &error);
      msg = can1RxBuf;
      
      ///**
      if(msg.id==EM_STOP){
        msgOut.id=EM_STOP_ACK_CONV;
        msgOut.dataA=blockCount;
        txOk = canWrite(CAN_PORT_1, &msgOut);
        // x1=98;
        // x2=99;
        emStop=89;
        start=0;
        blockCount=0;
        conveyorSetState(CONVEYOR_OFF);    
        
        stopConv=0;
        convEnd=0;
        break;
      }
      //**/
      
      INT32U timeX1 = OSTimeGet();
      INT32U timeX2 = OSTimeGet();
      
      
      uint32_t timeOut=10000;
      if(blockCount){
        y1=OSTimeGet();
        //y1=readWatch();  
        y3=y1-y2;
        y2=y1;
        blockTimeOut1+=y3;
        if(blockTimeOut1>timeOut){
          msgOut.id=ERR_CONV;
          msgOut.dataA=blockCount;
          txOk = canWrite(CAN_PORT_1, &msgOut);
          emStop=82;
          start=0;
          blockCount=0;
          conveyorSetState(CONVEYOR_OFF);    
          
          stopConv=0;
          convEnd=0;         
          
        }
      }else{
        blockTimeOut1=0;
      }
      y1=OSTimeGet();
      //y1=readWatch();  
      y3=y1-y2;
      y2=y1;
      blockTimeOut2+=y3;      
      
      
      static bool jsRTPressed = false;// right
      static bool jsLTPressed = false;// left
      static bool jsUpPressed = false;// up
      static bool jsDownPressed = false;// down
      static bool jsCentPressed = false;// centre
      
      static bool but1Pressed = false;// but1
      static bool but2Pressed = false;// but2
      
      //static bool sensor1 = false;
      //static bool sensor2 = false;
      
      
      
      btnState = buttonsRead();
      
      if (isButtonPressedInState(btnState, JS_LEFT)) {
        jsLTPressed = true;
      }else if (isButtonPressedInState(btnState, JS_RIGHT)) {
        jsRTPressed = true;
      }else if (isButtonPressedInState(btnState, JS_UP)) {
        jsUpPressed = true;
      }else if (isButtonPressedInState(btnState, JS_DOWN)) {
        jsDownPressed = true;
      }else if (isButtonPressedInState(btnState, JS_CENTRE)) {
        jsCentPressed = true;
      }else if (isButtonPressedInState(btnState, BUT_1)) {
        but1Pressed = true;
      }else if (isButtonPressedInState(btnState, BUT_2)) {
        but2Pressed = true;
      }
      
      
      //joystick
      /**
      **/ 
      if (jsLTPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
        jsLTPressed = false;
        stopConv=77; 
      }else if (jsRTPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
        jsRTPressed = false;
        stopConv=77;
      }else if (jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
        jsUpPressed = false;
        stopConv=77;
      }else if (jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
        jsDownPressed = false;
        stopConv=77;
      }else if (jsCentPressed && (!isButtonPressedInState(btnState, JS_CENTRE))) {
        jsCentPressed = false;
        stopConv=77;
        
      }else if (but1Pressed && (!isButtonPressedInState(btnState, BUT_1))) {//left button
        but1Pressed = false;
        
      }else if (but2Pressed && (!isButtonPressedInState(btnState, BUT_2))) {//right button
        but2Pressed = false;
        conveyorSetState(CONVEYOR_OFF);    
        blockCount=0;                      
      }
      
      //coveyer recieves drop request
      if(msg.id==REQ_DROP_CONV){
        stopConv=76;
        msgOut.id=START_ACK_CONV;
        msgOut.dataA=blockCount;
        txOk = canWrite(CAN_PORT_1, &msgOut);
      }
      
      if(stopConv){
        x1=11;
        conveyorSetState(CONVEYOR_OFF);
      }
      //This is for when a block is dropped on the conveyer
      if (stopConv && conveyorItemPresent(CONVEYOR_SENSOR_2) && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=21;
        OSTimeDly(1500);
        conveyorSetState(CONVEYOR_REVERSE);
        OSTimeDly(10);
        blockCount++;
        stopConv=0;
        //normal running with conveyer on
      }else if(!convEnd && !stopConv && blockCount && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=23;
        if(conveyorGetState() !=CONVEYOR_REVERSE){
          conveyorSetState(CONVEYOR_REVERSE);
        }
        //when a block reaches the end
      }else if(!convEnd && conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=24;
        conveyorSetState(CONVEYOR_OFF);
        //notify robot 2 for pickup
        convEnd=66;
        msgOut.id=REQ_PICKUP_CONV;
        msgOut.dataA=blockCount;
        txOk = canWrite(CAN_PORT_1, &msgOut);
        //this is when a block is removed
      }else if(convEnd && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=25;
        //successful removal
        //recieve ACK
        convEnd=0;
        if(blockCount>0){
          blockCount--;
        }    
        msgOut.id=ACK_CHK_CONV_PICKUP;
        msgOut.dataA=blockCount;
        txOk = canWrite(CAN_PORT_1, &msgOut);
        OSTimeDly(1000);
      }
      
      displayInfo( msg, blockCount, stopConv, convEnd, start, emStop,blockTimeOut1, blockTimeOut2);
    }
    displayInfo( msg, blockCount, stopConv, convEnd, start, emStop,blockTimeOut1, blockTimeOut2);
  } 
}

static void displayInfo(canMessage_t mes,uint8_t blockCount,
                        uint8_t stopConv,uint8_t convEnd,uint8_t start,
                        uint8_t emStop,INT32U blockTimeOut1,
                        INT32U blockTimeOut2
                          ){
                            ///**
                            static uint32_t xxx=4;
                            xxx=mes.id;
                            //conveyer display
                            lcdSetTextPos(2, 1);
                            lcdWrite("blockCount : %4u",blockCount);
                            lcdSetTextPos(2, 2);
                            lcdWrite("stopConv : %4u",stopConv);
                            //lcdSetTextPos(2, 3);
                            //lcdWrite("convEnd : %4u",convEnd);
                            lcdSetTextPos(2, 4);
                            lcdWrite("CAN.ID %4u          ",xxx);
                            lcdSetTextPos(2, 5);
                            lcdWrite("start %4u          ",start);
                            lcdSetTextPos(2, 6);
                            lcdWrite("emStop %4u          ",emStop);  
                            lcdSetTextPos(2, 7);
                            lcdWrite("blkTOut %10d          ",blockTimeOut1);  
                            //lcdSetTextPos(2, 8);
                            //lcdWrite("blkTOut %10d          ",blockTimeOut2);  
                            //  **/
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