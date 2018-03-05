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
Date-05/03/18


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
  APP_TASK_MONITOR_SENS_PRIO = 4,
  APP_TASK_CTRL_CONV_PRIO
    
    
};

/*************************************************************************
*                  APPLICATION TASK STACKS
*************************************************************************/



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
static void displayInfo1(canMessage_t mes,uint8_t blockCount,uint8_t stopConv,uint8_t convEnd,uint8_t start,uint8_t emStop,INT32U blockTimeOut,INT32U blockTimeOut2);

static void sendMes(  uint8_t blockCount,uint32_t mesID);


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
canMessage_t msgGlobal;
static void appTaskMonitorSens(void *pdata) {
  uint8_t error;
  canRxInterrupt(canHandler);//Event-Driven
  
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  
  /* 
  * Now execute the main task loop for this task
  */
  while (true) {
    interfaceLedSetState(D1_LED | D2_LED | D3_LED | D4_LED, LED_OFF);
    //ledSetState(USB_LINK_LED, LED_OFF);
    //ledSetState(USB_CONNECT_LED, LED_OFF);
    //ledSetState(USB_LINK_LED, LED_ON);    
    //ledSetState(USB_CONNECT_LED, LED_ON);    
    if (conveyorItemPresent(CONVEYOR_SENSOR_1)) {//back convyer
      interfaceLedSetState(D1_LED, LED_ON);
      ledToggle(USB_CONNECT_LED);
    } else{
      ledSetState(USB_CONNECT_LED, LED_OFF);
    }
    if (conveyorItemPresent(CONVEYOR_SENSOR_2)) {//front conveyer
      interfaceLedSetState(D2_LED, LED_ON);
      ledToggle(USB_LINK_LED);
    } else{
      ledSetState(USB_LINK_LED, LED_OFF);
    }
    OSSemPend(can1RxSem, 0, &error);
    msgGlobal = can1RxBuf;
    OSTimeDly(20);
  }
}
//timing vars
uint32_t y1=0;
uint32_t y2=0;
uint32_t y3=0;


static void appTaskCtrlConv(void *pdata) {
  //CAN
  //read
  uint8_t error;
  canMessage_t msg;
  //canRxInterrupt(canHandler);//Event-Driven
  
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
  //
  while (true) {
    //starting loop & in emergency stop
    while(msg.id!=START&&(msg.id!=RESET)){
      msg=msgGlobal;
      displayInfo( msg, blockCount, stopConv, convEnd, start, emStop,blockTimeOut1, blockTimeOut2);
    }
    start=41;
    if(emStop){
      start=44;
      emStop=0;
      sendMes(blockCount, RESET_ACK_CONV);
    }else{
      start=48;
      emStop=0;
      sendMes(blockCount, START_ACK_CONV);
    }
    //main running loop
    while (true) {
      msg.id=0;
      msg=msgGlobal;
      
      //checks for emergency stop
      if(msg.id==EM_STOP){
        sendMes(blockCount, EM_STOP_ACK_CONV);
        emStop=89;
        start=0;
        blockCount=0;
        conveyorSetState(CONVEYOR_OFF);    
        stopConv=0;
        convEnd=0;
        break;
      }      
      //INT32U timeX1 = OSTimeGet();
      //INT32U timeX2 = OSTimeGet();
      
      
      uint32_t timeOut=1000000;
      
      //for the time out emergency stop
      if(blockCount&&!convEnd){
        y1=OSTimeGet();
        y3=y1-y2;
        y2=y1;
        blockTimeOut1+=y3;
        if(blockTimeOut1>timeOut &&!stopConv ){
          sendMes(blockCount, ERR_CONV);
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
      
      //joystick get state
      static bool jsRTPressed = false;// right
      static bool jsLTPressed = false;// left
      static bool jsUpPressed = false;// up
      static bool jsDownPressed = false;// down
      static bool jsCentPressed = false;// centre
      
      static bool but1Pressed = false;// but1
      static bool but2Pressed = false;// but2
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
      //joystick logic
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
        //sends ACK saying it is ok to drop
        sendMes(blockCount, ACK_DROP_CONV);
      }
      
      if(stopConv){
        x1=11;
        conveyorSetState(CONVEYOR_OFF);
      }
      //This is for when a block is dropped on the conveyer
      if (stopConv && conveyorItemPresent(CONVEYOR_SENSOR_2) && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=21;
        //sends ACK to robot informing successful drop
        sendMes(blockCount, ACK_CHK_CONV_DROP);
        OSTimeDly(2500);
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
      }else if(blockCount && !convEnd && conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=24;
        conveyorSetState(CONVEYOR_OFF);
        //notify robot 2 for pickup
        convEnd=66;
        sendMes(blockCount, REQ_PICKUP_CONV);
        
        // \/this is when a block is removed\/
      }else if(convEnd && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
        x2=25;
        //successful removal
        //recieve ACK
        convEnd=0;
        if(blockCount>0){
          blockCount--;
        }    
        sendMes(blockCount, ACK_CHK_CONV_PICKUP);
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
                            lcdSetTextPos(2, 3);
                            lcdWrite("convEnd : %4u",convEnd);
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

static void displayInfo1(canMessage_t mes,uint8_t blockCount,
                         uint8_t stopConv,uint8_t convEnd,uint8_t start,
                         uint8_t emStop,INT32U blockTimeOut1,
                         INT32U blockTimeOut2
                           ){
                             /**
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
static void sendMes(  uint8_t blockCount,uint32_t mesID){
  canMessage_t msgOut;
  bool txOk = false;
  msgOut.id=mesID;
  msgOut.dataA=blockCount;
  txOk = canWrite(CAN_PORT_1, &msgOut);
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