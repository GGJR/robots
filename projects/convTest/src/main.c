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
Date-15/03/18


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
  APP_TASK_CAN_READ_PRIO = 4,
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
  uint8_t emStop;//in an emergency stop
  uint8_t blockCount;//the number of blocks on the conveyer
  uint32_t rob2ACK;//
  uint8_t blockEndWaiting;
  uint8_t successDrop;
};//9 members
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
static void displayInfo(canMessage_t mes,uint32_t lastSentMsg,struct Counter counter,struct ConvVars convVars);

static uint32_t sendMes(uint8_t blockCount,uint32_t mesID);
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


/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

static OS_EVENT *can1RxSem;
static OS_EVENT *LCDsem;
INT8U error;

static canMessage_t can1RxBuf;
static struct ButtonBool btnBool={0,0,0,0,0,0,0};

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
canMessage_t msgGlobal;
static void appTaskCanRead(void *pdata) {
  uint8_t error;
  canRxInterrupt(canHandler);//Event-Driven
  
  /* Start the OS ticker
  * (must be done in the highest priority task)
  */
  osStartTick();
  
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
    
    ///**
    
    //**/
    OSTimeDly(50);
  }
}
//timing vars

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
  struct ConvVars convVars={0,0,0,0,0,0,0,0,0};
  struct Counter counter={0,0,0,0,0};
  //static struct ButtonBool btnBool={0,0,0,0,0,0,0};
  canMessage_t msg;
  uint32_t lastSentMsg=0;
  //canRxInterrupt(canHandler);//Event-Driven
  uint32_t timeOut=2500000;//conveyer timeout for emergency stop
  
  uint32_t btnState;
  while (true) {
    //starting loop & in emergency stop
    ///**
    while(msg.id!=START&&(msg.id!=RESET)){
      msg=msgGlobal;
      displayInfo( msg,lastSentMsg, counter,  convVars);
      OSTimeDly(200);
      
    }
    msgGlobal.id=0;
    convVars.start=41;//move back
    
    if(convVars.emStop){
      convVars=resetSystem(convVars);
      lastSentMsg=sendMes(convVars.blockCount, RESET_ACK_CONV);
    }else{
      convVars=startSystem(convVars);
      lastSentMsg=sendMes(convVars.blockCount, START_ACK_CONV);
    }
    
    // **/  
    //      convVars.start=41;//move back
    
    
    //main running loop
    while (true) {
      msg.id=0;
      msg=msgGlobal;
      //checks for emergency stop
      if(msg.id==EM_STOP){
        lastSentMsg=sendMes(convVars.blockCount, EM_STOP_ACK_CONV);
        conveyorSetState(CONVEYOR_OFF);    
        convVars=emController(convVars);
        break;
      }              
      //for the time out emergency stop
      if(convVars.blockCount&&!convVars.blockEndWaiting){
        counter=countUp(counter);
        counter.blockTimeOut1+=counter.y3;
        if(counter.blockTimeOut1>timeOut &&!convVars.stopConv ){
          lastSentMsg=sendMes(convVars.blockCount, ERR_CONV);
          convVars=emConveyerTimeout(convVars);
          conveyorSetState(CONVEYOR_OFF);    
        }
      }else{
        counter.blockTimeOut1=0;
        counter=countUp(counter);
      }
      counter.blockTimeOut2+=counter.y3;
      //breaks without
          
      //new buttons
    convVars=readBtns(convVars);

      //coveyer recieves drop request
      
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
      
      if(msg.id==REQ_DROP_CONV){
        convVars.stopConv=76;
        //sends ACK saying it is ok to drop
        if(!convVars.convS1){
            lastSentMsg=sendMes(convVars.blockCount, ACK_DROP_CONV);
        }else{
            lastSentMsg=sendMes(convVars.blockCount, NACK_DROP_CONV);
        }
      } 
      if(msg.id==ACK_PICKUP_CONV){
        convVars.rob2ACK=78;
      }  
      if(msg.id==CHK_CONV_PICKUP){
        if(!convVars.blockEndWaiting){
            lastSentMsg=sendMes(convVars.blockCount, ACK_CHK_CONV_PICKUP);       
        }else{
            lastSentMsg=sendMes(convVars.blockCount, NACK_CHK_CONV_PICKUP);       
        }
      }
      if(msg.id==CHK_CONV_DROP){
        if(!convVars.successDrop){
            lastSentMsg=sendMes(convVars.blockCount, ACK_CHK_CONV_DROP);  
            convVars.successDrop=0;
        }else{
            lastSentMsg=sendMes(convVars.blockCount, NACK_CHK_CONV_DROP);       
        }
      }
      
      if(convVars.stopConv){
        conveyorSetState(CONVEYOR_OFF);
      }
   
      
      
      //This is for when a block is dropped on the conveyer
      if (convVars.stopConv && convVars.convS1 ){//&& !convVars.convS2
        //sends ACK to robot informing successful drop
        //lastSentMsg=sendMes(convVars.blockCount, ACK_CHK_CONV_DROP);
        
        convVars=successfulDrop(convVars);
          msgGlobal.id=0;
        //normal running with conveyer on
      }else if(!convVars.blockEndWaiting && !convVars.stopConv && convVars.blockCount && !convVars.convS2){
        if(conveyorGetState() !=CONVEYOR_REVERSE){
          conveyorSetState(CONVEYOR_REVERSE);
        }
        //when a block reaches the end
      }else if(convVars.blockCount && !convVars.blockEndWaiting && convVars.convS2){
        conveyorSetState(CONVEYOR_OFF);
        convVars.blockEndWaiting=6;
        if(!convVars.rob2ACK){
          lastSentMsg=sendMes(convVars.blockCount, REQ_PICKUP_CONV);
        }
        // \/this is when a block is removed\/
      }else if(convVars.blockEndWaiting && !convVars.convS2){
        //successful removal
        //recieve ACK
        convVars.blockEndWaiting=0;
        if(convVars.blockCount>0){
          convVars.blockCount--;
        }    
        OSTimeDly(1000);
      }
      
      displayInfo( msg,lastSentMsg,counter,  convVars);
      OSTimeDly(10);
    }
    displayInfo( msg,lastSentMsg, counter,  convVars);
  } 
}

static void displayInfo(canMessage_t mes,uint32_t lastSentMsg,struct Counter counter,struct ConvVars convVars){
  
  ///**
  static uint32_t xxx=4;
  xxx=mes.id;
  //conveyer display
  // uint8_t error;
  
  OSSemPend(LCDsem, 0, &error);
  lcdSetTextPos(1, 0);
  lcdWrite("blockCount : %6u           ",convVars.blockCount);
  lcdSetTextPos(1, 1);
  lcdWrite("stopConv   : %6u           ",convVars.stopConv);
  lcdSetTextPos(1, 2);
  lcdWrite("ConvS 1|2  : %5u|%u     ",convVars.convS1,convVars.convS2);
  lcdSetTextPos(1, 3);
  lcdWrite("CAN.ID     : %6u           ",xxx);
  lcdSetTextPos(1, 4);
  lcdWrite("start      : %6u           ",convVars.start);
  lcdSetTextPos(1, 5);
  lcdWrite("emStop     : %6u           ",convVars.emStop);  
  lcdSetTextPos(1, 6);
  lcdWrite("blkTOut    : %6d           ",counter.blockTimeOut1); 
  lcdSetTextPos(1, 7);
  lcdWrite("successDrop: %6d           ",convVars.successDrop); 
  lcdSetTextPos(1, 8); 
  lcdWrite("Rob2 ACK   : %6u           ",convVars.rob2ACK);  
  lcdSetTextPos(1, 9); 
  lcdWrite("lastSent   : %6u           ",lastSentMsg);  
  lcdSetTextPos(1, 10); 
  lcdWrite("blockEndW  : %6u           ",convVars.blockEndWaiting);  
  //  **/
  error = OSSemPost(LCDsem);
  
}
static struct ConvVars emConveyerTimeout(struct ConvVars convVars){
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
  OSTimeDly(500);//2500
  //conveyorSetState(CONVEYOR_REVERSE);
  //OSTimeDly(10);
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
static struct ConvVars emConveyerTimeout(struct ConvVars convVars){

return convVars;
}
**/
static uint32_t sendMes(  uint8_t blockCount,uint32_t mesID){
  canMessage_t msgOut;
  bool txOk = false;
  msgOut.id=mesID;
  msgOut.dataA=blockCount;
  txOk = canWrite(CAN_PORT_1, &msgOut);
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