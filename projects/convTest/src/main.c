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

/*************************************************************************
*                    GLOBAL FUNCTION DEFINITIONS
*************************************************************************/

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
  osStartTick();
  
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

static void appTaskCtrlConv(void *pdata) {
    osStartTick();
    
    //testing vars
    uint32_t x1=98;
    uint32_t x2=99;
    
  uint32_t btnState;
  uint8_t blockCount=0;
  
  uint8_t stopConv=0;
  uint8_t convEnd=0;
  while (true) {
    
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
    if (jsLTPressed && (!isButtonPressedInState(btnState, JS_LEFT))) {
            jsLTPressed = false;
                  
    }else if (jsRTPressed && (!isButtonPressedInState(btnState, JS_RIGHT))) {
            jsRTPressed = false;

    }else if (jsUpPressed && (!isButtonPressedInState(btnState, JS_UP))) {
            jsUpPressed = false;
                        conveyorSetState(CONVEYOR_OFF);

    }else if (jsDownPressed && (!isButtonPressedInState(btnState, JS_DOWN))) {
            jsDownPressed = false;
                        conveyorSetState(CONVEYOR_OFF);

    }else if (jsCentPressed && (!isButtonPressedInState(btnState, JS_CENTRE))) {
            jsCentPressed = false;
            conveyorSetState(CONVEYOR_OFF);    
            blockCount=0;
            
    }else if (but1Pressed && (!isButtonPressedInState(btnState, BUT_1))) {//left button
            but1Pressed = false;
            stopConv=77;
            /**
            if(emergencyStop){
              conveyorSetState(CONVEYOR_OFF);
              //send can ACK
            }
            if(pause){
              conveyorSetState(CONVEYOR_OFF);
              //send can ACK
              }
            if(resume){
              resume();
              //send can ACK
              }
            if(start){
              start();
              //send can ACK
              }
            **/        
    }else if (but2Pressed && (!isButtonPressedInState(btnState, BUT_2))) {//right button
            but2Pressed = false;
          

    }

    //CONVEYOR_FORWARD
    //CONVEYOR_REVERSE
    if(stopConv){
           x1=11;
          conveyorSetState(CONVEYOR_OFF);
    }
    if (stopConv && conveyorItemPresent(CONVEYOR_SENSOR_2) && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
      x2=21;
      OSTimeDly(1000);
      conveyorSetState(CONVEYOR_REVERSE);
      OSTimeDly(1000);
      blockCount++;
      stopConv=0;
    }else if(!convEnd && !stopConv && blockCount && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
      x2=23;
      //makes the conveyer more responsive after removals
      if(conveyorGetState() !=CONVEYOR_REVERSE){
        conveyorSetState(CONVEYOR_REVERSE);
      }
      //normal running
    }else if(!convEnd && conveyorItemPresent(CONVEYOR_SENSOR_1)){
      x2=24;
      conveyorSetState(CONVEYOR_OFF);
      //notify robot 2 for pickup
      convEnd=66;
    }else if(convEnd && !conveyorItemPresent(CONVEYOR_SENSOR_1)){
      x2=25;
      //successful removal
      //recieve ACK
      convEnd=0;
      if(blockCount>0){
        blockCount--;
      }    
      OSTimeDly(1000);
    }
    



    
    //conveyer display
    lcdSetTextPos(2, 1);
    lcdWrite("blockCount : %4u",blockCount);
    lcdSetTextPos(2, 2);
    lcdWrite("stopConv : %4u",stopConv);
    lcdSetTextPos(2, 3);
    lcdWrite("convEnd : %4u",convEnd);
    lcdSetTextPos(2, 4);
    lcdWrite("X1 %4u",x1);
    lcdSetTextPos(2, 5);
    lcdWrite("X2 %4u",x2);
    lcdSetTextPos(2, 6);
    lcdSetTextPos(2, 7);
    lcdSetTextPos(2, 8);
    
    
    
  } 
}

