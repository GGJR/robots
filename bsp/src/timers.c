#include <iolpc2378.h>
#include <board.h>
#include <bsp.h>
#include <timers.h>

static pVoidFunc_t userFuncInISR0;
static pVoidFunc_t userFuncInISR1;
static void timer0ISR(void);
static void timer1ISR(void);

void initTimer(timerIdentifier_t timer, pVoidFunc_t handler, uint32_t ticksPerSec) {
  if (timer == TIMER0) {
    T0TCR = 0x02;  // reset timer
    T0PR = 0x00;   // set prescaler to 0
    T0CTCR = 0x00; // set mode: every rising PCLK edge
    T0MR0 = getFpclk(TIMER0_PCLK_OFFSET) / ticksPerSec; 
    T0IR = 0xff;   // reset all interrupts
    T0MCR = 0x03;  // enable interrupt and reset on match
    userFuncInISR0 = handler;
    vicInstallIRQhandler(timer0ISR, 0, VIC_TIMER0);
  } else if (timer == TIMER1) {
    T1TCR = 0x02;  // reset timer
    T1PR = 0x00;   // set prescaler to 0
    T1CTCR = 0x00; // set mode: every rising PCLK edge
    T1MR0 = getFpclk(TIMER1_PCLK_OFFSET) / ticksPerSec; 
    T1IR = 0xff;   // reset all interrupts
    T1MCR = 0x03;  // enable interrupt and reset on match
    userFuncInISR1 = handler;
    vicInstallIRQhandler(timer1ISR, 0, VIC_TIMER1);
  }   
}


void startTimer(timerIdentifier_t timer) {
  if (timer == TIMER0) {
    T0TCR = 0x01;  // start timer 0
  } else if (timer == TIMER1) {
    T1TCR = 0x01;  // start timer 1
  }
}

void stopTimer(timerIdentifier_t timer) {
  if (timer == TIMER0) {
    T0TCR = 0x00;  // stop timer 0
  } else if (timer == TIMER1) {
    T1TCR = 0x00;  // stop timer 1
  }
}

void initWatch(void) {
  T1TCR = 0x02;  // reset timer
  T1PR = 0x00;   // set prescaler to 0
  T1CTCR = 0x00; // set mode: every rising PCLK edge
  T1MR0 = 0x00;  // not interested in match
  T1IR = 0xff;   // reset all interrupts
  T1MCR = 0x00;  // not interested in match
}


void startWatch(void) {
  T1TCR = 0x01;  // start timer 1
}

uint32_t stopWatch(void) {
  uint32_t counter = 0;
  
  T1TCR = 0x00;    // stop the timer 
  counter = T1TC;  // get the value of the timer counter
  T1TCR = 0x02;    // reset the timer
  return counter;
}

static void timer0ISR(void) {
  userFuncInISR0(); // Call user-installed handler
  T0IR = 1;         // Clear the interrupt on MR0; writing 0 to the other bits is safe
}

static void timer1ISR(void) {
  userFuncInISR1(); // Call user-installed handler
  T1IR = 1;         // Clear the interrupt on MR0; writing 0 to the other bits is safe
}

