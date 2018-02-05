#ifndef __BSP_H
#define __BSP_H

#include <stdint.h>

typedef void (* pVoidFunc_t)(void);           // type definition of pointer to void function

void mamInit(void);                           // initialise memory accelerator module
void clockInit(void);                         // initialise system and peripheral clocks
uint32_t getFsclk(void);                      // get the system clock frequency
uint32_t getFpclk(uint32_t peripheral);       // get a peripheral clock frequency
void gpioInit(void);                          // initialise pins to default function - GPIO, input
void vicInit(void);                           // initialise VIC
void vicInstallIRQhandler(                    // install VIC interrupt handler
  pVoidFunc_t handler,                          // pointer to handler function
  uint32_t priority,                            // interrupt priority level (0..15)
  uint32_t vicIntSource);                       // interrupt source (0..31)
void lowLevelInit(void);                      // call low-level initialisation functions
void bspInit(void);                           // call all board support package initialisation functions

#endif
