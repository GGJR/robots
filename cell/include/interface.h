#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <stdint.h>
#include <leds.h>

#define D1_LED 0x00000080
#define D2_LED 0x00000040
#define D3_LED 0x00000020
#define D4_LED 0x00000010


typedef enum interface {NO_DEVICE, CONTROL, CONVEYOR, ROBOT} interface_t;

void interfaceInit(interface_t);
void interfaceLedSetState(uint32_t, ledState_t);
void interfaceLedToggle(uint32_t);

#endif

