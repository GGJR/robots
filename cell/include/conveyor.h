#ifndef __CONVEYOR_H
#define __CONVEYOR_H

#include <stdint.h>
#include <stdbool.h>

#define CONVEYOR_SENSOR_1  0x00000008
#define CONVEYOR_SENSOR_2  0x00000004

typedef enum conveyorState {CONVEYOR_OFF, CONVEYOR_FORWARD, CONVEYOR_REVERSE} conveyorState_t;

void conveyorInit(void);
void conveyorSetState(conveyorState_t);
conveyorState_t conveyorGetState(void);
bool conveyorItemPresent(uint32_t);

#endif