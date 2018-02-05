#include <stdint.h>
#include <stdbool.h>
#include <iolpc2378.h>
#include <interface.h>
#include <conveyor.h>

#define CONVEYOR_MOTOR     0x00000002
#define CONVEYOR_DIRECTION 0x00000001

static conveyorState_t state = CONVEYOR_OFF;

void conveyorInit(void) {
  interfaceInit(CONVEYOR);
  conveyorSetState(CONVEYOR_OFF);
}

conveyorState_t conveyorGetState() {
  return state;
}

void conveyorSetState(conveyorState_t newState) {
  switch (newState) {
  case CONVEYOR_OFF:
    FIO2CLR |= CONVEYOR_MOTOR;
    break;
  case CONVEYOR_FORWARD:
    FIO2SET |= CONVEYOR_MOTOR; 
    FIO2CLR |= CONVEYOR_DIRECTION;
    break;
  case CONVEYOR_REVERSE:
    FIO2SET |= CONVEYOR_MOTOR;
    FIO2SET |= CONVEYOR_DIRECTION; 
    break;
  default:
    break;
  }
  state = newState;
}

bool conveyorItemPresent(uint32_t sensor) {
  return (FIO2PIN & sensor ? false : true);
}


