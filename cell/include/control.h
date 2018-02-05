#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_SENSOR_1 0x00000001
#define CONTROL_SENSOR_2 0x00000002

typedef enum controlAlarmState {CONTROL_ALARM_OFF, CONTROL_ALARM_ON} controlAlarmState_t;

void controlInit(void);
void controlAlarmSetState(controlAlarmState_t);
controlAlarmState_t controlAlarmGetState(void);
void controlAlarmToggleState(void);
bool controlEmergencyStopButtonPressed(void);
bool controlItemPresent(uint32_t);


#endif