#include <stdint.h>
#include <iolpc2378.h>
#include <pwm.h>
#include <interface.h>
#include <control.h>

#define EMERGENCY_STOP_BUTTON  0x00000008

static controlAlarmState_t state = CONTROL_ALARM_OFF;

void controlInit(void) {
  interfaceInit(CONTROL);
  pwmInit(440);             // major cycle 440Hz
  pwmChannelInit(PWM3, 0);  // duty cycle 0% on channel 3 
  controlAlarmSetState(CONTROL_ALARM_OFF);
}


void controlAlarmSetState(controlAlarmState_t newState) {
  if (newState == CONTROL_ALARM_ON) {
    pwmChangeDutyCycle(PWM3, 500000); // duty cycle 50% on channel 3
  } else {
    pwmChangeDutyCycle(PWM3, 0);      // duty cycle 0% on channel 3
  }
  state = newState;
}


controlAlarmState_t controlAlarmGetState(void) {
  return state;
}


void controlAlarmToggleState(void) {
  if (state == CONTROL_ALARM_OFF) {
    controlAlarmSetState(CONTROL_ALARM_ON);
  } else {
    controlAlarmSetState(CONTROL_ALARM_OFF);
  }
}


bool controlEmergencyStopButtonPressed(void) {
  return (FIO2PIN & EMERGENCY_STOP_BUTTON ? false : true); 
}


bool controlItemPresent(uint32_t sensor) {
  return (FIO2PIN & sensor ? false : true);
}
