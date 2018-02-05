#include <assert.h>
#include <stdint.h>
#include <pwm.h>
#include <interface.h>
#include <robot.h>

/* Major cycle of 50Hz (20ms) 
 * Duty cycle ranges from ~1ms to ~2ms
 * i.e. ~5% to ~10% of major cycle 
 * 
 */
enum {PWM_MAJOR_CYCLE_HZ = 50};

/*
 * Constants below represent millionths of the major cycle,
 * e.g. HAND_MIN 45000 means the minimum value of the duty cycle
 * for the hand joint is 45000/1000000 of the major cycle time,
 * i.e. duty cycle time = (20 * (45000/1000000) = 0.9ms
 */
enum {
  DUMMY_MIN = 45000,
  HAND_MIN = 45000,
  WRIST_MIN = 45000,
  ELBOW_MIN = 45000,
  WAIST_MIN = 45000,
  DUMMY_MAX = 100000,
  HAND_MAX  = 100000,
  WRIST_MAX = 100000,
  ELBOW_MAX = 100000,
  WAIST_MAX = 100000,
  DUMMY_NEUTRAL = 72500,
  HAND_NEUTRAL = 68750,
  WRIST_NEUTRAL = 82250,
  ELBOW_NEUTRAL = 87500,
  WAIST_NEUTRAL = 67250,
};

static const uint32_t jointMinPos[5] = {DUMMY_MIN, HAND_MIN, WRIST_MIN, ELBOW_MIN, WAIST_MIN};
static const uint32_t jointMaxPos[5] = {DUMMY_MAX, HAND_MAX, WRIST_MAX, ELBOW_MAX, WAIST_MAX};
static const uint32_t jointStep = 250;

static uint32_t jointPos[5] = {DUMMY_NEUTRAL, HAND_NEUTRAL, WRIST_NEUTRAL, ELBOW_NEUTRAL, WAIST_NEUTRAL};

void robotInit(void) {
  interfaceInit(ROBOT);
  pwmInit(PWM_MAJOR_CYCLE_HZ);               // set major cycle (should be 20ms period)        
  pwmChannelInit((pwmIdentifier_t)ROBOT_HAND, HAND_NEUTRAL);  
  pwmChannelInit((pwmIdentifier_t)ROBOT_WRIST, WRIST_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_ELBOW, ELBOW_NEUTRAL); 
  pwmChannelInit((pwmIdentifier_t)ROBOT_WAIST, WAIST_NEUTRAL); 
}

void robotJointSetState(robotJoint_t joint, robotJointStep_t step) {
  uint32_t newJointPos;
  
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  if (step == ROBOT_JOINT_POS_INC) {
    newJointPos = jointPos[joint] + jointStep;
    if (newJointPos > jointMaxPos[joint]) {
      newJointPos = jointMaxPos[joint];
    }
  } 
  else {
    newJointPos = jointPos[joint] - jointStep;  
    if (newJointPos < jointMinPos[joint]) {
      newJointPos = jointMinPos[joint];
    } 
  }
  pwmChangeDutyCycle((pwmIdentifier_t)joint, newJointPos);
  jointPos[joint] = newJointPos;
}

uint32_t robotJointGetState(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointPos[joint];
}

uint32_t robotJointGetMinPos(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointMinPos[joint];
}

uint32_t robotJointGetMaxPos(robotJoint_t joint) {
  assert((ROBOT_HAND <= joint)&&(joint <= ROBOT_WAIST));
  return jointMaxPos[joint];
}

uint32_t robotJointGetStepValue(void) {
  return jointStep;
}

