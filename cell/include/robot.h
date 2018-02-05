#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

typedef enum robotJoint {ROBOT_HAND=1, ROBOT_WRIST=2, ROBOT_ELBOW=3, ROBOT_WAIST=4} robotJoint_t;
typedef enum robotJointStep_t {ROBOT_JOINT_POS_INC, ROBOT_JOINT_POS_DEC} robotJointStep_t;

void robotInit(void);
void robotJointSetState(robotJoint_t, robotJointStep_t);
uint32_t robotJointGetState(robotJoint_t);
uint32_t robotJointGetMinPos(robotJoint_t);
uint32_t robotJointGetMaxPos(robotJoint_t);
uint32_t robotJointGetStepValue(void);

#endif
