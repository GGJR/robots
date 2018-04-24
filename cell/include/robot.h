#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

typedef enum robotJoint {ROBOT_HAND=1, ROBOT_WRIST=2, ROBOT_ELBOW=3, ROBOT_WAIST=4} robotJoint_t;
typedef enum robotJointStep_t {ROBOT_JOINT_POS_INC, ROBOT_JOINT_POS_DEC} robotJointStep_t;

/********************************************************************************
*            ROBOT POSITION COORDINATES OF PRODUCTION CELL SYSTEM
*********************************************************************************/

// Data structure robotCoordinates holding uint32_t elbow, wrist, waist 
// and hand values.  
typedef struct robotCoordinates{
	
	uint32_t elbow;
	uint32_t wrist;
	uint32_t waist;
	uint32_t hand;
	
}robotCoordinates_t;

// Definitions for elbow's position coordinates (from lower to higher order)(^)
#define elbow_82000  82000
#define elbow_83500  83500
#define elbow_87500  87500
#define elbow_100000 100000

// Definition for wrist's position coordinate
#define wrist_90000  90000

// Definitions for waist's position coordinates (from lower to higher order) (^)
#define waist_45000  45000
#define waist_67250  67250
#define waist_68750  68750
#define waist_82250  82250
#define waist_84750  84750

// Definitions for hand's position coordinates (from lower to higher order) (^)
#define hand_45000  45000
#define hand_69000  69000
#define hand_87500  87500

void robotInit(void);
void robotJointSetState(robotJoint_t, robotJointStep_t);
uint32_t robotJointGetState(robotJoint_t);
uint32_t robotJointGetMinPos(robotJoint_t);
uint32_t robotJointGetMaxPos(robotJoint_t);
uint32_t robotJointGetStepValue(void);

#endif
