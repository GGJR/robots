/***************************************************************************
* 
* messageTable.h
*
* Header file of OUTPUT ROBOT of the production cell project
* containing the CAN message IDs, message display contents,
* system state IDs and system state display contents.
*
* File shared with Input Robot, Controller and Conveyor
* for the successful implementation of the production cell
* under CAN network.
*
* @Date: 26 March 2018
* @Author: Giorgos Tsapparellas
* @Group ID: 4
* @Version: 1.0 (Complete) 
*                
*********************************************************************************/

/*********************************************************************************
*                 CAN Message IDs and Display Message Contents
*********************************************************************************/

// enum containing CAN message IDs starting from 1.
// Utilized by the appTaskCanSendReceive function
// for sending and receiving CAN messages to/from 
// Controller and Conveyor.
// CAN message IDs employed also for monitoring
// appTaskCanSendReceive function current state.
enum {
  EM_STOP = 1,
  EM_STOP_ACK_CONV,
  EM_STOP_ACK_ROB1,
  EM_STOP_ACK_ROB2,
  ERR_CONV,
  ERR_ROB1,
  ERR_ROB2,
  PAUSE,
  PAUSE_ACK_CONV,
  PAUSE_ACK_ROB1,
  PAUSE_ACK_ROB2,
  RESUME,
  RESUME_ACK_CONV,
  RESUME_ACK_ROB1,
  RESUME_ACK_ROB2,
  START,
  START_ACK_CONV,
  START_ACK_ROB1,
  START_ACK_ROB2,
  CTRL_STOP,
  CTRL_STOP_ACK_CONV,
  CTRL_STOP_ACK_ROB1,
  CTRL_STOP_ACK_ROB2,
  RESET,
  RESET_ACK_CONV,
  RESET_ACK_ROB1,
  RESET_ACK_ROB2,
  REQ_PICKUP_PAD1,
  ACK_PICKUP_PAD1,
  CHK_PAD1_PICKUP,
  ACK_CHK_PAD1_PICKUP,
  NACK_CHK_PAD1_PICKUP,
  REQ_DROP_CONV,
  ACK_DROP_CONV,
  NACK_DROP_CONV,
  CHK_CONV_DROP,
  ACK_CHK_CONV_DROP,
  NACK_CHK_CONV_DROP,
  REQ_PICKUP_CONV,
  ACK_PICKUP_CONV,
  CHK_CONV_PICKUP,
  ACK_CHK_CONV_PICKUP,
  NACK_CHK_CONV_PICKUP,
  REQ_DROP_PAD2,
  ACK_DROP_PAD2,
  NACK_DROP_PAD2,
  CHK_PAD2_DROP,
  ACK_CHK_PAD2_DROP,
  NACK_CHK_PAD2_DROP
}; // end of enum containing CAN message IDs.

// char containing CAN message display contents.
// Utilized by the appTaskCanSendReceive function
// for informing user which CAN message is
// currently on-going through LCD. 
// Array of size 50 with length 21 for 
// each message. Position 0 kept null as
// CAN message IDs starting from 1.
char displayMessageContents[50][21] = {
  "                     ",
  "Emergency stop sent  ",
  "Conv emergency stop  ",
  "Robo1 emergency stop ",
  "Robo2 emergency stop ",
  "Conveyor Error       ",
  "Robot1 Error         ",
  "Robot2 Error         ",
  "Pause Message Sent   ",
  "Conv pause ack       ",
  "Robo1 pause ack      ",
  "Robo2 pause ack      ",
  "Resume message sent  ",
  "Conv resume ack      ",
  "Robo1 resume ack     ",
  "Robo2 resume ack     ",
  "Start message sent   ",
  "Conv start ack       ",
  "Robo1 start ack      ",
  "Robo2 start ack      ",
  "Stop message sent    ",
  "Conv stop ack        ",
  "Robo1 stop ack       ",
  "Robo2 stop ack       ",
  "Reset message sent   ",
  "Conv reset ack       ",
  "Robo1 reset ack      ",
  "Robo2 reset ack      ",
  "Pad1 has block sent  ",
  "Robo1 ack pad1 block ",
  "Robo1 chk got block  ",
  "Pad1 success pickup  ",
  "Pad1 failed pickup   ",
  "Robo1 request drop   ",
  "Conv approved drop   ",
  "Conv denied drop     ",
  "Robo1 check drop     ",
  "Conv drop success    ",
  "Conv drop failure    ",
  "Conv req pickup      ",
  "Robo2 pickup ack     ",
  "Robo2 chk got block  ",
  "Conv success pickup  ",
  "Conv failed pickup   ",
  "Robo2 request pad2   ",
  "Pad2 free sent       ",
  "Pad2 full sent       ",
  "Pad2 drop check      ",
  "Pad2 drop success    ",
  "Pad2 drop fail       "
}; // end of char containing display message contents.

/*********************************************************************************
*               System State IDs and Display System State Contents
*********************************************************************************/

// enum containing System State IDs starting from 0.
// Utilized by the appTaskCanSendReceive function
// for monitoring current system state.
enum {
  SYSTEM_NOT_STARTED = 0,
  SYSTEM_STARTING,
  SYSTEM_RUNNING,
  SYSTEM_PAUSING,
  SYSTEM_PAUSED,
  SYSTEM_RESUMING,
  SYSTEM_STOPPING,
  SYSTEM_STOPPED,
  SYSTEM_RESETTING,
  SYSTEM_ERROR_DETECTED,
  SYSTEM_EMERGENCY_STOP,
}; // end of enum containing System State IDs.

// char containing System State display contents.
// Utilized by the appTaskCanSendReceive function
// for informing user which System State is
// currently on-going through LCD. 
// Array of size 11 with length 21 for 
// each message.
char displaySystemState[11][21] = {
  "System Not Started   ",
  "System Starting      ",
  "System Running       ",
  "System Pausing       ",
  "System Paused        ",
  "System Resuming      ",
  "System Stopping      ",
  "System Stopped       ",
  "System Resetting     ",
  "System Error         ",
  "Emergency Stop       "
}; // end of char containing display system state contents.