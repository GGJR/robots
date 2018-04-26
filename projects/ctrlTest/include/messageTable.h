// Message IDs
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
};
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
  "Pad2 drop fail       ",};

// System State IDs
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
};
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
  "Emergency Stop       "};