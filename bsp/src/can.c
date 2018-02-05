#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <iolpc2378.h>
#include <can.h>
#include <bsp.h>


/*
 * Bit Timing Values for 60MHz clk frequency
 */
#define BITRATE100K60MHZ          0x00090031
#define BITRATE125K60MHZ          0x00090027
#define BITRATE250K60MHZ          0x00090013
#define BITRATE500K60MHZ          0x00090009
#define BITRATE1000K60MHZ         0x00090004


/**
 * Bit Timing Values for 15MHz(60Mhz CCLK) clk frequency
 *
#define BITRATE100K15MHZ		0x0007000E
#define BITRATE125K15MHZ		0x0007000B
#define BITRATE250K15MHZ		0x00070005
#define BITRATE500K15MHZ		0x00070002
 */


#ifndef CAN_BITRATE
#define CAN_BITRATE BITRATE100K60MHZ
#endif

#define ACCF_BYPASS          0x02   // filter off, receive all


bool can1SendMessage(canMessage_t *);
bool can2SendMessage(canMessage_t *);
void canRxInterrupt(pVoidFunc_t);



/* Please note, this PCLK is set in the bsp.h file. Since the example
  program is set to test USB device, there is only a limited number of
  options to set CCLK and PCLK when USB is used. The default setting is
  CCLK 60MHz, PCLK is 1/4 CCLK so 15MHz, except for CAN1, CAN2 and ACF where
  PCLK is set to CCLK. The bit timing is based on the
  setting of the PCLK, if different PCLK is used, please read can.h carefully
  and set your CAN bit timing accordingly.

  No CAN interrupts
*/
void canInit(void) {
  PCONP |= (1 << 13) | (1 << 14); // Enable clock to the peripheral

  PINSEL0 &= ~0x00000F0F;
  PINSEL0 |= 0x0000A05;   // port0.0~1, function 0x01, port0.4~5, function 0x10

  AFMR = ACCF_BYPASS;     // Acceptance filtering off, receive all messages
  CAN1MOD = 1;
  CAN2MOD = 1;  // Reset CAN
  CAN1IER = 0;
  CAN2IER = 0;  // Disable Receive Interrupt
  CAN1GSR = 0;
  CAN2GSR = 0;  // Reset error counter when CANxMOD is in reset

  CAN1BTR = CAN_BITRATE;
  CAN2BTR = CAN_BITRATE;
  CAN1MOD = 0x0;          // CAN in normal operation mode
  CAN2MOD = 0x0;
}

/* 
 * Enable CAN interrupts and install handler
 */
void canRxInterrupt(pVoidFunc_t canHandler) {
  vicInstallIRQhandler(canHandler, 1, VIC_CAN12);
  CAN1IER = 0x01;
  CAN2IER = 0x01;   // Enable receive interrupts
} 


/* 
 * return true if a message is available in the receive
 * buffer of the specified CAN port
 * bit 8 in CANRXSR is set if message available for port 1
 * bit 9 in CANRXSR is set if message available for port 2
 */
bool canReady(uint32_t port) {
  assert((port == 0) || (port == 1));
  return ((CANRXSR & (1 << (8 + port)) ? true : false));
}

          
bool canWrite(uint32_t port, canMessage_t *msg) {
  bool result;
  
  assert((port ==0) || (port == 1));
  if (port == 0) {
    result = can1SendMessage(msg);
  } else { 
    result = can2SendMessage(msg);
  }
  return result;
}
            

         
/*
 * Assume 11-bit identifier
 */          
void canRead(uint32_t port, canMessage_t *msg) {
  assert((port == 0) || (port == 1));
  if (port == 0) {
    msg->id = (CAN1RID & 0x000007FF);
    msg->len = ((CAN1RFS & 0x000F0000) >> 16);
    msg->dataA = CAN1RDA;
    msg->dataB = CAN1RDB;
    CAN1CMR |= 0x04;
  } else {
    msg->id = (CAN2RID & 0x000007FF);
    msg->len = ((CAN2RFS & 0x000F0000) >> 16);
    msg->dataA = CAN2RDA;
    msg->dataB = CAN2RDB;
    CAN2CMR |= 0x04;
  }
}

bool can1SendMessage(canMessage_t *msg) {
  uint32_t canStatus;
  bool result = false;

  canStatus = CAN1SR;
  if ( canStatus & 0x00000004 ) {
    CAN1TID1 = msg->id;
    CAN1TFI1 = msg->len << 16; 
    CAN1TDA1 = msg->dataA;
    CAN1TDB1 = msg->dataB;
    CAN1CMR = 0x21;
    result = true;
  }
  else if ( canStatus & 0x00000400 ) {
    CAN1TID2 = msg->id;
    CAN1TFI2 = msg->len << 16;
    CAN1TDA2 = msg->dataA;
    CAN1TDB2 = msg->dataB;
    CAN1CMR = 0x41;
    result = true;
  }
  else if ( canStatus & 0x00040000 ) { 
    CAN1TID3 = msg->id;
    CAN1TFI3 = msg->len << 16;
    CAN1TDA3 = msg->dataA;
    CAN1TDB3 = msg->dataB;
    CAN1CMR = 0x81;
    result = true;
  }
  return result;
}

bool can2SendMessage(canMessage_t *msg) {
  uint32_t canStatus;
  bool result = false;

  canStatus = CAN2SR;
  if ( canStatus & 0x00000004 ) {
    CAN2TID1 = msg->id;
    CAN2TFI1 = msg->len << 16; 
    CAN2TDA1 = msg->dataA;
    CAN2TDB1 = msg->dataB;
    CAN2CMR = 0x21;
    result = true;
  }
  else if ( canStatus & 0x00000400 ) {
    CAN2TID2 = msg->id;
    CAN2TFI2 = msg->len << 16;
    CAN2TDA2 = msg->dataA;
    CAN2TDB2 = msg->dataB;
    CAN2CMR = 0x41;
    result = true;
  }
  else if ( canStatus & 0x00040000 ) { 
    CAN2TID3 = msg->id;
    CAN2TFI3 = msg->len << 16;
    CAN2TDA3 = msg->dataA;
    CAN2TDB3 = msg->dataB;
    CAN2CMR = 0x81;
    result = true;
  }
  return result;
}
          

/*
 * Returns Global Status Register for chosen CAN port
 *
 * 31      24 23      16 15       8  7   6   5   4   3   2   1   0
 * +---------+---------+-----------+---+---+---+---+---+---+---+---+
 * | TXERROR | RXERROR | Reserved  | BS| ES| TS| RS|TCS|TBS|DOS|RBS|
 * +---------+---------+-----------+---+---+---+---+---+---+---+---+
 *
 */
uint32_t canStatus(uint32_t port) {
  assert((port == 0) || (port == 1));
  return (port == 0 ? CAN1GSR : CAN2GSR);
}
