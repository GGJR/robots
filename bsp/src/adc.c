/**************************************************************************
 * ADC Interface to LPC2378 board
 *
 * ADC.c
 *
 * There are four analogue devices on the LPC2378 board:
 *    One trim potentiometer on ANALOG_TRIP_CHANNEL;
 *    three accelerometer inputs on X_CHANNEL, Y_CHANNEL and Z_CHANNEL
 *
 * This interface supports Potentiometer and Accelerometer initialisation
 * and read functions
 *
 * WDHenderson
 * November 25th, 2009
 *
 * Modified : DK, 25-11-11
 **************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <iolpc2378.h> 
#include <board.h>
#include <bsp.h>
#include <adc.h>

typedef uint32_t volatile  *pDeviceRegister_t;

#define __ADC_AD0CR      (*(pDeviceRegister_t)0xE0034000)
#define __ADC_AD0GDR     (*(pDeviceRegister_t)0xE0034004)
#define __ADC_AD0DR0     (*(pDeviceRegister_t)0xE0034010)
#define __ADC_AD0DR0_ADDR ((pDeviceRegister_t)0xE0034010)

enum {
  AD0CR_SEL_OFFSET    = 0,
  AD0CR_CLKDIV_OFFSET = 8,
  AD0CR_BURST_OFFSET  = 16,
  AD0CR_CLKS_OFFSET   = 17,
  AD0CR_PDN_OFFSET    = 21,
  AD0CR_START_OFFSET  = 24,
  AD0CR_EDGE_OFFSET   = 27
};

enum {
  AD0CR_SEL_MASK    = (0xFF << AD0CR_SEL_OFFSET),
  AD0CR_CLKDIV_MASK = (0xFF << AD0CR_CLKDIV_OFFSET),
  AD0CR_BURST_MASK  = (0x01 << AD0CR_BURST_OFFSET),
  AD0CR_CLKS_MASK   = (0x07 << AD0CR_CLKS_OFFSET),
  AD0CR_PDN_MASK    = (0x01 << AD0CR_PDN_OFFSET),
  AD0CR_START_MASK  = (0x07 << AD0CR_START_OFFSET),
  AD0CR_EDGE_MASK   = (0x01 << AD0CR_EDGE_OFFSET)
};

void adcInit( void ) {

/* Disable all ADC interrupts - this is a polling service */
   ADINTEN = 0;
  
/* Configure ADC clock */
   PCONP_bit.PCAD = 1;                     // Enable ADC clk
   PCLKSEL0 |= (1UL << ADC_PCLK_OFFSET);   // ADC_PCLK == CCLK

   __ADC_AD0CR = (
     (0u << AD0CR_SEL_OFFSET)     |        //  [0]  in SEL bits 0 .. 7
     (13u << AD0CR_CLKDIV_OFFSET) |        //  [13] A/D clk == ADC_PCLK / (CLKDIV + 1) <= 4.5 MHz
     (0u << AD0CR_BURST_OFFSET)   |        //  [0]  conversions are software controlled
     (0u << AD0CR_CLKS_OFFSET)    |        //  [0]  10 bit conversions
     (1u << AD0CR_PDN_OFFSET)     |        //  [1]  A/D converter is operational
     (0u << AD0CR_START_OFFSET)   |        //  [0]  no start
     (0u << AD0CR_EDGE_OFFSET)             //  [0]  significant only when start >= 2
   );
}



/*
 * Initialise a single ADC channel for polling
 *
 * Parameter in {AIN0, AIN1, AIN2, AIN5, AIN6}
 *
 * 
 *
 */
void adcChannelInit( adcChannel_t channel ) {

/* Check parameter in range */
  assert( (channel == AIN0) || 
          (channel == AIN1) ||
          (channel == AIN5) || 
          (channel == AIN6) );

/* Enable required ADC channel */
   switch ( channel ) {
   case AIN0:
     PINSEL1_bit.P0_23 = 1;    /* Assign P0.23 to AIN0 */
     break;
   case AIN1:
     PINSEL1_bit.P0_24 = 1;    /* Assign P0.24 to AIN1 */
     break;
  case AIN5:
     PINSEL3_bit.P1_31 = 3;    /* Assign P1.31 to AIN5 */
     break;
   case AIN6:
     PINSEL0_bit.P0_12 = 3;    /* Assign P0.12 to AIN6 */
   }
}

/*
 * Reads a single ADC channel
 * Parameter in {AIN0, AIN1, AIN5, AIN6}
 *
 * Returns the 10-bit ADC conversion value
 * 
 *
 */
uint32_t adcGetState( adcChannel_t channel ) {
   uint32_t adcData;     /* Value read from ADC */

/* Check parameter in range */
  assert( (channel == AIN0) || 
          (channel == AIN1) || 
          (channel == AIN5) || 
          (channel == AIN6) );
   
/* Select ADC channel */
  __ADC_AD0CR &= ~AD0CR_SEL_MASK;     
  __ADC_AD0CR |= ((0x01 << channel) << AD0CR_SEL_OFFSET);     

  __ADC_AD0CR |= (1ul << AD0CR_START_OFFSET);         // Start conversion

  adcData = *(__ADC_AD0DR0_ADDR + channel);           // Read value on selected channel
  while ((adcData  & (1ul << 31) == 0)) {             // Test if DONE 
    adcData = *(__ADC_AD0DR0_ADDR + channel);         // Read value
  }

  __ADC_AD0CR &= ~AD0CR_START_MASK;                   // Stop conversion

/* Shift 10-bit ADC value down and mask other bits */
  return ((adcData >> 6) & 0x000003FF);
}

