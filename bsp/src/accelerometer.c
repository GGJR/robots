/*************************************************************************
 *
 *    File name   : accelerometer.c
 *    Description : Configures and LPC2378 ADC to read accelerometer
 *                  values
 *
 *    History :
 *    1. Date        : November 24, 2009
 *       Author      : WDH
 *       Description : Create
 *
 *       Modified    : DK, 25-11-11
 *
 **************************************************************************/
#include <stdint.h>
#include <adc.h>
#include <accelerometer.h>



/* accelerometerInit()
 *
 * Initialise the ADC associated with the accelerometer on the LPC2378
 *
 * Must be called prior to accelerometerRead()
 */
void accelerometerInit( void ) {
  adcChannelInit((adcChannel_t)ACCEL_X);
  adcChannelInit((adcChannel_t)ACCEL_Y);
  adcChannelInit((adcChannel_t)ACCEL_Y);
}

/* Read the ADC to determine an accelerometer X-axis value
 * Returns a 10-bit value in {0..1023}
 */
uint32_t accelerometerRead(accelerometerChannel_t channel ) {
  return adcGetState((adcChannel_t)channel);
}
