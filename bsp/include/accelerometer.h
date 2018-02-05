/*************************************************************************
 *
 *    File name   : accelerometer.h
 *    Description : Configures and LPC2378 ADC to read accelerometer
 *                  values
 *
 *    History :
 *    1. Date        : November 24, 2009
 *       Author      : WDH
 *       Description : Create
 *
 **************************************************************************/
#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include <stdint.h>

typedef enum {
  ACCEL_X = 1UL,
  ACCEL_Y = 0UL,
  ACCEL_Z = 6UL
} accelerometerChannel_t;

/* Initialise the ADC to read X, Y and X axis accelerometer values
 */
void accelerometerInit( void );



/* Read the ADC to determine an accelerometer channel value
 */
uint32_t accelerometerRead( accelerometerChannel_t );

#endif