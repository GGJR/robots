#ifndef __ADC_H
#define __ADC_H

#include <stdint.h>

typedef enum {
  AIN0 = 0UL, AIN1, AIN2, AIN3, AIN4, AIN5, AIN6
} adcChannel_t;
  
void adcInit(void);
void adcChannelInit(adcChannel_t channel);
uint32_t adcGetState(adcChannel_t channel);

#endif