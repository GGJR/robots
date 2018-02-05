#include <assert.h>
#include <iolpc2378.h>
#include <board.h>
#include <bsp.h>
#include <pwm.h>

static uint32_t pwmMatchMajor = 0;

void pwmInit(uint32_t majorCycleHz) {
  PWM1TCR = 0x02;  // reset PWM
  PWM1PR = 0x00;   // set prescaler to 0
  PWM1CTCR = 0x00; // set Timer mode
  pwmMatchMajor = getFpclk(PWM1_PCLK_OFFSET) / majorCycleHz;
  PWM1MR0 = pwmMatchMajor; // set the major cycle
  PWM1MCR = 0x02;  // enable reset on match
  PWM1PCR = 0x00;  // select single-edge PWM and disable PWM outputs on all channels
  PWM1TCR = 0x09;  // enable PWM function and start PWM1
}

void pwmChannelInit(pwmIdentifier_t channel, uint32_t dutyCyclePerMillion) {
  uint32_t pwmMatchDuty;

  assert(dutyCyclePerMillion <= 1000000);
  pwmMatchDuty = (uint32_t)(pwmMatchMajor * (dutyCyclePerMillion / 1000000.0));
  
  switch (channel) {
  case PWM1:
    PWM1MR1 = pwmMatchDuty;    // set the duty cycle on channel 1
    PWM1LER |= (1UL << 1);     // latch enable the new duty cycle value on channel 1
    PWM1PCR |= (1UL << 9);     // enable PWM output on channel 1
    break;
  case PWM2:
    PWM1MR2 = pwmMatchDuty;    // set the duty cycle on channel 2
    PWM1LER |= (1UL << 2);     // latch enable the new duty cycle value on channel 2
    PWM1PCR |= (1UL << 10);    // enable PWM output on channel 2
    break;
  case PWM3:
    PWM1MR3 = pwmMatchDuty;    // set the duty cycle on channel 3
    PWM1LER |= (1UL << 3);     // latch enable the new duty cycle value on channel 3
    PWM1PCR |= (1UL << 11);    // enable PWM output on channel 3
    break;
  case PWM4:
    PWM1MR4 = pwmMatchDuty;    // set the duty cycle on channel 4
    PWM1LER |= (1UL << 4);     // latch enable the new duty cycle value on channel 4
    PWM1PCR |= (1UL << 12);    // enable PWM output on channel 4
    break;
  case PWM5:
    PWM1MR5 = pwmMatchDuty;    // set the duty cycle on channel 5
    PWM1LER |= (1UL << 5);     // latch enable the new duty cycle value on channel 5
    PWM1PCR |= (1UL << 13);    // enable PWM output on channel 5
    break;
  case PWM6:
    PWM1MR6 = pwmMatchDuty;    // set the duty cycle on channel 6
    PWM1LER |= (1UL << 6);     // latch enable the new duty cycle value on channel 6
    PWM1PCR |= (1UL << 14);    // enable PWM output on channel 6
    break;
  default:
    break;
 }
}

void pwmChangeDutyCycle(pwmIdentifier_t channel, uint32_t dutyCyclePerMillion) {
  uint32_t pwmMatchDuty;
  
  assert(dutyCyclePerMillion <= 1000000);
  pwmMatchDuty = (uint32_t)(pwmMatchMajor * (dutyCyclePerMillion / 1000000.0));
  
  switch (channel) {
  case PWM1:
    PWM1MR1 = pwmMatchDuty;    // set the duty cycle on channel 1
    PWM1LER |= (1UL << 1);     // latch enable the new duty cycle value on channel 1
    break;
  case PWM2:
    PWM1MR2 = pwmMatchDuty;    // set the duty cycle on channel 2
    PWM1LER |= (1UL << 2);     // latch enable the new duty cycle value on channel 2
    break;
  case PWM3:
    PWM1MR3 = pwmMatchDuty;    // set the duty cycle on channel 3
    PWM1LER |= (1UL << 3);     // latch enable the new duty cycle value on channel 3
    break;
  case PWM4:
    PWM1MR4 = pwmMatchDuty;    // set the duty cycle on channel 4
    PWM1LER |= (1UL << 4);     // latch enable the new duty cycle value on channel 4
    break;
  case PWM5:
    PWM1MR5 = pwmMatchDuty;    // set the duty cycle on channel 5
    PWM1LER |= (1UL << 5);     // latch enable the new duty cycle value on channel 5
    break;
  case PWM6:
    PWM1MR6 = pwmMatchDuty;    // set the duty cycle on channel 6
    PWM1LER |= (1UL << 6);     // latch enable the new duty cycle value on channel 6
    break;
  default:
    break;
 }
}


