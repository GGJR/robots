#include <stdint.h>
#include <bsp.h>

typedef enum pwmIdentifier {PWM1=1, PWM2=2, PWM3=3, PWM4=4, PWM5=5, PWM6=6} pwmIdentifier_t;


void pwmInit(uint32_t majorCycleHz);
void pwmChannelInit(pwmIdentifier_t pwm, uint32_t dutyCyclePerMillion);
void pwmChangeDutyCycle(pwmIdentifier_t pwm, uint32_t dutyCyclePerMillion);
