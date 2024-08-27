#ifndef _ESTOP_
#define _ESTOP_

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <math.h>

#define PWM1 GPIO_NUM_2
#define PWM2 GPIO_NUM_1

#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_OUTPUT_IO (PWM1) 
#define PWM_OUTPUT_IO1 (PWM2) 
#define PWM_CHANNEL_0 LEDC_CHANNEL_0
#define PWM_CHANNEL_1 LEDC_CHANNEL_1
#define PWM_DUTY_RES LEDC_TIMER_13_BIT // set duty resolution to 13 bits
#define PWM_FREQUENCY (10)
#define PWM_DUTY (819) // set duty to 10%

extern ledc_timer_config_t ESTOP0_timer;
extern ledc_channel_config_t ESTOP0_channel;
extern ledc_timer_config_t ESTOP1_timer;
extern ledc_channel_config_t ESTOP1_channel;

void estop_begin();
void estop_set_duty_cycle(double duty_cycle);
void estop_stop_pulse();

#ifdef __cplusplus
}
#endif

#endif //_ESTOP_