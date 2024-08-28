#include "estop.h"

void estop_begin(shoalbot_estop* obj) {
    obj->ESTOP0_timer.speed_mode = PWM_MODE;
    obj->ESTOP0_timer.timer_num = LEDC_TIMER_0;
    obj->ESTOP0_timer.duty_resolution = PWM_DUTY_RES;
    obj->ESTOP0_timer.freq_hz = PWM_FREQUENCY;
    obj->ESTOP0_timer.clk_cfg = LEDC_AUTO_CLK;

    obj->ESTOP0_channel.gpio_num = PWM_OUTPUT_IO;
    obj->ESTOP0_channel.speed_mode = PWM_MODE;
    obj->ESTOP0_channel.channel = PWM_CHANNEL_0;
    obj->ESTOP0_channel.timer_sel = LEDC_TIMER_0;
    obj->ESTOP0_channel.duty = PWM_DUTY;
    obj->ESTOP0_channel.hpoint = 0;

    obj->ESTOP1_timer.speed_mode = PWM_MODE;
    obj->ESTOP1_timer.timer_num = LEDC_TIMER_1;
    obj->ESTOP1_timer.duty_resolution = PWM_DUTY_RES;
    obj->ESTOP1_timer.freq_hz = PWM_FREQUENCY;
    obj->ESTOP1_timer.clk_cfg = LEDC_AUTO_CLK;

    obj->ESTOP1_channel.gpio_num = PWM_OUTPUT_IO1;
    obj->ESTOP1_channel.speed_mode = PWM_MODE;
    obj->ESTOP1_channel.channel = PWM_CHANNEL_1;
    obj->ESTOP1_channel.timer_sel = LEDC_TIMER_1;
    obj->ESTOP1_channel.duty = PWM_DUTY;
    obj->ESTOP1_channel.hpoint = 0;

    ledc_timer_config(&obj->ESTOP0_timer);
    vTaskDelay(pdMS_TO_TICKS(50));
    ledc_timer_config(&obj->ESTOP1_timer);

    ledc_channel_config(&obj->ESTOP0_channel);
    ledc_channel_config(&obj->ESTOP1_channel);
}

void estop_set_duty_cycle(double duty_cycle) {
    uint32_t pwm_duty = pow(2, 13) * duty_cycle;
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_0, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_1));
}

void estop_stop_pulse() {
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_1));
}