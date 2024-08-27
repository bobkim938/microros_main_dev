#include "estop.h"

ledc_timer_config_t ESTOP0_timer = {
    .speed_mode = PWM_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = PWM_DUTY_RES,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t ESTOP0_channel = {
    .gpio_num = PWM_OUTPUT_IO,
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = PWM_DUTY,
    .hpoint = 0
};

ledc_timer_config_t ESTOP1_timer = {
    .speed_mode = PWM_MODE,
    .timer_num = LEDC_TIMER_1,
    .duty_resolution = PWM_DUTY_RES,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t ESTOP1_channel = {
    .gpio_num = PWM_OUTPUT_IO1,
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .duty = PWM_DUTY,
    .hpoint = 0
};

void estop_begin() {
    ledc_timer_config(&ESTOP0_timer);
    vTaskDelay(pdMS_TO_TICKS(50));
    ledc_timer_config(&ESTOP1_timer);

    ledc_channel_config(&ESTOP0_channel);
    ledc_channel_config(&ESTOP1_channel);
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