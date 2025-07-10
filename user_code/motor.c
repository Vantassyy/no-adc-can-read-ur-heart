#include "motor.h"
#include "zf_libraries_headfile.h" // 需要包含逐飞库的头文件才能用zf_函数
#include <stdlib.h> // 需要包含它才能用 abs() 函数

void motor_init(void) {
    zf_hrtim_module_init(HRTIM_1, 17000);
    zf_hrtim_channel_init(HRTIM1_CHA1_E4, 0);
    zf_gpio_init(E2, GPO_PUSH_PULL, 0);
    // ... 如果有其他电机，也在这里初始化
}

void set_motor_speed(int motor_id, int speed) {
    if(speed > 100) speed = 100;
    if(speed < -100) speed = -100;

    uint16_t duty = (uint16_t)(abs(speed) * (PWM_DUTY_MAX / 100.0f));

    if(motor_id == 1) {
        if(speed >= 0) zf_gpio_set_level(E2, GPIO_HIGH);
        else zf_gpio_set_level(E2, GPIO_LOW);
        zf_hrtim_set_duty(HRTIM1_CHA1_E4, duty);
    }
}/*
 * motor.c
 *
 *  Created on: 2025年7月6日
 *      Author: 20766
 */


