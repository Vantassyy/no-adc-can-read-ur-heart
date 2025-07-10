#include "motion_control.h"
#include <stdlib.h> // for abs()

// ================== 内部宏定义 ==================

// -- 舵机通道定义 (来自 bsp_pwm.h, 正确) --
#define SERVO_CHANNEL       BSP_PWM_SERVO_1

// -- 电机通道定义 (来自原理图和你的代码, 确认无误) --
#define MOTOR_PWM_CH        BSP_PWM_MOTOR_L1   // 使用 E4 引脚作为速度PWM
#define MOTOR_DIR_PIN       (E2)               // 使用 E2 引脚作为方向GPIO

// -- [已修正] 电机方向电平定义 (使用从 zf_driver_gpio.h 中找到的官方宏) --
#define MOTOR_DIR_FORWARD   (GPIO_HIGH)
#define MOTOR_DIR_REVERSE   (GPIO_LOW)

// ================== 内部辅助函数 ==================

// 线性映射辅助函数
static inline float map_value(float value, float from_min, float from_max, float to_min, float to_max)
{
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

// ================== API函数实现 ==================

void motion_control_init(void)
{
    // 1. 初始化PWM模块
    bsp_pwm_init(MOTOR_PWM_FREQ_HZ, SERVO_PWM_FREQ_HZ);

    // 2. [已修正] 初始化电机的方向控制引脚
    //    使用官方宏 GPO_PUSH_PULL 和 MOTOR_DIR_REVERSE (即 GPIO_LOW)。
    //    函数调用为正确的3个参数。
    zf_gpio_init(MOTOR_DIR_PIN, GPO_PUSH_PULL, MOTOR_DIR_REVERSE);

    // 3. 设置初始状态
    motion_set_servo_angle(0.0f);
    motion_set_motor_speed_openloop(0, 0);
}

void motion_set_servo_angle(float angle)
{
    // ... 此部分代码不涉及GPIO，是正确的，无需修改 ...
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;

    uint16_t duty_us;
    if (angle >= 0)
    {
        duty_us = (uint16_t)map_value(angle, 0, SERVO_ANGLE_MAX, SERVO_DUTY_NEUTRAL_US, SERVO_DUTY_MAX_US);
    }
    else
    {
        duty_us = (uint16_t)map_value(angle, SERVO_ANGLE_MIN, 0, SERVO_DUTY_MIN_US, SERVO_DUTY_NEUTRAL_US);
    }

    uint32_t period_us = 1000000 / SERVO_PWM_FREQ_HZ;
    uint16_t duty_permillage = (uint32_t)duty_us * 10000 / period_us;

    bsp_pwm_set_duty(SERVO_CHANNEL, duty_permillage);
}

void motion_set_motor_speed_openloop(int16_t left_speed, int16_t right_speed)
{
    int16_t speed = (left_speed + right_speed) / 2;

    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    if (speed >= 0) // 前进或停止
    {
        // 1. [已修正] 设置方向为前进
        zf_gpio_set_level(MOTOR_DIR_PIN, MOTOR_DIR_FORWARD);

        // 2. 设置PWM占空比
        uint16_t duty = (uint16_t)(speed * 100);
        bsp_pwm_set_duty(MOTOR_PWM_CH, duty);
    }
    else // 后退
    {
        // 1. [已修正] 设置方向为后退
        zf_gpio_set_level(MOTOR_DIR_PIN, MOTOR_DIR_REVERSE);

        // 2. 设置PWM占空比
        uint16_t duty = (uint16_t)(abs(speed) * 100);
        bsp_pwm_set_duty(MOTOR_PWM_CH, duty);
    }
}

void motion_set_motor_speed_closedloop(float left_target_speed, float right_target_speed)
{
    (void)left_target_speed;
    (void)right_target_speed;
    // 待实现
}
