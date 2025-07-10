#include "bsp_PWM.h"

/**
 * @brief  初始化智能车所有用到的PWM模块
 * @param  motor_freq_hz: 电机PWM的频率 (例如 17000 Hz)
 * @param  servo_freq_hz: 舵机PWM的频率 (通常为 50 Hz)
 * @retval None
 */
void bsp_pwm_init(uint32_t motor_freq_hz, uint32_t servo_freq_hz)
{
    // ---- 初始化电机PWM (使用HRTIM) ----
    // 从你的例程得知，电机接口使用了 HRTIM_1
    zf_hrtim_module_init(HRTIM_1, motor_freq_hz);

    // 初始化 HRTIM 的四个通道，初始占空比为0
    zf_hrtim_channel_init(HRTIM1_CHA1_E4, 0); // 对应 BSP_PWM_MOTOR_L1
    zf_hrtim_channel_init(HRTIM1_CHB1_E2, 0); // 对应 BSP_PWM_MOTOR_L2
    zf_hrtim_channel_init(HRTIM1_CHD1_D13, 0); // 对应 BSP_PWM_MOTOR_R1
    zf_hrtim_channel_init(HRTIM1_CHF1_D8, 0);  // 对应 BSP_PWM_MOTOR_R2

    // ---- 初始化舵机PWM (使用标准TIM) ----
    // 从你的例程得知，舵机接口使用了 TIM8 和 TIM16
    zf_pwm_module_init(PWM_TIM8, PWM_ALIGNMENT_EDGE, servo_freq_hz);
    zf_pwm_module_init(PWM_TIM16, PWM_ALIGNMENT_EDGE, servo_freq_hz);

    // 初始化 TIM8 和 TIM16 的四个通道，初始占空比为0
    zf_pwm_channel_init(PWM_TIM8_CH1_I0, 0); // 对应 BSP_PWM_SERVO_1
    zf_pwm_channel_init(PWM_TIM8_CH2_I2, 0); // 对应 BSP_PWM_SERVO_2
    zf_pwm_channel_init(PWM_TIM8_CH3_I4, 0); // 对应 BSP_PWM_SERVO_3
    zf_pwm_channel_init(PWM_TIM16_CH1_I5, 0);// 对应 BSP_PWM_SERVO_4

    // ---- [可选] 初始化LED的PWM ----
    // 你可以根据需要决定是否初始化LED
    // zf_pwm_module_init(PWM_TIM3, PWM_ALIGNMENT_EDGE, 13000);
    // zf_pwm_channel_init(PWM_TIM3_CH1_G14, PWM_DUTY_MAX); // LED 默认熄灭
    // zf_pwm_channel_init(PWM_TIM3_CH2_G15, PWM_DUTY_MAX);
    // zf_pwm_channel_init(PWM_TIM3_CH3_H0, PWM_DUTY_MAX);
}

/**
 * @brief  设置指定逻辑通道的PWM占空比
 * @param  channel: 要操作的逻辑通道，来自 bsp_pwm_channel_e 枚举
 * @param  duty_permillage: 占空比，万分比 (0 - 10000)，例如 5000 代表 50%
 * @retval None
 */
void bsp_pwm_set_duty(bsp_pwm_channel_e channel, uint16_t duty_permillage)
{
    // 输入检查，防止超出范围
    if (duty_permillage > 10000)
    {
        duty_permillage = 10000;
    }

    // 将万分比的占空比 (0-10000) 映射到逐飞库使用的占空比值
    // 逐飞库的 PWM_DUTY_MAX 宏定义了最大占空比值，我们直接用它来计算
    uint16_t zf_duty = (uint32_t)duty_permillage * PWM_DUTY_MAX / 10000;

    // 使用 switch 语句将逻辑通道映射到具体的逐飞库函数调用
    switch (channel)
    {
        // -- 舵机通道 --
        case BSP_PWM_SERVO_1:
            zf_pwm_set_duty(PWM_TIM8_CH1_I0, zf_duty);
            break;
        case BSP_PWM_SERVO_2:
            zf_pwm_set_duty(PWM_TIM8_CH2_I2, zf_duty);
            break;
        case BSP_PWM_SERVO_3:
            zf_pwm_set_duty(PWM_TIM8_CH3_I4, zf_duty);
            break;
        case BSP_PWM_SERVO_4:
            zf_pwm_set_duty(PWM_TIM16_CH1_I5, zf_duty);
            break;

        // -- 电机通道 --
        case BSP_PWM_MOTOR_L1:
            zf_hrtim_set_duty(HRTIM1_CHA1_E4, zf_duty);
            break;
        case BSP_PWM_MOTOR_L2:
            zf_hrtim_set_duty(HRTIM1_CHB1_E2, zf_duty);
            break;
        case BSP_PWM_MOTOR_R1:
            zf_hrtim_set_duty(HRTIM1_CHD1_D13, zf_duty);
            break;
        case BSP_PWM_MOTOR_R2:
            zf_hrtim_set_duty(HRTIM1_CHF1_D8, zf_duty);
            break;

        // -- LED 通道 --
        case BSP_PWM_LED_R:
            zf_pwm_set_duty(PWM_TIM3_CH1_G14, PWM_DUTY_MAX - zf_duty); // LED是共阳接法，高占空比反而暗
            break;
        case BSP_PWM_LED_G:
            zf_pwm_set_duty(PWM_TIM3_CH2_G15, PWM_DUTY_MAX - zf_duty);
            break;
        case BSP_PWM_LED_B:
            zf_pwm_set_duty(PWM_TIM3_CH3_H0, PWM_DUTY_MAX - zf_duty);
            break;

        default:
            // 无效通道，不做任何事
            break;
    }
}/*
 * PWM.c
 *
 *  Created on: 2025年7月7日
 *      Author: 20766
 */


