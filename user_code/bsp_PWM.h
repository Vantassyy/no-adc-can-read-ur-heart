/*
 * PWM.h
 *
 *  Created on: 2025年7月7日
 *      Author: 20766
 */

#ifndef USER_CODE_BSP_PWM_H_
#define USER_CODE_BSP_PWM_H_

#include "zf_libraries_headfile.h" // 必须包含逐飞库的头文件

/**
 * @brief 定义我们智能车上所有使用PWM的逻辑通道
 *        我们将原理图上的物理引脚功能，映射为易于理解的逻辑名称
 */
typedef enum
{
    // -- 舵机通道 (对应原理图 舵机接口X4) --
    //   这些通道使用标准定时器(TIM)
    BSP_PWM_SERVO_1,        // 逻辑舵机1 -> 物理引脚 I0  -> 逐飞库宏 PWM_TIM8_CH1_I0
    BSP_PWM_SERVO_2,        // 逻辑舵机2 -> 物理引脚 I2  -> 逐飞库宏 PWM_TIM8_CH2_I2
    BSP_PWM_SERVO_3,        // 逻辑舵机3 -> 物理引脚 I4  -> 逐飞库宏 PWM_TIM8_CH3_I4
    BSP_PWM_SERVO_4,        // 逻辑舵机4 -> 物理引脚 I5  -> 逐飞库宏 PWM_TIM16_CH1_I5

    // -- 电机驱动通道 (对应原理图 有刷/无刷电机接口) --
    //   这些通道使用高精度定时器(HRTIM)
    BSP_PWM_MOTOR_L1,       // 逻辑电机左1 -> 物理引脚 E4 -> 逐飞库宏 HRTIM1_CHA1_E4
    BSP_PWM_MOTOR_L2,       // 逻辑电机左2 -> 物理引脚 E2 -> 逐飞库宏 HRTIM1_CHB1_E2
    BSP_PWM_MOTOR_R1,       // 逻辑电机右1 -> 物理引脚 D13-> 逐飞库宏 HRTIM1_CHD1_D13
    BSP_PWM_MOTOR_R2,       // 逻辑电机右2 -> 物理引脚 D8 -> 逐飞库宏 HRTIM1_CHF1_D8

    // -- 其他PWM通道 (例如用于RGB LED) --
    BSP_PWM_LED_R,          // 逻辑LED-R -> 物理引脚 G14 -> 逐飞库宏 PWM_TIM3_CH1_G14
    BSP_PWM_LED_G,          // 逻辑LED-G -> 物理引脚 G15 -> 逐飞库宏 PWM_TIM3_CH2_G15
    BSP_PWM_LED_B,          // 逻辑LED-B -> 物理引脚 H0  -> 逐飞库宏 PWM_TIM3_CH3_H0

    BSP_PWM_CHANNEL_MAX     // 通道总数，用于校验
} bsp_pwm_channel_e;


/**
 * @brief  初始化智能车所有用到的PWM模块
 * @param  motor_freq_hz: 电机PWM的频率 (例如 17000 Hz)
 * @param  servo_freq_hz: 舵机PWM的频率 (通常为 50 Hz)
 * @retval None
 */
void bsp_pwm_init(uint32_t motor_freq_hz, uint32_t servo_freq_hz);

/**
 * @brief  设置指定逻辑通道的PWM占空比
 * @param  channel: 要操作的逻辑通道，来自 bsp_pwm_channel_e 枚举
 * @param  duty_permillage: 占空比，万分比 (0 - 10000)，例如 5000 代表 50%
 * @retval None
 */
void bsp_pwm_set_duty(bsp_pwm_channel_e channel, uint16_t duty_permillage);


#endif /* USER_CODE_BSP_PWM_H_ */
