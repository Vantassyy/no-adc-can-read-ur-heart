#include "../user_code/bsp_PWM.h" // 确保你的工程里有 bsp_pwm.h 和 bsp_pwm.c
#include "zf_libraries_headfile.h"

// ====================================================================
// ===================  你唯一需要修改的地方  ========================
// ====================================================================
//  修改这个值，然后重新编译、烧录，观察舵机位置
//  单位是微秒 (us)
//
#define TEST_DUTY_US         (1500)  // 初始值设为1500us，这是一个常见的舵机中值

// ====================================================================
// ====================================================================

// 我们要标定的舵机连接在哪个逻辑通道上
#define SERVO_TO_CALIBRATE   BSP_PWM_SERVO_1

// 舵机工作频率
#define SERVO_FREQ           (50) // 50Hz

/**
 * @brief 辅助函数：将脉宽(us)转换为万分比占空比并设置PWM
 */
void set_servo_duty_us(uint16_t duty_us)
{
    uint32_t period_us = 1000000 / SERVO_FREQ;
    if(duty_us > period_us) duty_us = period_us;

    uint16_t duty_permillage = (uint32_t)duty_us * 10000 / period_us;

    bsp_pwm_set_duty(SERVO_TO_CALIBRATE, duty_permillage);
}

int main(void)
{
    zf_system_clock_init(SYSTEM_CLOCK_300M);

    // 初始化PWM模块
    bsp_pwm_init(0, SERVO_FREQ);

    // 设置一个固定的PWM脉宽
    set_servo_duty_us(TEST_DUTY_US);

    // 程序进入死循环，舵机将保持在固定的位置
    for (;;)
    {
        // 这里什么都不用做
        zf_delay_ms(100);
    }
}
