/*********************************************************************************************************************
 * @file        main_hardware_test.c
 * @brief       电机与舵机硬件开环测试程序
 * @version     1.0
 * @date        2023-10-27
 * @author      (Your Name)
 * @note        本程序不依赖编码器，专门用于验证电机和舵机硬件是否正常工作。
 ********************************************************************************************************************/

#include "zf_libraries_headfile.h"
#include "motor.h"  // 只需要 motor 和 servo 模块
#include "servo.h"

int main (void)
{
    // --- 1. 系统初始化 ---
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    debug_init();

    // --- 2. 硬件模块初始化 ---
    motor_init();
    servo_init();

    printf("Hardware Test Program Started.\r\n");
    printf("Motor and Servo will be tested now.\r\n");

    // --- 3. 舵机功能测试 ---
    printf("Testing Servo...\r\n");
    set_servo_angle(90);    // 让舵机回到中位
    zf_delay_ms(1000);      // 等待1秒

    set_servo_angle(45);    // 转到左边45度
    zf_delay_ms(1000);

    set_servo_angle(135);   // 转到右边135度
    zf_delay_ms(1000);

    set_servo_angle(90);    // 回到中位
    zf_delay_ms(1000);
    printf("Servo test finished.\r\n");


    // --- 4. 电机功能测试 ---
    // 我们将让电机以不同的速度正转和反转
    printf("Testing Motor...\r\n");

    // 以30%的速度正转3秒
    printf("Forward at 30%% speed for 3 seconds.\r\n");
    set_motor_speed(1, 2);
    zf_delay_ms(3000);

    // 以70%的速度正转3秒
    printf("Forward at 70%% speed for 3 seconds.\r\n");
    set_motor_speed(1, 5);
    zf_delay_ms(3000);

    // 停止1秒
    printf("Stop for 1 second.\r\n");
    set_motor_speed(1, 0);
    zf_delay_ms(1000);


    printf("Motor test finished. Program end.\r\n");
    set_motor_speed(1, 0);

    // --- 5. 进入无限循环 ---
    // 测试完成，程序停在这里
    for( ; ; )
    {
    }
}/*
 * main_tast.c
 *
 *  Created on: 2025年7月7日
 *      Author: 20766
 */


