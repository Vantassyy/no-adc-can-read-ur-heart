#include "zf_libraries_headfile.h"
#include "motion_control.h" // 包含我们最终确认的单电机版运动控制模块

// ================== 测试动作配置 ==================
#define TEST_DELAY_MS       (2000)  // 每个测试动作持续的时间 (2秒)
#define TEST_MOTOR_SPEED    (20)    // 测试时使用的电机速度百分比 (40%)
#define TEST_SERVO_ANGLE    (20.0f) // 测试时使用的舵机转角 (向左转25度)

int main(void)
{
    // 1. 系统初始化
    zf_system_clock_init(SYSTEM_CLOCK_300M);

    // 2. 初始化运动控制模块
    //    这一步将自动完成PWM和GPIO的初始化，并让舵机回中、电机停止。
    motion_control_init();

    // 暂停3秒，给你充足的时间把车放在地上，并观察初始状态
    // 预期现象：舵机在正中间，车轮静止
    zf_delay_ms(3000);

    // ==========================================================
    // =================== 开始自动化测试序列 ===================
    // ==========================================================

    // --- 测试 1: 直行前进 ---
    // 功能验证: motion_set_motor_speed() 是否能驱动电机以指定速度前进
    motion_set_motor_speed(TEST_MOTOR_SPEED);
    zf_delay_ms(TEST_DELAY_MS);

    // --- 测试 2: 直行后退 ---
    // 功能验证: motion_set_motor_speed() 是否能驱动电机以指定速度后退
    motion_set_motor_speed(-TEST_MOTOR_SPEED);
    zf_delay_ms(TEST_DELAY_MS);

    // 动作间暂停，方便观察
    motion_set_motor_speed(0);
    zf_delay_ms(1000);

    // --- 测试 3: 舵机左转 (原地) ---
    // 功能验证: motion_set_servo_angle() 是否能精确控制舵机向左转
    motion_set_servo_angle(TEST_SERVO_ANGLE);
    zf_delay_ms(TEST_DELAY_MS);

    // --- 测试 4: 舵机右转 (原地) ---
    // 功能验证: motion_set_servo_angle() 是否能精确控制舵机向右转
    motion_set_servo_angle(-TEST_SERVO_ANGLE);
    zf_delay_ms(TEST_DELAY_MS);

    // 动作间暂停，舵机回中
    motion_set_servo_angle(0.0f);
    zf_delay_ms(1000);

    // --- 测试 5: 左转弯前进 (组合动作) ---
    // 功能验证: 电机和舵机是否能同时协调工作
    motion_set_servo_angle(TEST_SERVO_ANGLE); // 先打方向
    zf_delay_ms(500);                         // 稍作停顿，确保舵机到位
    motion_set_motor_speed(TEST_MOTOR_SPEED); // 再给油门
    zf_delay_ms(TEST_DELAY_MS);

    // --- 测试 6: 右转弯后退 (组合动作) ---
    // 功能验证: 反向的组合动作
    motion_set_servo_angle(-TEST_SERVO_ANGLE); // 先打方向
    zf_delay_ms(500);                          // 稍作停顿
    motion_set_motor_speed(-TEST_MOTOR_SPEED); // 再给倒车油门
    zf_delay_ms(TEST_DELAY_MS);


    // --- 测试结束，恢复安全状态 ---
    motion_set_motor_speed(0);
    motion_set_servo_angle(0.0f);

    for (;;)
    {
        // 所有测试已完成，程序在此处循环等待，车辆保持静止
        // 你可以在这里添加一个LED闪烁，作为测试完成的明确信号
        zf_delay_ms(500);
    }
}
