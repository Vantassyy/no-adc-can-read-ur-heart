/*********************************************************************************************************************
 * @file        main.c
 * @brief       智能车主程序入口
 * @version     1.2
 * @date        2023-10-27
 * @author      (Your Name)
 *
 * @note
 * V1.0: 初始框架
 * V1.1: 整合了电机、舵机、PID、编码器模块
 * V1.2: 修正了函数声明和调用逻辑，以解决链接器错误 (undefined reference)
 ********************************************************************************************************************/

#include "../user_code/bsp_encoder.h" // 包含了所有 encoder 模块函数的声明
#include "zf_libraries_headfile.h"
#include "pid.h"
#include "motor.h"
#include "servo.h"

// --- 全局变量定义 ---
PID_Controller g_motor_pid;             // 电机速度环PID控制器实例
volatile float g_motor_output = 0;      // PID累计输出值，必须是 volatile

/**
 * @brief      主 PIT 中断服务函数 (The Main Periodic Interrupt Handler)
 * @details    这是整个实时控制系统的核心。它由PIT定时器以固定频率调用。
 *             所有对时间敏感的任务都在这里执行，以确保控制的稳定性和实时性。
 * @param      event  (未使用) 中断事件标志
 * @param      ptr    (未使用) 用户传递的指针
 */
void main_pit_handler(uint32_t event, void *ptr)
{
    // --- 步骤 1: 调用编码器处理函数，更新当前速度值 ---
    // encoder_pit_handler 会读取编码器计数值并存入全局变量 g_motor1_speed_counts
    encoder_pit_handler(event, ptr);

    // --- 步骤 2: 执行电机PID闭环控制计算 ---
    // 2.1 读取最新的速度值
    //     get_motor1_speed() 函数会返回 g_motor1_speed_counts 的值
    float current_speed = (float)get_motor1_speed();

    // 2.2 PID 计算，得到控制增量
    float increment = PID_IncCalc(&g_motor_pid, current_speed);
    g_motor_output += increment;

    // 2.3 对输出进行限幅，防止超出电机驱动能力
    if(g_motor_output > 100.0f) g_motor_output = 100.0f;
    if(g_motor_output < -100.0f) g_motor_output = -100.0f;

    // 2.4 将最终控制量应用到电机
    set_motor_speed(1, (int)g_motor_output);

    // --- 未来可以扩展其他实时任务 ---
    // 例如：舵机角度环PID控制
    // 例如：IMU姿态解算
}


/**
 * @brief      主函数
 * @details    程序执行的入口。负责系统初始化、模块配置、启动实时中断，
 *             并进入主循环处理非实时任务。
 */
int main (void)
{
    // ---- 1. 系统级初始化 ----
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    debug_init();

    // ---- 2. 自定义模块初始化 ----
    motor_init();
    servo_init();
    encoder_init();

    // ---- 3. PID 控制器初始化 ----
    // 正确调用 PID_Init，传入 Kp, Ki, Kd 三个参数
    PID_Init(&g_motor_pid, 1.0f, 0.2f, 0.1f); // 这里的参数需要反复调试

    // ---- 4. 设置初始任务目标 ----
    // 设定电机的目标速度。单位是“每个PID周期内的编码器计数值”
    PID_SetPoint(&g_motor_pid, 50.0f); // 假设目标是每10ms转50个脉冲

    // ---- 5. 启动实时控制核心 ----
    // 初始化并启动PIT定时器，周期设为10ms (100Hz)，中断回调函数为 main_pit_handler
    // 最后一个参数 NULL 表示不向中断传递额外的用户数据指针
    zf_pit_ms_init(PIT_TIM6, 10, main_pit_handler, NULL);

    printf("Smart Car System Initialized. PID Control Running...\r\n");

    // ---- 6. 进入主循环 (后台任务) ----
    for( ; ; )
    {
        // 主循环可以做一些低优先级的、非实时的任务
        // 比如，通过串口打印调试信息

        // 注意：在高速PID控制下，主循环里的长延时要慎用
        // 下面的舵机控制仅为测试，实际应用中应由决策算法驱动
        set_servo_angle(80);
        zf_delay_ms(500);
        set_servo_angle(100);
        zf_delay_ms(500);

        // 推荐的调试方法：
        // printf("Target:%.1f, Speed:%d, Output:%.1f\r\n", g_motor_pid.SetPoint, get_motor1_speed(), g_motor_output);
        // zf_delay_ms(100); // 每100ms打印一次状态
    }
}
