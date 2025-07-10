// =======================================================
// 文件: user_code/speed_control.c
// 版本: V2.1 - [AI-MOD] 为直接串口分析优化了打印格式
// =======================================================
#include "speed_control.h"
#include "motion_control.h"
#include "bsp_encoder.h"
#include "pid.h"
#include "zf_libraries_headfile.h"
#include <stdio.h> // 包含标准输入输出库，以使用printf

// ================== 内部变量 ==================
static PID_Controller g_motor_pid;
static float g_current_speed_cmps = 0.0f;
static float g_target_speed_cmps = 0.0f;
static float g_motor_output = 0.0f;
static float g_counts_to_cmps_factor = 0.0f;

// ================== 内部函数 (中断服务程序) ==================

/**
 * @brief  速度闭环控制中断服务程序 (已集成串口直接分析功能)
 * @note   此函数会被PIT定时器以 CONTROL_PERIOD_MS 的周期调用
 */
static void speed_control_interrupt_handler(uint32_t event, void *ptr)
{
    (void)event; (void)ptr;

    // --- 1. 感知 (Perception) ---
    int16_t counts = encoder_get_and_clear_counts();
    counts = -counts;
    g_current_speed_cmps = (float)counts * g_counts_to_cmps_factor;

    // --- 2. 决策 (Decision) ---
    float pid_increment = PID_IncCalc(&g_motor_pid, g_current_speed_cmps);

    // --- 3. 执行 (Execution) ---
    if (!((g_motor_output >= 100.0f && pid_increment > 0) ||
          (g_motor_output <= -100.0f && pid_increment < 0)))
    {
        g_motor_output += pid_increment;
    }
    if (g_motor_output > 100.0f) { g_motor_output = 100.0f; }
    else if (g_motor_output < -100.0f) { g_motor_output = -100.0f; }

    motion_set_motor_speed_openloop((int16_t)g_motor_output, (int16_t)g_motor_output);


    // --- 4. [AI-MOD] 调试信息打印 (用于直接串口分析) ---
    // 使用一个静态计数器来降低打印频率，避免刷屏
    static int print_counter = 0;
    // 假设 CONTROL_PERIOD_MS=10ms, "10" 表示每 10*10ms = 100ms 打印一次
    if (++print_counter >= 10)
    {
        print_counter = 0;
        float error = g_target_speed_cmps - g_current_speed_cmps;

        printf("Target:%5.1f | Current:%5.1f | Error:%+6.1f | Output:%4d\n",
               g_target_speed_cmps,
               g_current_speed_cmps,
               error,
               (int)g_motor_output);
    }
}

// ================== API函数实现 ==================

// (speed_control_init, set_speed, get_current_speed 函数保持不变, 这里省略以保持简洁)
// (请确保您文件中的这些函数是完整的)
void speed_control_init(void)
{
    // 1. 初始化依赖的底层模块
    encoder_init();
    motion_control_init();

    // 2. 计算转换系数
    const float wheel_circumference_cm = (WHEEL_DIAMETER_MM / 10.0f) * 3.1415926f;
    const float total_pulses_per_rev = (float)ENCODER_PPR * 4.0f;
    const float pulses_per_wheel_rev = total_pulses_per_rev * GEAR_RATIO;
    const float cm_per_pulse = wheel_circumference_cm / pulses_per_wheel_rev;
    const float control_period_s = (float)CONTROL_PERIOD_MS / 1000.0f;
    g_counts_to_cmps_factor = cm_per_pulse / control_period_s;

    // 3. 初始化PID控制器
    PID_Init(&g_motor_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);
    speed_control_set_speed(0.0f);

    // 4. 初始化并启动单个PIT定时器，用于速度闭环控制
    zf_pit_ms_init(PIT_TIM7, CONTROL_PERIOD_MS, speed_control_interrupt_handler, NULL);
}

void speed_control_set_speed(float target_speed_cmps)
{
    g_target_speed_cmps = target_speed_cmps;
    PID_SetPoint(&g_motor_pid, g_target_speed_cmps);
}

float speed_control_get_current_speed(void)
{
    return g_current_speed_cmps;
}
