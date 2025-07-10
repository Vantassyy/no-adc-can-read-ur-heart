/*************************************************************************************************
 * 文件名称          main_direct_analysis.c
 * 功能描述          一个简单的阶跃响应测试程序，用于直接从串口分析PID参数。
 *************************************************************************************************/
#include "zf_libraries_headfile.h"
#include "systick.h"        // [重要] 确保包含ST官方的systick头文件
#include "speed_control.h"
#include <stdio.h>

int main(void)
{
    // --- 1. 系统初始化 ---
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    systick_init(&DRV_SYSTICK);
    systick_start(&DRV_SYSTICK);
    bsp_uart_init(BSP_UART_DEBUG, 460800);

    // --- 2. 速度控制模块初始化 ---
    speed_control_init();

    // --- 3. 开始测试 ---
    systick_delay_millisec(&DRV_SYSTICK, 1000);
    printf("\n\n--- PID Direct Analysis Test ---\n");
    printf("Setting target speed to 50.0 cm/s in 2 seconds...\n");
    systick_delay_millisec(&DRV_SYSTICK, 2000);

    // 设置一个固定的目标速度
    speed_control_set_speed(50.0f);
    printf("--- Test Started. Target = 50.0 cm/s ---\n");

    // --- 4. 等待 ---
    // 所有工作都在中断中完成，主循环空闲。
    for (;;)
    {
        // 可以执行一些低优先级任务，或者保持空闲
    }
}
