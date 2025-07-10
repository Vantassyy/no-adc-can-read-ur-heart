/*********************************************************************************************************************
 * 文件名称          main_navigation_test.c
 * 功能描述          RTK自主导航系统的主测试程序
 * 版本信息          V1.2 (修正编译警告)
 * 开发环境          StellarStudio
 * 适用平台          Stellar-SR5E1E3 (基于逐飞开源库)
 ********************************************************************************************************************/

#include "zf_libraries_headfile.h"

// 包含我们自己编写的所有核心模块头文件
#include "bsp_uart.h"
#include "bsp_rtk.h"
#include "speed_control.h"
#include "path_manager.h"   // path_manager.h 已经被包含了，很好

// [AI-MOD] 添加此行以解决 "implicit declaration" 警告
// 因为本文件调用了 navigation_init() 和 navigation_run_once()，
// 所以必须包含它们的声明文件 navigation.h
#include "navigation.h"

// ================== 全局变量 ==================

volatile uint32_t g_navigation_tick_count = 0;

// ================== 中断服务程序 ==================

/**
 * @brief  导航控制循环的中断服务程序 (ISR - Interrupt Service Routine)
 */
void navigation_isr_callback(uint32_t event, void *ptr)
{
    (void)event;
    (void)ptr;

    // 1. 调用RTK数据任务函数，检查并解析新数据
    if (bsp_rtk_data_task())
    {
        // 2. 只有在确认有新的、有效的数据时，才执行导航计算
        gnss_info_struct rtk_info = bsp_rtk_get_info();

        // 将这份新鲜的数据传递给导航计算函数
        navigation_run_once(&rtk_info);
    }
    // 如果 bsp_rtk_data_task() 返回false，本周期内不做任何导航计算

    // 3. 中断计数器自增
    g_navigation_tick_count++;
}

// ================== 主函数 ==================

int main(void)
{
    // 1. 系统级初始化
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    // [AI-COMMENT] 你的 bsp_uart_init 函数需要一个参数，假设是 BSP_UART_DEBUG
    bsp_uart_init(BSP_UART_DEBUG, 460800);

    // 2. 运动与导航系统初始化
    bsp_rtk_init();
    speed_control_init();
    navigation_init(); // 调用 navigation_init

    // 3. 初始化并启动导航定时器中断 (10Hz)
    zf_pit_ms_init(PIT_TIM5, 100, navigation_isr_callback, NULL);

    // 4. 打印启动信息
    printf("\r\n============================================\r\n");
    printf("=       RTK Autonomous Navigation Test       =\r\n");
    printf("============================================\r\n");
    printf("System Initialized. Navigation ISR is running at 10Hz.\r\n");
    printf("Please ensure the vehicle is in a safe, open area.\r\n\r\n");

    // 5. 主循环
    for (;;)
    {
        // 可以在这里添加一些低优先级的任务，比如状态显示
        zf_delay_ms(200);
    }
}
