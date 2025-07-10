/*
 * main_point_collector.c
 *
 *  Created on: 2025年7月9日
 *      Author: 20766
 */

/*********************************************************************************************************************
 * 文件名称          main_point_collector.c
 * 功能描述          一个用于智能车RTK路径点采集的专用工具程序。
 *                   它会通过串口持续输出车辆当前从RTK模块获取的原始经纬度坐标，
 *                   以及经过path_manager转换后的本地XY坐标。
 * 使用方法          将此文件设为编译目标并烧录至小车。通过串口助手查看输出，
 *                   手动将小车放置在赛道上的各个关键路径点，并记录下对应的坐标值。
 * 版本信息          V1.0
 ********************************************************************************************************************/

#include "zf_libraries_headfile.h"

// === 需要包含的核心模块 ===
#include "bsp_uart.h"       // 用于初始化串口和 printf 输出
#include "bsp_rtk.h"        // 用于初始化和获取RTK数据
#include "path_manager.h"   // [关键] 用于调用坐标转换函数，并将路径原点作为参考

// === 注意：我们不需要以下模块，因为小车是静止的 ===
// #include "navigation.h"
// #include "speed_control.h"
// #include "motion_control.h"
// #include "pid.h"

// ================== 主函数 ==================

int main(void)
{
    // 1. 系统基础初始化
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    bsp_uart_init(BSP_UART_DEBUG, 460800); // 初始化用于打印的串口

    // 2. 打印欢迎和说明信息
    printf("\r\n\r\n");
    printf("============================================\r\n");
    printf("=         RTK Path Point Collector         =\r\n");
    printf("============================================\r\n");

    // 3. 初始化核心模块
    bsp_rtk_init();       // 初始化RTK模块
    path_manager_init();  // [关键] 初始化路径管理器，这会设定坐标转换的“原点”

    printf("Initialization complete. Waiting for RTK data...\r\n");
    printf("Output Format: FixState, Latitude, Longitude, X_coord(m), Y_coord(m)\r\n");
    printf("----------------------------------------------------------------------\r\n");

    // 4. 进入主循环，持续采集和输出数据
    for (;;)
    {
        // 检查RTK模块是否有新的、有效的数据
        if (bsp_rtk_data_task())
        {
            // 获取最新的RTK信息
            gnss_info_struct rtk_info = bsp_rtk_get_info();

            // 将原始经纬度坐标转换为本地XY坐标
            Point_t local_xy = path_manager_gps_to_local_xy(rtk_info.longitude, rtk_info.latitude);

            // 通过串口打印所有信息
            // - rtk_info.state: 定位状态 (1代表有效固定解, 其他值可能代表浮点解或无效)
            // - 经纬度建议保留至少8位小数以保证精度
            // - XY坐标保留3位小数（毫米级精度）即可
            printf("State: %u, 经度: %.8f, 纬度: %.8f, X: %.3f, Y: %.3f\r\n",
                   rtk_info.state,
                   rtk_info.latitude,
                   rtk_info.longitude,
                   local_xy.x,
                   local_xy.y);
        }

        // 延时一段时间，避免串口信息刷新过快，也给RTK模块处理时间
        // 500ms的刷新率对于手动采点来说非常合适
        zf_delay_ms(500);
    }
}
