/*********************************************************************************************************************
 * 文件名称          main_coord_heading_test.c
 * 功能描述          坐标系与航向角统一性验证程序
 * 版本信息          V1.0
 * 开发环境          StellarStudio
 * 适用平台          Stellar-SR5E1E3 (基于您的项目代码)
 * 使用方法          1. 烧录程序，将小车放置在室外开阔地带。
 *                   2. 打开串口助手，等待RTK定位成功并打印 "Origin Set"。
 *                   3. 将小车保持车头朝向正东，手动向正东方向移动5米左右。
 *                   4. 观察串口打印的 heading_rad_calc 和 pos_azimuth_rad_calc 是否都接近0。
 *                   5. 保持小车位置不变，将车头旋转至正北，观察heading_rad_calc是否接近1.57。
 ********************************************************************************************************************/

#include "zf_libraries_headfile.h"
#include <math.h> // 包含数学库以使用 M_PI

// 包含我们自己编写的所有核心模块头文件
#include "bsp_uart.h"
#include "bsp_rtk.h"
#include "path_manager.h" // 需要使用其坐标转换函数和内部数学函数
#include "navigation.h"   // 只是为了让编译通过，实际不使用其循迹功能

// 如果math.h中没有M_PI, 取消下面的注释
// #ifndef M_PI
// #define M_PI (3.14159265358979323846)
// #endif


// ================== 全局变量 ==================
static bool g_origin_is_set = false;
static gnss_info_struct g_origin_gps_for_test;

// ================== 辅助函数 (直接从 path_manager.c 复制过来，确保独立性) ==================
// 注意：为了让这个测试文件能独立工作，我们把坐标转换的核心逻辑复制一份在这里。
// 这样就不需要初始化整个 path_manager。

double test_get_two_points_distance(double lat1, double lon1, double lat2, double lon2)
{
    const double EARTH_RADIUS = 6378137.0;
    double rad_lat1 = ANGLE_TO_RAD(lat1);
    double rad_lat2 = ANGLE_TO_RAD(lat2);
    double a = rad_lat1 - rad_lat2;
    double b = ANGLE_TO_RAD(lon1) - ANGLE_TO_RAD(lon2);
    double distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_lat1) * cos(rad_lat2) * pow(sin(b / 2), 2)));
    return distance * EARTH_RADIUS;
}

Point_t test_gps_to_local_xy(double longitude, double latitude)
{
    Point_t local_pos = {0.0, 0.0};
    if (!g_origin_is_set) return local_pos;

    // 计算X (东西方向距离)
    local_pos.x = test_get_two_points_distance(g_origin_gps_for_test.latitude, g_origin_gps_for_test.longitude, g_origin_gps_for_test.latitude, longitude);
    if (longitude < g_origin_gps_for_test.longitude) local_pos.x = -local_pos.x;

    // 计算Y (南北方向距离)
    local_pos.y = test_get_two_points_distance(g_origin_gps_for_test.latitude, g_origin_gps_for_test.longitude, latitude, g_origin_gps_for_test.longitude);
    if (latitude < g_origin_gps_for_test.latitude) local_pos.y = -local_pos.y;

    return local_pos;
}


// ================== 主函数 ==================

int main(void)
{
    // 1. 系统级初始化
    zf_system_clock_init(SYSTEM_CLOCK_300M);
    bsp_uart_init(BSP_UART_DEBUG, 460800);

    // 2. 仅初始化RTK模块
    bsp_rtk_init();

    // 3. 打印启动信息和操作指引
    printf("\r\n============================================\r\n");
    printf("= Coordinate System & Heading Test         =\r\n");
    printf("============================================\r\n");
    printf("Waiting for RTK fix to set origin point...\r\n");

    // 4. 主循环
    for (;;)
    {
        // 检查并解析新的RTK数据
        if (bsp_rtk_data_task())
        {
            gnss_info_struct rtk_info = bsp_rtk_get_info();

            // 确保RTK状态有效 (1=单点, 2=差分, 4=固定解, 5=浮点解)
            // 并且双天线测向有效
            if (rtk_info.state > 0 && rtk_info.antenna_direction_state == 1)
            {
                // 如果原点尚未设定，则使用第一个有效点作为原点
                if (!g_origin_is_set)
                {
                    g_origin_gps_for_test = rtk_info;
                    g_origin_is_set = true;
                    printf("\r\n--- Origin Set! ---\r\n");
                    printf("Origin Lat: %lf, Lon: %lf\r\n", g_origin_gps_for_test.latitude, g_origin_gps_for_test.longitude);
                    printf("Please move the vehicle EAST and observe the data.\r\n\r\n");
                }

                // ---- 数据计算 ----

                // 1. 获取当前局部坐标
                Point_t current_pos = test_gps_to_local_xy(rtk_info.longitude, rtk_info.latitude);

                // 2. 获取RTK原始航向角 (以北为0, 顺时针)
                float heading_deg = rtk_info.antenna_direction;

                // 3. 计算我们代码中使用的航向角 (以东为0, 逆时针为正, 弧度)
                double heading_rad_calc = (90.0 - heading_deg) * (M_PI / 180.0);
                heading_rad_calc = atan2(sin(heading_rad_calc), cos(heading_rad_calc)); // 归一化

                // 4. 计算当前位置向量的方位角 (以东为0, 逆时针为正, 弧度)
                double pos_azimuth_rad_calc = atan2(current_pos.y, current_pos.x);

                // ---- 打印结果 ----
                printf("Heading(deg):%6.1f | Pos(x,y):[%6.2f, %6.2f] | Heading(rad):% 6.2f | PosAzimuth(rad):% 6.2f\r",
                       heading_deg,
                       current_pos.x,
                       current_pos.y,
                       heading_rad_calc,
                       pos_azimuth_rad_calc);
            }
        }

        // 延时以降低打印频率
        zf_delay_ms(100);
    }
}
