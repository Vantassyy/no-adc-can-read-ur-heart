#include "path_manager.h"
#include <math.h>
#include <string.h> // 用于 memset
#include <stdio.h>  // 用于 printf

// ================== 内部宏定义与配置 ==================
#define WAYPOINT_REACHED_THRESHOLD_M  (0.5) // [AI-COMMENT] 建议使用一个稍大的阈值增加容错性

// ================== 路径点数据 ==================
// ================== 路径点数据 ==================
// [用户自定义路径 - 共 16 个点]
static const gnss_info_struct g_path_gps[] = {
    { .latitude = 30.76816110, .longitude = 103.97817569 }, // 点 1 (起点)
    { .latitude = 30.76813875,        .longitude = 103.97813812 },        // 点 2
    { .latitude = 30.76811350, .longitude = 103.97809728 }, // 点 3
    { .latitude = 30.76807150,        .longitude = 103.97812830},        // 点 4
    { .latitude = 30.76809090, .longitude = 103.97816439 }, // 点 5
    { .latitude = 30.76812141, .longitude = 103.97814030 }, // 点 6
    { .latitude = 30.76811000, .longitude = 103.97812350 }, // 点 7
    { .latitude = 30.76809457,         .longitude = 103.97813470 }, // 点 8
    { .latitude = 30.76812649, .longitude = 103.97818855 }, // 点 9
    { .latitude = 30.76811090, .longitude = 103.97819854 }, // 点 10
    { .latitude = 30.76810225, .longitude = 103.97818045 }, // 点 11
    { .latitude = 30.76813025, .longitude = 103.97816135 }, // 点 12
    { .latitude = 30.76814810, .longitude = 103.97819700 }, // 点 13
    { .latitude = 30.76812175, .longitude = 103.97821591 }, // 点 14
    { .latitude = 30.76813095, .longitude = 103.97823565 }, // 点 15
    { .latitude = 30.76817420,        .longitude = 103.97820340 }, // 点 16 (终点)
    { .latitude = 30.76816110, .longitude = 103.97817569 },
};
#define TOTAL_WAYPOINTS (sizeof(g_path_gps) / sizeof(g_path_gps[0]))

// ================== 内部变量 ==================
static Point_t g_path_local[TOTAL_WAYPOINTS];
static gnss_info_struct g_origin_gps;
static bool g_is_initialized = false;
static volatile int g_target_waypoint_index = 1;

// [AI-MOD] 新增全局状态变量，用于标记整个导航任务是否完成
static volatile bool g_mission_completed = false;

// ================== 依赖的数学函数 ==================
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD(angle) ((angle) * 3.14159265358979323846 / 180.0)
#endif
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE(rad)   ((rad) * 180.0 / 3.14159265358979323846)
#endif
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2)
{
    const double EARTH_RADIUS = 6378137;
    double rad_latitude1 = ANGLE_TO_RAD(latitude1);
    double rad_latitude2 = ANGLE_TO_RAD(latitude2);
    double a = rad_latitude1 - rad_latitude2;
    double b = ANGLE_TO_RAD(longitude1) - ANGLE_TO_RAD(longitude2);
    double distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_latitude1) * cos(rad_latitude2) * pow(sin(b / 2), 2)));
    distance = distance * EARTH_RADIUS;
    return distance;
}
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2)
{
    double lat1_rad = ANGLE_TO_RAD(latitude1);
    double lon1_rad = ANGLE_TO_RAD(longitude1);
    double lat2_rad = ANGLE_TO_RAD(latitude2);
    double lon2_rad = ANGLE_TO_RAD(longitude2);
    double x = sin(lon2_rad - lon1_rad) * cos(lat2_rad);
    double y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon2_rad - lon1_rad);
    double angle = RAD_TO_ANGLE(atan2(x, y));
    return ((angle >= 0) ? angle : (angle + 360.0));
}

// ================== 核心API函数实现 ==================
void path_manager_init(void)
{
    if (g_is_initialized) return;
    if (TOTAL_WAYPOINTS < 2)
    {
        while(1); // 路径点少于2个，无法导航，死循环
    }

    // [AI-MOD] 在初始化时，重置任务完成状态和目标点索引
    g_mission_completed = false;
    g_target_waypoint_index = 1;

    g_origin_gps = g_path_gps[0];
    memset(g_path_local, 0, sizeof(g_path_local));
    for (size_t i = 0; i < TOTAL_WAYPOINTS; ++i)
    {
        g_path_local[i] = path_manager_gps_to_local_xy(g_path_gps[i].longitude, g_path_gps[i].latitude);
    }
    g_is_initialized = true;
}

Point_t path_manager_gps_to_local_xy(double longitude, double latitude)
{
    Point_t local_pos = {0.0, 0.0};

    // [AI-MOD] 全局变量 g_is_initialized 在 path_manager_init 中被置为 true
    // 如果没有初始化，直接返回(0,0)
    if (!g_is_initialized) return local_pos;

    // 1. 计算当前点到原点的直线距离 (单位: 米)
    double distance = get_two_points_distance(g_origin_gps.latitude, g_origin_gps.longitude, latitude, longitude);

    // 2. 计算当前点相对于原点的方位角 (以正北为0, 顺时针, 0-360度)
    double azimuth_deg = get_two_points_azimuth(g_origin_gps.latitude, g_origin_gps.longitude, latitude, longitude);

    // 3. 将方位角转换为数学角度 (以正东为0, 逆时针为正, 弧度)
    //    这个转换公式必须与 navigation.c 中对航向角的转换公式保持一致！
    double azimuth_rad = (90.0 - azimuth_deg) * (3.1415926535 / 180.0);

    // 4. 使用三角函数将极坐标 (distance, azimuth_rad) 转换为笛卡尔坐标 (x, y)
    local_pos.x = distance * cos(azimuth_rad);
    local_pos.y = distance * sin(azimuth_rad);

    return local_pos;
}

/**
 * @brief  根据车辆当前位置，更新目标航点
 * @note   [AI-MOD] 增加了对任务完成状态的判断和设置。
 */
bool path_manager_update_target_waypoint(Point_t current_pos_xy)
{
    // 如果未初始化或任务已经完成，则直接返回，不再更新航点
    if (!g_is_initialized || g_mission_completed)
    {
        return false;
    }

    Point_t start_pos = g_path_local[g_target_waypoint_index - 1];
    Point_t target_pos = g_path_local[g_target_waypoint_index];

    bool reached = false;
    // --- 切换逻辑判断 (此部分保持不变) ---
    double ab_x = target_pos.x - start_pos.x;
    double ab_y = target_pos.y - start_pos.y;
    double ap_x = current_pos_xy.x - start_pos.x;
    double ap_y = current_pos_xy.y - start_pos.y;
    double len_sq_ab = ab_x * ab_x + ab_y * ab_y;

    if (len_sq_ab < 0.000001) {
        double dist_sq = ap_x * ap_x + ap_y * ap_y;
        if (dist_sq < (WAYPOINT_REACHED_THRESHOLD_M * WAYPOINT_REACHED_THRESHOLD_M)) {
            reached = true;
        }
    } else {
        double dot_product = ap_x * ab_x + ap_y * ab_y;
        double t = dot_product / len_sq_ab;
        if (t > 1.0) {
            reached = true;
        }
    }

    if (!reached) {
        double dx = target_pos.x - current_pos_xy.x;
        double dy = target_pos.y - current_pos_xy.y;
        double distance_to_target = sqrt(dx * dx + dy * dy);
        if (distance_to_target < WAYPOINT_REACHED_THRESHOLD_M) {
            reached = true;
        }
    }

    // --- [AI-MOD] 修改后的执行切换逻辑 ---
    if (reached)
    {
        // 判断当前到达的是不是路径中的最后一个航点
        if (g_target_waypoint_index == TOTAL_WAYPOINTS - 1)
        {
            // 如果是，则标记任务完成，并且不再增加索引
            g_mission_completed = true;
            printf("Mission Completed: Reached final waypoint.\r\n");
        }
        else
        {
            // 如果不是最后一个点，则正常切换到下一个目标
            g_target_waypoint_index++;
        }
        return true; // 目标点已更新或任务已完成
    }

    return false; // 目标点未变
}

Point_t path_manager_get_target_waypoint(void)
{
    if (!g_is_initialized) return (Point_t){0, 0};
    int index = g_target_waypoint_index;
    if (index >= (int)TOTAL_WAYPOINTS)
    {
        index = TOTAL_WAYPOINTS - 1;
    }
    return g_path_local[index];
}

Point_t path_manager_get_start_waypoint(void)
{
    if (!g_is_initialized) return (Point_t){0, 0};
    int index = g_target_waypoint_index - 1;
    if (index < 0)
    {
        index = 0;
    }
    return g_path_local[index];
}

int path_manager_get_target_index(void)
{
    return g_target_waypoint_index;
}

// [AI-MOD] 新增API函数实现
bool path_manager_is_mission_completed(void)
{
    return g_mission_completed;
}
