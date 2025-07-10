#ifndef PATH_MANAGER_H_
#define PATH_MANAGER_H_

// [已修正] 添加 extern "C" 保护，强制以C语言方式链接，解决链接错误
#ifdef __cplusplus
extern "C" {
#endif

#include "zf_libraries_headfile.h" // 假设这是你的主头文件
#include <stdbool.h>

// Point_t 结构体定义
typedef struct
{
    double x;
    double y;
} Point_t;

// ================== 核心API函数声明 ==================
void path_manager_init(void);
Point_t path_manager_gps_to_local_xy(double longitude, double latitude);
bool path_manager_update_target_waypoint(Point_t current_pos_xy);
Point_t path_manager_get_target_waypoint(void);
Point_t path_manager_get_start_waypoint(void);
int path_manager_get_target_index(void);

// [AI-MOD] 新增API函数声明，用于查询导航任务是否完成
/**
 * @brief  查询导航任务是否已经完成。
 * @param  None
 * @return bool: 如果小车已经成功通过最后一个航点，则返回 true，否则返回 false。
 */
bool path_manager_is_mission_completed(void);


// ================== [已修正] 添加缺失的数学函数声明 ==================
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2);
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2);


#ifdef __cplusplus
}
#endif

#endif /* PATH_MANAGER_H_ */
