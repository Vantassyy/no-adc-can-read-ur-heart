/*
 * navigation.c
 *
 * [文件说明]
 * 本文件实现了智能循迹小车的核心导航算法。
 *
 * [版本说明]
 * AI 优化建议版 V3.1 - Dynamic Pure Pursuit
 * - 核心算法: 使用Pure Pursuit（纯跟踪）算法，以获得更平滑的弯道轨迹。
 * - 核心优化: 实现动态前瞻距离(Ld)，Ld会根据车速自动调整，以兼顾高速稳定性和低速精确性。
 * - 鲁棒性设计: 包含任务完成、RTK信号失效的停车保护，并对动态Ld进行了上下限约束。
 */

// 包含所有必要的头文件
#include "navigation.h"      // 自身模块的API声明
#include "bsp_rtk.h"         // RTK定位模块的接口
#include "path_manager.h"    // 路径管理模块的接口
#include "motion_control.h"  // 运动控制模块的接口
#include "speed_control.h"   // 速度闭环控制模块的接口
#include <math.h>            // C语言标准数学库
#include <stdio.h>           // C语言标准输入输出库

// ====================================================================
// 内部宏定义与配置参数
// ====================================================================

// --- Pure Pursuit 算法核心参数 ---
// [!!!请务必测量并修改!!!] 小车的前后轮轴距，单位：米。这是算法必需的关键物理参数。
#define VEHICLE_WHEELBASE   (0.22f)

// --- 动态前瞻距离 (Ld) 的配置参数 ---
// [核心调试参数#1] 速度增益系数 k (单位:秒)。表示预瞄前方多少秒路程的点。
#define LD_GAIN_K           (0.0f)//临时为0
// [核心调试参数#2] 基础前瞻距离 (单位:米)。小车在极低速时的最小预瞄距离。
#define LD_BASE             (0.8f)
// [安全保护] 对计算出的Ld进行限幅，增加鲁棒性
#define LD_MAX              (1.0f) // 最大前瞻距离，防止高速时看得太远导致切弯过大
#define LD_MIN              (0.25f)// 最小前瞻距离，防止低速时Ld过小导致震荡

// ---- 速度规划参数 ----
#define MAX_SPEED_CMPS      (50.0f) // 可以适当提高速度上限来测试动态Ld的效果
#define MIN_SPEED_CMPS      (20.0f)
#define SPEED_P_GAIN        (1.5f)  // 转向角越大，速度降低越多的比例系数

// ---- 舵机物理限制 ----
#define SERVO_ANGLE_MAX     (30.0f)
#define SERVO_ANGLE_MIN     (-30.0f)

// ====================================================================
// 内部变量定义
// ====================================================================
static float g_steering_output = 0.0f; // 存储最终计算出的舵机转向角度

// ====================================================================
// 内部辅助函数定义
// ====================================================================

/**
 * @brief  在路径段(a, b)上寻找距离车辆(p)为Ld的预瞄点。
 * @note   该函数通过解一元二次方程来寻找以车辆为圆心、Ld为半径的圆与目标路径段的交点。
 * @param  p: 车辆当前位置
 * @param  a: 路径段起点
 * @param  b: 路径段终点
 * @param  ld: 前瞻距离 (Lookahead Distance)
 * @param  lookahead_pt: (输出参数) 找到的预瞄点将存放在此指针指向的地址
 * @return bool: 如果成功找到一个在线段内的预瞄点，返回true；否则返回false。
 */
static bool find_lookahead_point(Point_t p, Point_t a, Point_t b, float ld, Point_t* lookahead_pt)
{
    Point_t vec_ab = {b.x - a.x, b.y - a.y};
    Point_t vec_ap = {p.x - a.x, p.y - a.y};

    // 解一元二次方程 a*t^2 + b*t + c = 0, 其中t是预瞄点在向量AB上的比例系数
    double A = vec_ab.x * vec_ab.x + vec_ab.y * vec_ab.y;
    double B = 2 * (vec_ap.x * vec_ab.x + vec_ap.y * vec_ab.y);
    double C = vec_ap.x * vec_ap.x + vec_ap.y * vec_ap.y - ld * ld;
    double discriminant = B * B - 4 * A * C;

    if (discriminant < 0) {
        return false; // 圆和直线没有交点
    }

    discriminant = sqrt(discriminant);
    double t1 = (-B + discriminant) / (2 * A);
    double t2 = (-B - discriminant) / (2 * A);

    // 优先选择在车辆前进方向上的、且在线段内的交点 (0 <= t <= 1)
    if (t1 >= 0 && t1 <= 1) {
        lookahead_pt->x = a.x + t1 * vec_ab.x;
        lookahead_pt->y = a.y + t1 * vec_ab.y;
        return true;
    }
    if (t2 >= 0 && t2 <= 1) {
        lookahead_pt->x = a.x + t2 * vec_ab.x;
        lookahead_pt->y = a.y + t2 * vec_ab.y;
        return true;
    }

    return false; // 两个交点都不在线段ab上
}

// ====================================================================
// API函数实现
// ====================================================================

/**
 * @brief  导航模块初始化。
 */
void navigation_init(void)
{
    path_manager_init();
    g_steering_output = 0.0f; // 初始化舵机角度为0
}

/**
 * @brief  执行一次核心导航与控制计算。
 */
void navigation_run_once(const gnss_info_struct* rtk_info)
{
    // --- 第一部分：安全与状态检查 ---
    if (path_manager_is_mission_completed()) {
        speed_control_set_speed(0);
        motion_set_servo_angle(0.0f);
        return;
    }
    if (rtk_info->state == 0) {
        speed_control_set_speed(0);
        motion_set_servo_angle(0.0f);
        return;
    }

    // --- 第二部分：数据准备与动态Ld计算 ---
    Point_t current_pos = path_manager_gps_to_local_xy(rtk_info->longitude, rtk_info->latitude);
    path_manager_update_target_waypoint(current_pos);
    Point_t start_waypoint = path_manager_get_start_waypoint();
    Point_t target_waypoint = path_manager_get_target_waypoint();

    // [依赖确认] 假设 speed_control_get_current_speed() 返回单位为 cm/s 的当前速度。
    float current_speed_mps = speed_control_get_current_speed() / 100.0f; // 转换为 m/s
    float lookahead_dist_dynamic = LD_GAIN_K * current_speed_mps + LD_BASE;

    // 对计算出的动态Ld进行限幅
    if (lookahead_dist_dynamic > LD_MAX) lookahead_dist_dynamic = LD_MAX;
    else if (lookahead_dist_dynamic < LD_MIN) lookahead_dist_dynamic = LD_MIN;

    // --- 第三部分：Pure Pursuit 转向决策 ---
    Point_t lookahead_point;
    if (find_lookahead_point(current_pos, start_waypoint, target_waypoint, lookahead_dist_dynamic, &lookahead_point))
    {
        // --- 核心算法修正开始 ---

        // 1. 获取车辆当前航向角 (Yaw)
        //    优先使用双天线航向角 (antenna_direction)，因为它在静止时也准确。
        //    需要检查其状态位 antenna_direction_state。
        //    航向角的单位是度，参考系是真北。
        float heading_deg = 0.0f;
        if (rtk_info->antenna_direction_state == 1) {
            heading_deg = rtk_info->antenna_direction;
        } else {
            // 如果双天线测向无效，使用备用的地面航向。
            // 注意：地面航向在低速或静止时可能不准。
            heading_deg = rtk_info->direction;
        }

        // 2. 将航向角从“度”转换为“弧度”。
        //    我们的局部坐标系Y轴正方向是正北，X轴正方向是正东。
        //    RTK的航向角以真北为0度，顺时针增加。
        //    这与数学坐标系（X轴正向为0度，逆时针增加）存在差异。
        //    转换公式: rad = (90 - deg) * PI / 180
        double heading_rad = (90.0 - heading_deg) * (3.1415926535 / 180.0);
        // 对结果进行归一化，确保在 -PI 到 PI 之间 (可选但推荐)
        heading_rad = atan2(sin(heading_rad), cos(heading_rad));

        // 3. 计算预瞄点在世界坐标系下的方位角
        double angle_to_lookahead_point_rad = atan2(lookahead_point.y - current_pos.y, lookahead_point.x - current_pos.x);

        // 4. 计算车辆坐标系下的相对夹角 alpha
        //    alpha = 世界坐标系下预瞄点方向 - 世界坐标系下车头方向
        double alpha = angle_to_lookahead_point_rad - heading_rad;
        // 归一化alpha到 -PI 到 PI 范围
        if (alpha > 3.1415926535) alpha -= 2 * 3.1415926535;
        if (alpha < -3.1415926535) alpha += 2 * 3.1415926535;

        // 5. 应用Pure Pursuit公式计算转向角 (这里使用修正后的alpha)
        //    注意：这里的sin函数接收的是弧度制的alpha，这是正确的。
        double steering_rad = atan2(2.0 * VEHICLE_WHEELBASE * sin(alpha), lookahead_dist_dynamic);

        // 6. 将计算出的舵机弧度角转换为角度
        g_steering_output = (float)(steering_rad * 180.0 / 3.1415926535);

        // --- 核心算法修正结束 ---
    }
    else
    {
        // ... (未找到预瞄点的备用策略保持不变) ...
        // 注意：这里的备用策略也应该使用航向角进行修正，逻辑同上。
        // 为了简化，我们先聚焦主逻辑。当主逻辑验证通过后，可以再优化这里。

        // --- 备用策略修正建议（可选，可后续优化） ---
        // 1. 获取航向角 (同上)
        float heading_deg = 0.0f;
        if (rtk_info->antenna_direction_state == 1) heading_deg = rtk_info->antenna_direction;
        else heading_deg = rtk_info->direction;
        double heading_rad = (90.0 - heading_deg) * (3.1415926535 / 180.0);
        heading_rad = atan2(sin(heading_rad), cos(heading_rad));

        // 2. 计算目标点在世界坐标系下的方位角
        double angle_to_target_rad = atan2(target_waypoint.y - current_pos.y, target_waypoint.x - current_pos.x);

        // 3. 计算相对夹角
        double relative_angle_rad = angle_to_target_rad - heading_rad;
        // 归一化
        if (relative_angle_rad > 3.1415926535) relative_angle_rad -= 2 * 3.1415926535;
        if (relative_angle_rad < -3.1415926535) relative_angle_rad += 2 * 3.1415926535;

        // 4. 将相对角度直接作为转向角（这是一种简单的比例控制）
        g_steering_output = (float)(relative_angle_rad * 180.0 / 3.1415926535);
    }


    // [!!!请务必测试并校准!!!] 根据你的舵机安装方向，可能需要对计算出的角度取反。
    // 如果上车测试时转向方向错误（例如应左转却右转），请将下面这行代码注释掉，反之则保留。
    g_steering_output = -g_steering_output;

    // --- 第四部分：执行控制指令 ---
    // 对最终计算出的转向角进行物理限幅
    if (g_steering_output > SERVO_ANGLE_MAX) g_steering_output = SERVO_ANGLE_MAX;
    else if (g_steering_output < SERVO_ANGLE_MIN) g_steering_output = SERVO_ANGLE_MIN;

    // [依赖确认] 假设 motion_set_servo_angle() 接收的是角度值。
    motion_set_servo_angle(g_steering_output);
    //*************************************************************************
    float target_speed = 20.0f; // 定义一个名为 target_speed 的局部变量并赋值
    speed_control_set_speed(20.0f); // 固定为20 cm/s
    // 动态速度规划：转角越大，速度越慢
    //************************************************************************
    /*
    float target_speed = MAX_SPEED_CMPS - SPEED_P_GAIN * fabs(g_steering_output);
    if (target_speed < MIN_SPEED_CMPS) target_speed = MIN_SPEED_CMPS;
    speed_control_set_speed(target_speed);
    */
    //**********************************************************************
    // --- 第五部分：调试信息输出 ---
    printf("Idx:%d, Ld:%.2f, Steer:%.1f, Spd:%.1f\r\n",
           path_manager_get_target_index(),
           lookahead_dist_dynamic,
           g_steering_output,
           target_speed);
}
