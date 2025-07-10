/*
 * speed_control.h
 *
 *  Created on: 2025年7月8日
 *      Author: 20766
 */

#ifndef USER_CODE_SPEED_CONTROL_H_
#define USER_CODE_SPEED_CONTROL_H_


#include "bsp_encoder.h"
#include "motion_control.h"
#include "pid.h"

// ================== 配置与宏定义 ==================

// ---- 物理参数 (这是最需要你确认和修改的地方！) ----
#define ENCODER_PPR             (1024)     // [请修改] 编码器原始线数 (Pulses Per Revolution)
#define GEAR_RATIO              (4.074f)   // [请修改] 电机减速比 (例如 30:1)
#define WHEEL_DIAMETER_MM       (72.0f)   // [请修改] 车轮直径 (单位: 毫米)

// ---- 控制周期 ----
#define CONTROL_PERIOD_MS       (10)      // 控制周期/采样周期 (单位: 毫秒)，10ms是个常用值

// ---- PID参数 (需要反复调试) ----
#define MOTOR_PID_KP            (0.4f)
#define MOTOR_PID_KI            (0.15f)
#define MOTOR_PID_KD            (0.8f)

// ================== API函数声明 ==================

/**
 * @brief  初始化速度闭环控制系统
 * @param  None
 * @retval None
 */
void speed_control_init(void);

/**
 * @brief  设置电机目标速度
 * @param  target_speed_cmps: 目标速度 (单位: 厘米/秒 cm/s)
 * @retval None
 * @note   由于是单电机驱动四轮，我们只需要一个目标速度
 */
void speed_control_set_speed(float target_speed_cmps);

/**
 * @brief  获取当前计算出的电机速度
 * @return float: 当前速度 (单位: 厘米/秒 cm/s)
 */
float speed_control_get_current_speed(void);


#endif /* USER_CODE_SPEED_CONTROL_H_ */
