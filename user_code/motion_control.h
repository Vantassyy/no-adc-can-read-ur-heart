#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

// 使用 extern "C" 保护，解决潜在的C/C++混合编译链接问题
#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_PWM.h"
#include <stdint.h> // 明确包含

// ================== 宏定义与配置 ==================
// [确认] 这些是你通过标定得到的精确值，保持不变
#define SERVO_ANGLE_MAX       (30.0f)
#define SERVO_ANGLE_MIN       (-30.0f)
#define SERVO_DUTY_MAX_US     (1900)
#define SERVO_DUTY_NEUTRAL_US (1550)
#define SERVO_DUTY_MIN_US     (1200)
#define SERVO_PWM_FREQ_HZ     (50)
#define MOTOR_PWM_FREQ_HZ     (17000)

// ================== API函数声明 ==================

void motion_control_init(void);
void motion_set_servo_angle(float angle);

/**
 * @brief 设置左右轮的目标速度。
 * @param left_speed:  左轮速度 (-100 到 +100)。
 * @param right_speed: 右轮速度 (-100 到 +100)。
 * @note  在你的单电机驱动模型中，此函数会取两个速度的平均值来控制电机，
 *        并使用差值来辅助转向（如果需要的话，目前简化处理）。
 *        【关键】函数名和参数列表已修改，以匹配 speed_control.c 的调用。
 */
void motion_set_motor_speed_openloop(int16_t left_speed, int16_t right_speed);

void motion_set_motor_speed_closedloop(float left_target_speed, float right_target_speed);


#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_H_ */
