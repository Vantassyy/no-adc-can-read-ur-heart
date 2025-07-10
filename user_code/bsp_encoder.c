// =======================================================
// 文件: user_code/encoder.c (优化后版本)
// =======================================================
#include "bsp_encoder.h"
#include "zf_libraries_headfile.h"

/**
 * @brief  编码器模块初始化
 * @note   初始化硬件编码器接口
 */
void encoder_init(void) {
    // 初始化编码器接口，假设电机1使用TIM2
    zf_encoder_init(ENCODER_TIM2, ENCODER_MODE_QUADRATURE, ENCODER_TIM2_A_PLUS_D14, ENCODER_TIM2_B_DIR_D15);
}

/**
 * @brief  获取并清零电机1的编码器增量计数
 * @note   这是一个原子操作，用于在控制中断中一次性完成读取和复位。
 * @return int16_t: 自上次调用以来的编码器脉冲增量
 */
int16_t encoder_get_and_clear_counts(void) {
    int16_t current_counts;

    // 1. 读取当前的计数值
    zf_encoder_get_count(ENCODER_TIM2, &current_counts);

    // 2. 清零硬件计数器，为下一次计数做准备
    zf_encoder_clear_count(ENCODER_TIM2);

    // 3. 返回本次读取到的计数值
    return current_counts;
}
