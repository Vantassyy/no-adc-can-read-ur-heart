/*
 * bsp_encoder.h
 *
 *  Created on: 2025年7月6日 (修改于新日期)
 *      Author: 20766 (及AI助手)
 *
 *  [版本说明] 此版本已根据系统架构优化进行了接口调整。
 *            中断处理和速度获取逻辑已移至 speed_control 模块。
 */

#ifndef USER_CODE_BSP_ENCODER_H_
#define USER_CODE_BSP_ENCODER_H_

#include "zf_common_typedef.h" // 包含逐飞的类型定义

//-------------------------------------------------------------------------------------------------------------------
// 函数原型声明 (Function Prototypes)
// 这里只声明本模块向外提供的公共函数。
//-------------------------------------------------------------------------------------------------------------------

/**
 * @brief  编码器模块初始化
 * @note   初始化硬件编码器接口。
 * @param  None
 * @retval None
 */
void encoder_init(void);

/**
 * @brief  获取并清零电机1的编码器增量计数
 * @note   这是一个原子操作，用于在控制中断中一次性完成读取和复位。
 *         它直接读取硬件定时器的计数值，然后将其清零。
 * @param  None
 * @retval int16_t: 自上次调用以来的编码器脉冲增量。
 */
int16_t encoder_get_and_clear_counts(void);


/*
 * [已移除] 以下函数和变量声明已被移除，因为其功能已合并到 speed_control 模块中，
 * 不再作为 encoder 模块的公共接口。
 *
 * void encoder_pit_handler(uint32_t event, void *ptr);
 * int16_t get_motor1_speed(void);
 * extern volatile int16_t g_motor1_speed_counts;
 *
 */

#endif /* USER_CODE_BSP_ENCODER_H_ */
