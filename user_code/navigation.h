#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#ifdef __cplusplus
extern "C" {
#endif

// 依赖 bsp_rtk.h 来获取 'gnss_info_struct' 类型的定义。
#include "bsp_rtk.h"

/**
 * @brief  导航模块初始化。
 * @note   在系统启动时调用一次，用于初始化导航算法。
 * @param  None
 * @retval None
 */
void navigation_init(void);

/**
 * @brief  执行一次核心导航与控制计算。
 * @note   在一个固定的周期性任务中调用此函数（例如，每100ms一次）。
 * @param  rtk_info 指向包含最新RTK定位信息的结构体的指针。
 * @retval None
 */
void navigation_run_once(const gnss_info_struct* rtk_info);


#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H_ */
