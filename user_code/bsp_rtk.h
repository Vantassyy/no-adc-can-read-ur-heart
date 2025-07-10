/*
 * bsp_rtk.h
 *
 *  [版本说明] 最终优化版。
 *            提供对逐飞GNSS库的标准化封装接口。
 */
#ifndef BSP_RTK_H_
#define BSP_RTK_H_

#include "zf_device_gnss.h" // 明确包含依赖的官方库头文件

// ================== API函数声明 ==================

/**
 * @brief  初始化RTK模块。
 * @note   此函数调用逐飞官方gnss_init，完成UART、FIFO和中断的全部后台配置。
 * @param  None
 * @retval bool: true-初始化失败, false-初始化成功。
 */
bool bsp_rtk_init(void);

/**
 * @brief  检查并处理新的RTK数据。
 * @note   此函数是与RTK模块交互的核心。
 *         它应该在主控制循环中被周期性调用（例如在10Hz的定时器中断中）。
 *         内部逻辑:
 *         1. 检查由UART中断置位的 `gnss_flag`。
 *         2. 若有新数据，则调用 `gnss_data_parse()` 来解析已分好类的NMEA语句，
 *            并更新全局的 `gnss_info` 结构体。
 * @param  None
 * @retval bool: true-本周期有新的、有效的数据被成功解析, false-无新数据或解析失败。
 */
bool bsp_rtk_data_task(void);

/**
 * @brief  获取最新的RTK信息结构体。
 * @note   在调用此函数前，应先通过 `bsp_rtk_data_task()` 确认有新数据。
 * @param  None
 * @return gnss_info_struct: 返回一个包含最新定位信息的结构体副本。
 *         返回副本可保证上层应用在使用数据时，数据不会被中断意外修改，更安全。
 */
gnss_info_struct bsp_rtk_get_info(void);

#endif /* BSP_RTK_H_ */
