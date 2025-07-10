/*
 * bsp_rtk.c
 *
 *  [版本说明] 最终优化版。
 */
#include "bsp_rtk.h"

// ================== 外部变量与函数声明 ==================
// 这些变量和函数由逐飞的 `zf_device_gnss` 库定义并导出。
extern uint8 gnss_flag;
extern gnss_info_struct gnss_info;
// gnss_init 和 gnss_data_parse 由 bsp_rtk.h 中包含的官方头文件声明，此处无需重复 extern。


// ================== API函数实现 ==================

/**
 * @brief  初始化RTK模块。
 * @note   此函数调用逐飞官方gnss_init，完成UART、FIFO和中断的全部后台配置。
 */
bool bsp_rtk_init(void)
{
    // 调用官方API，并传入RTK模组类型。
    // 根据逐飞库的习惯，返回0 (ZF_NO_ERROR) 表示成功。
    if (gnss_init(GNSS_TYPE_GN43RFA) == 0)
    {
        return false; // 初始化成功
    }
    return true; // 初始化失败
}

/**
 * @brief  检查并处理新的RTK数据。
 * @note   此函数是对您原有 `bsp_rtk_is_new_data_available` 的优化命名和封装。
 */
bool bsp_rtk_data_task(void)
{
    // 检查由UART中断在接收到换行符后置位的标志位。
    if (gnss_flag)
    {
        // 清除标志，避免重复处理。这是必须的步骤。
        gnss_flag = 0;

        // 调用解析函数。它会处理已分类的内部缓冲区(gnss_xxx_buffer)，
        // 进行校验和验证，并更新全局的 `gnss_info` 结构体。
        // 函数返回0 (ZF_NO_ERROR) 表示所有已接收的语句都校验成功并被解析。
        if (gnss_data_parse() == 0)
        {
            return true; // 确认有新数据，并且已成功更新。
        }
    }

    // 如果 gnss_flag 为 0，或 gnss_data_parse() 返回错误，都表示本周期没有有效新数据。
    return false;
}

/**
 * @brief  获取最新的RTK信息结构体。
 * @note   此函数实现保持不变。
 */
gnss_info_struct bsp_rtk_get_info(void)
{
    // 直接返回全局结构体的副本，这是最安全的方式，
    // 可以防止上层应用的数据在读取过程中被中断意外修改（虽然在此架构中风险极低）。
    return gnss_info;
}
