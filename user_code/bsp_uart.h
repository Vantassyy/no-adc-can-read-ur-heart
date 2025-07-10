/*
 * uart.h
 *
 *  Created on: 2025年7月7日
 *      Author: 20766
 */

#ifndef USER_CODE_BSP_UART_H_
#define USER_CODE_BSP_UART_H_


#include "zf_driver_uart.h"
#include "zf_common_fifo.h" // 因为我们用到了 zf_fifo_obj_struct
#include <stdbool.h>

// ================== 配置与宏定义 ==================

// 定义我们系统中用到的UART逻辑设备
// 这样上层代码就不需要关心具体是UART_1还是UART_2了
typedef enum
{
    BSP_UART_DEBUG,     // 用于调试打印的串口 (对应 StellarLINK 的虚拟串口)
    BSP_UART_RTK,       // 用于连接RTK模块的串口

    BSP_UART_NUM_MAX    // 系统中UART的总数
} bsp_uart_e;

// 定义每个UART接收缓冲区的大小 (单位: 字节)
// 这个值应该足够大，以防止在主循环处理不及时的情况下数据丢失
// 对于高波特率的RTK，建议至少512字节或更大
#define BSP_UART_DEBUG_RX_BUF_SIZE   (256)
#define BSP_UART_RTK_RX_BUF_SIZE     (1024)

// ================== API函数声明 ==================

/**
 * @brief  初始化指定的UART通道
 * @param  uart_ch: 要初始化的逻辑UART通道 (BSP_UART_DEBUG 或 BSP_UART_RTK)
 * @param  baudrate: 波特率
 * @retval None
 */
void bsp_uart_init(bsp_uart_e uart_ch, uint32_t baudrate);

/**
 * @brief  向指定的UART通道发送一个字节
 * @param  uart_ch: 目标UART通道
 * @param  data: 要发送的字节数据
 * @retval None
 */
void bsp_uart_write_byte(bsp_uart_e uart_ch, uint8_t data);

/**
 * @brief  向指定的UART通道发送一个字节数组
 * @param  uart_ch: 目标UART通道
 * @param  buffer: 要发送的数据缓冲区指针
 * @param  length: 要发送的数据长度
 * @retval None
 */
void bsp_uart_write_buffer(bsp_uart_e uart_ch, const uint8_t* buffer, uint32_t length);

/**
 * @brief  从指定的UART通道读取一个字节
 * @param  uart_ch: 目标UART通道
 * @param  data: 用于接收数据的指针
 * @retval bool: true-成功读取到一个字节, false-接收缓冲区为空
 */
bool bsp_uart_read_byte(bsp_uart_e uart_ch, uint8_t* data);


#endif /* USER_CODE_BSP_UART_H_ */
