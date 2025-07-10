/*
 * ano_protocol.c
 *
 * [文件说明]
 * 实现了匿名科创上位机的通信协议，用于发送调试数据。
 *
 * [版本说明]
 * V1.1 - 修正版
 * - 修正了因复制粘贴导致的非法字符错误。
 * - 修正了串口发送函数的名称以匹配bsp库。
 */

#include "bsp_uart.h" // 包含bsp_uart头文件以获取函数声明和宏定义
#include <stdint.h>

// [AI-COMMENT] 这是我之前为你编写的函数，但你提供的代码中函数名可能不同。
// 我们现在创建一个新的、正确的函数，与 speed_test.c 中调用的 ANO_DT_Send_F1 匹配。
// 如果你之前已经有了这个文件，请确保函数名和参数与你实际调用的匹配。

/**
 * @brief  使用匿名协议发送用户自定义数据帧 (F1)
 * @note   这是我们调试时最常用的函数！它可以发送4个float类型的数据。
 *         我们用它来发送目标速度和当前速度。
 * @param  data1 (目标速度), data2 (当前速度), data3, data4: 你想发送的4个浮点数
 */
void ANO_DT_Send_F1(float data1, float data2, float data3, float data4)
{
    // [AI-COMMENT] 帧协议: 帧头(2) + 功能码(1) + 数据长度(1) + 数据(4*4) + 校验和(1) = 21字节
    uint8_t data_to_send[21];
    uint8_t sum_check = 0;

    // 1. 帧头
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xAF;

    // 2. 功能码 (0xF1 表示用户自定义数据)
    data_to_send[2] = 0xF1;

    // 3. 数据长度 (4个float，每个4字节，共16字节)
    data_to_send[3] = 16;

    // 4. 放置数据
    // [AI-MOD] 修正了所有错误的核心部分，使用正确的指针操作
    uint8_t *p;

    p = (uint8_t *)&data1;
    data_to_send[4] = p[0];
    data_to_send[5] = p[1];
    data_to_send[6] = p[2];
    data_to_send[7] = p[3];

    p = (uint8_t *)&data2;
    data_to_send[8] = p[0];
    data_to_send[9] = p[1];
    data_to_send[10] = p[2];
    data_to_send[11] = p[3];

    p = (uint8_t *)&data3;
    data_to_send[12] = p[0];
    data_to_send[13] = p[1];
    data_to_send[14] = p[2];
    data_to_send[15] = p[3];

    p = (uint8_t *)&data4;
    data_to_send[16] = p[0];
    data_to_send[17] = p[1];
    data_to_send[18] = p[2];
    data_to_send[19] = p[3];

    // 5. 计算校验和 (从功能码到数据结束)
    for(int i = 2; i < 20; i++)
    {
        sum_check += data_to_send[i];
    }
    data_to_send[20] = sum_check;

    // 6. 通过串口发送出去
    // [AI-MOD] 使用编译器提示的、正确的函数名 bsp_uart_write_buffer
    bsp_uart_write_buffer(BSP_UART_DEBUG, data_to_send, sizeof(data_to_send));
}
