/*
 * ano_protocol.h
 *
 *  Created on: 2025年7月9日
 *      Author: 20766
 */

#ifndef USER_CODE_ANO_PROTOCOL_H_
#define USER_CODE_ANO_PROTOCOL_H_


#include <stdint.h>

// [AI-COMMENT] 这是匿名协议最核心的几个数据发送函数。
// 你的库里很可能已经包含了它们。我们主要会用到 ANO_DT_Send_F1。

/**
 * @brief  使用匿名协议发送传感器数据（通常是加速度计、陀螺仪、磁力计）
 * @param  acc_x, acc_y, acc_z: 加速度计三轴数据
 * @param  gyro_x, gyro_y, gyro_z: 陀螺仪三轴数据
 * @param  mag_x, mag_y, mag_z: 磁力计三轴数据
 */
void ANO_DT_Send_Senser(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z);

/**
 * @brief  使用匿名协议发送姿态数据（欧拉角）
 * @param  roll, pitch, yaw: 横滚角、俯仰角、航向角
 */
void ANO_DT_Send_Tait(int16_t roll, int16_t pitch, int16_t yaw);

/**
 * @brief  使用匿名协议发送用户自定义数据帧 (F1)
 * @note   这是我们调试时最常用的函数！它可以发送4个float类型的数据。
 *         这4个数据会对应到匿名上位机“自定义帧”界面的Data1, Data2, Data3, Data4。
 * @param  data1, data2, data3, data4: 你想发送的4个浮点数
 */
void ANO_DT_Send_F1(float data1, float data2, float data3, float data4);


#endif /* USER_CODE_ANO_PROTOCOL_H_ */
