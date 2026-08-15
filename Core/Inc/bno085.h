/*
 * bno085.h
 *
 *  Created on: Mar 1, 2026
 *      Author: Acer
 */

#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// I2C Address: Default 0x4A[cite: 261], shifted left by 1 for STM32 HAL library
#define BNO085_I2C_ADDR (0x4A << 1)

// SHTP Channels
#define SHTP_CHANNEL_COMMAND 2
#define SHTP_CHANNEL_REPORTS 3

// Report IDs
#define REPORT_GAME_ROTATION_VECTOR 0x08

// Q-Point for Rotation Vectors is 14 (1 / 2^14)
#define ROTATION_VECTOR_Q_POINT (16384.0f)

typedef struct {
    float real;
    float i;
    float j;
    float k;
} bno085_quat_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} bno085_euler_t;

// Function Prototypes
bool BNO085_ReadPacket(I2C_HandleTypeDef *hi2c, uint8_t *rx_buffer, uint16_t buffer_size, uint16_t *packet_length);
bool BNO085_EnableGameRotationVector(I2C_HandleTypeDef *hi2c, uint16_t report_interval_us);
bool BNO085_ParseRotationVector(uint8_t *payload, bno085_quat_t *quat);
void BNO085_QuaternionToEuler(bno085_quat_t *q, bno085_euler_t *euler);

#endif /* INC_BNO085_H_ */
