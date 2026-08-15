/*
 * blf_laser_sensor.h
 *
 *  Created on: Mar 10, 2026
 *      Author: Acer
 */

#ifndef INC_BLF_LASER_SENSOR_H_
#define INC_BLF_LASER_SENSOR_H_

#include "main.h"

uint16_t Calculate_CRC16(uint8_t *buffer, uint16_t length);
void Request_Sensor_Distance_DMA(uint8_t sensor_address, uint8_t* tx_buffer, uint8_t* rx_buffer, UART_HandleTypeDef* huart);
int32_t Parse_Distance(uint8_t *rx_buffer);
void Parse_Distance_Status(uint8_t *rx_buffer, int32_t *distance, int8_t *status);

#endif /* INC_BLF_LASER_SENSOR_H_ */
