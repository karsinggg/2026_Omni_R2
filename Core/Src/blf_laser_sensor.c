#include "blf_laser_sensor.h"
#include <stdint.h>

// Standard Modbus RTU CRC-16 Calculation
uint16_t Calculate_CRC16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Request_Sensor_Distance_DMA(uint8_t sensor_address, uint8_t* tx_buffer, uint8_t* rx_buffer, UART_HandleTypeDef* huart)
{
    // Construct the payload (same as before)
	tx_buffer[0] = sensor_address; // Address code (e.g., 0x01 or 0x02)
	tx_buffer[1] = 0x04;           // Function code (0x04 = Read Input Register)
	tx_buffer[2] = 0x00;           // Register start address (High Byte)
	tx_buffer[3] = 0x00;           // Register start address (Low Byte)
	tx_buffer[4] = 0x00;           // Number of registers (High Byte)
	tx_buffer[5] = 0x02;           // Number of registers (Low Byte - reading 2 registers/4 bytes)

    uint16_t crc = Calculate_CRC16(tx_buffer, 6);

    // Append CRC (Modbus sends Low Byte first, then High Byte for CRC)
    tx_buffer[6] = crc & 0xFF;
    tx_buffer[7] = (crc >> 8) & 0xFF;

    // First, set up the DMA to listen for the 9-byte reply so it's ready BEFORE we ask
    HAL_UART_Receive_DMA(huart, rx_buffer, 9);

    // Then, transmit our 8-byte request using DMA
    HAL_UART_Transmit_DMA(huart, tx_buffer, 8);
}

// Assuming you have received the 9 bytes into rx_buffer using HAL_UART_Receive
// rx_buffer = {0x01, 0x04, 0x04, Data1, Data2, Data3, Data4, CRC_L, CRC_H}
int32_t Parse_Distance(uint8_t *rx_buffer)
{
    // 1. Verify the CRC to ensure the data is not corrupted
    uint16_t received_crc = rx_buffer[7] | (rx_buffer[8] << 8);
    uint16_t calculated_crc = Calculate_CRC16(rx_buffer, 7); // Calculate CRC over first 7 bytes

    if (received_crc != calculated_crc) {
        // Handle CRC Error (Return an error code, e.g., -1)
        return -1;
    }

    // 2. Check if the function code and byte count match the expected response
    if (rx_buffer[1] == 0x04 && rx_buffer[2] == 0x04) {
        // 3. Combine the 4 data bytes into a single 32-bit integer
        // Note: Modbus transmits Big-Endian (Most Significant Byte first)
        int32_t distance_raw = (rx_buffer[3] << 24) |
                               (rx_buffer[4] << 16) |
                               (rx_buffer[5] << 8)  |
                                rx_buffer[6];

        // Sanity Check: If the value is unrealistically huge (e.g., > 100,000 mm),
        if (distance_raw > 100000) {   // it's an error code. Clamp it or return an error state.
            return -2; // to have your main task ignore this reading
        }

        return distance_raw;
    }

    return -1; // Unknown format
}

void Parse_Distance_Status(uint8_t *rx_buffer, int32_t *distance, int8_t *status)
{
    // 1. Verify the CRC to ensure the data is not corrupted
    uint16_t received_crc = rx_buffer[7] | (rx_buffer[8] << 8);
    uint16_t calculated_crc = Calculate_CRC16(rx_buffer, 7); // Calculate CRC over first 7 bytes

    if (received_crc != calculated_crc) {
        // Handle CRC Error (Return an error code, e.g., -1)
        *status = -1;
    }

    // 2. Check if the function code and byte count match the expected response
    if (rx_buffer[1] == 0x04 && rx_buffer[2] == 0x04) {
        // 3. Combine the 4 data bytes into a single 32-bit integer
        // Note: Modbus transmits Big-Endian (Most Significant Byte first)
        int32_t distance_raw = (rx_buffer[3] << 24) |
                               (rx_buffer[4] << 16) |
                               (rx_buffer[5] << 8)  |
                                rx_buffer[6];

        // Sanity Check: If the value is unrealistically huge (e.g., > 100,000 mm),
        if (distance_raw > 100000) {   // it's an error code. Clamp it or return an error state.
            *status = -2; // to have your main task ignore this reading
        }

        *distance = distance_raw;
    }

    *status = -3; // Unknown format
}
