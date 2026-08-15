// PG6 -> INT_Pin
// PG5 -> RST pin
// PG8 -> Boot_PIN
// what is clock stretching? How to configure so that I2C1 support clock stretching

#include "bno085.h"
#include <math.h>
#include <string.h>

bool BNO085_ReadPacket(I2C_HandleTypeDef *hi2c, uint8_t *rx_buffer, uint16_t buffer_size, uint16_t *packet_length) {

    // Read 64 bytes in ONE transaction to prevent the BNO085 from shifting the buffer
    if (HAL_I2C_Master_Receive(hi2c, BNO085_I2C_ADDR, rx_buffer, 64, 50) != HAL_OK) {
        return false;
    }

    // Packet length is first two bytes. Mask out the MSB (continuation flag).
    *packet_length = (rx_buffer[0] | (rx_buffer[1] << 8)) & 0x7FFF;

    if (*packet_length == 0 || *packet_length > buffer_size) {
        return false; // Invalid or too large packet
    }

    return true;
}

/*
 * Constructs and sends a "Set Feature Command" to enable the Game Rotation Vector.
 */
bool BNO085_EnableGameRotationVector(I2C_HandleTypeDef *hi2c, uint16_t report_interval_us) {
    uint8_t command_packet[21] = {0};

    // SHTP Header
    command_packet[0] = 21; // Packet length LSB
    command_packet[1] = 0;  // Packet length MSB
    command_packet[2] = SHTP_CHANNEL_COMMAND;	// "Set Feature Command" (0xFD) must be sent over Channel 2 (SH-2 control)
    command_packet[3] = 0;  // Sequence number (can be 0)

    // Set Feature Command (Command ID 0xFD)
    command_packet[4] = 0xFD;
    command_packet[5] = REPORT_GAME_ROTATION_VECTOR;
    command_packet[6] = 0; // Feature flags
    command_packet[7] = 0; // Change sensitivity (LSB)
    command_packet[8] = 0; // Change sensitivity (MSB)

    // Report interval in microseconds (4 bytes, little endian)
    command_packet[9]  = (report_interval_us & 0xFF);
    command_packet[10] = (report_interval_us >> 8) & 0xFF;
    command_packet[11] = (report_interval_us >> 16) & 0xFF;
    command_packet[12] = (report_interval_us >> 24) & 0xFF;

    // Remaining bytes are batch interval and sensor-specific config (leave 0)

    if (HAL_I2C_Master_Transmit(hi2c, BNO085_I2C_ADDR, command_packet, 21, 50) != HAL_OK) {
        return false;
    }

    return true;
}

/*
 * Extracts the 16-bit quaternion components from the payload and applies the Q-point.
 */
bool BNO085_ParseRotationVector(uint8_t *payload, bno085_quat_t *quat) {
    // Check if this is a report channel and matches our requested report
    if (payload[2] != SHTP_CHANNEL_REPORTS || payload[9] != REPORT_GAME_ROTATION_VECTOR) {
        return false;
    }

    // SHTP payload starts at byte 4. Report ID is at byte 9. Data starts at byte 13.
    // LSB is first in the sequence.
    int16_t i    = (payload[14] << 8) | payload[13];
    int16_t j    = (payload[16] << 8) | payload[15];
    int16_t k    = (payload[18] << 8) | payload[17];
    int16_t real = (payload[20] << 8) | payload[19];

    quat->i    = (float)i / ROTATION_VECTOR_Q_POINT;
    quat->j    = (float)j / ROTATION_VECTOR_Q_POINT;
    quat->k    = (float)k / ROTATION_VECTOR_Q_POINT;
    quat->real = (float)real / ROTATION_VECTOR_Q_POINT;

    return true;
}

/*
 * Converts a quaternion to Euler angles (Roll, Pitch, Yaw) in degrees.
 */
void BNO085_QuaternionToEuler(bno085_quat_t *q, bno085_euler_t *euler) {
    float sqi = q->i * q->i;
    float sqj = q->j * q->j;
    float sqk = q->k * q->k;

    // Roll (x-axis rotation)
    euler->roll = atan2f(2.0f * (q->real * q->i + q->j * q->k), 1.0f - 2.0f * (sqi + sqj));

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        euler->pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    euler->yaw = atan2f(2.0f * (q->real * q->k + q->i * q->j), 1.0f - 2.0f * (sqj + sqk));

    // Convert radians to degrees
    euler->roll  *= (180.0f / M_PI);
    euler->pitch *= (180.0f / M_PI);
    euler->yaw   *= (180.0f / M_PI);
}
