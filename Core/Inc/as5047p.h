/*
 * as5047p.h
 *
 *  Created on: Jan 7, 2026
 *      Author: Acer
 */

#ifndef INC_AS5047P_H_
#define INC_AS5047P_H_

#include "main.h"
#include <stdbool.h>

// AS5047P Register Addresses
#define AS5047P_NOP         0x0000
#define AS5047P_ERRFL       0x0001
#define AS5047P_PROG        0x0003
#define AS5047P_DIAAGC      0x3FFC
#define AS5047P_MAG         0x3FFD
#define AS5047P_ANGLEUNC    0x3FFE  // Raw angle without dynamic compensation
#define AS5047P_ANGLECOM    0x3FFF  // Angle with dynamic compensation

// Command Modifiers
#define AS5047P_READ        0x4000  // Bit 14 is 1 for Read
#define AS5047P_WRITE       0x0000  // Bit 14 is 0 for Write

// Pin Definitions (Change these to match your CubeMX setup)
//#define AS5047P_SPI3_HANDLE   &hspi3
//#define AS5047P_SPI4_HANDLE   &hspi4

// --- Diagnostic Structure ---
typedef struct {
    uint8_t AGC_Value;      // 0 to 255 (Ideal is ~128)
    uint8_t MAGL;           // 1 = Magnet Too Far (Field Low)
    uint8_t MAGH;           // 1 = Magnet Too Close (Field High)
    uint8_t COF;            // 1 = CORDIC Overflow (Serious error)
    uint8_t LF;             // 1 = Offset Compensation Finished (Usually 0 is fine during run)
    uint8_t ErrorFlag;      // 1 = The previous frame had an error
    uint8_t ParityError;    // 1 = Parity check failed
    bool OK;
} AS5047P_Diagnostics_t;

enum
{
	SWERVE_FL,
	SWERVE_FR,
	SWERVE_BL,
	SWERVE_BR,
	STEP_UP_FL,
	STEP_UP_FR,
	STEP_UP_BL,
	STEP_UP_BR,
	MECHANISM_1,
	MECHANISM_2,
	AS5047P_COUNT
};

enum
{
	SWERVE,
	MECH1, // mechanism 1
	STEP_UP,
	MECH2, // mechanism 2
	CS_PIN_COUNT
};

// Structure to map a CS pin to its specific SPI buses
typedef struct {
    GPIO_TypeDef* cs_port;    // GPIO Port for Chip Select
    uint16_t cs_pin;          // GPIO Pin for Chip Select
    uint8_t num_spis;         // Number of SPI buses sharing this CS pin
    SPI_HandleTypeDef** spis; // Array of pointers to the SPI handles
} AS5047P_CS_Group_t;

// Struct to hold the specific SPI communication errors
typedef struct {
    uint8_t FRERR;   // Framing Error
    uint8_t INVCOMM; // Invalid Command Error
    uint8_t PARERR;  // Parity Error
} AS5047P_SPI_Errors_t;

typedef struct {
    float continuous_angle;
    float prev_angle;
    float full_resolution;
    bool first_reading;
} Unwrap_Tracker_t;

float AS5047P_ReadAngle(SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin);
uint16_t AS5047P_ReadRegister(uint16_t reg_address, SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin);
void AS5047P_GetDiagnostics(AS5047P_Diagnostics_t *diag, SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin);
void auto_recovery(SPI_HandleTypeDef* SPIs, float raw_angle, uint8_t ADC_val, uint8_t* error_count, GPIO_TypeDef* cs_port, uint16_t cs_pin);
void AS5047P_Group_Read(AS5047P_CS_Group_t* group, float* output_angle, AS5047P_Diagnostics_t* diag, uint8_t* error_count, float* offset);
void AS5047P_Group_GetDiagnostics(AS5047P_CS_Group_t* group, AS5047P_Diagnostics_t* diags);
void AS5047P_SPI_Init(AS5047P_CS_Group_t* cs_groups, uint8_t num_groups);
void AS5047P_Group_ReadAndClearErrors(AS5047P_CS_Group_t* group, AS5047P_SPI_Errors_t* errors);
float Read_And_Unwrap(float current_value, Unwrap_Tracker_t *tracker);

#endif /* INC_AS5047P_H_ */
