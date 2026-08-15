

#ifndef INC_FDCAN_NEW_H_
#define INC_FDCAN_NEW_H_

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "string.h"
#include <stdbool.h>

#define NUM_RM_CAN1		4
#define NUM_RM_CAN3		6
#define TOTAL_NUM_RM 	NUM_RM_CAN1 + NUM_RM_CAN3
#define NUM_VESC 		4
#define NUM_BRT 		0

// Robomaster M2006 and M3508
#define RM1 			0x200 // Control Speed of 1st to 4th Robomaster BLDC
#define RM2 			0x1FF // Control Speed 5th to 8th Robomaster BLDC

#define RM1_ID 			0x201 // 1st Robomaster BLDC ID
#define RM2_ID 			0x205

#define M3508_ENCODER_RESOLUTION	8192.00f

#define VESC_1_ID	11
#define VESC_2_ID	12
#define VESC_3_ID	13
#define VESC_4_ID	14

//#define Odrive_ID1 	0x05
//#define Odrive_ID2	0x06
//#define Odrive_ID3	0x07
//#define Odrive_ID4	0x08

#define BRT_ID_FL	0x05
#define BRT_ID_FR	0x06
#define BRT_ID_RL	0x07
#define BRT_ID_RR	0x08

#define Odrive_SetSpeed 0x0D // Odrive Command
#define Odrive_Reboot 0x016
#define MSG_ENCODER_ESTIMATES 0x09

// BRT command
#define ReadEncoderValue 	0x01
#define SetEncoderID		0x02
#define SetCanBaudRate		0x03
#define SetPositionZero		0x06

//static const int OD_ID[NUM_ODRIVE] = {Odrive_ID1, Odrive_ID2, Odrive_ID3, Odrive_ID4};
//static const int BRT_ID[NUM_BRT] = {BRT_ID_FL, BRT_ID_FR, BRT_ID_RL, BRT_ID_RR};

typedef struct {
    // --- 1. Configuration (Set in Init) ---
    // We store IDs here so we can loop through them to check timeouts
    uint32_t VESC_IDs[NUM_VESC];
    uint32_t RM_IDs[TOTAL_NUM_RM];
    uint32_t BRT_IDs[NUM_BRT];

    // --- 2. VESC State (Extended ID) ---
    struct {
        float rpm;           // Parsed Value
        float current;       // Parsed Value
        float duty;          // Parsed Value
        uint32_t last_rx_time; // For Watchdog
        uint8_t  updated;      // Flag: 1 = New Data Available
    } VESC_State[NUM_VESC];

    // --- 3. RoboMaster State (Standard ID) ---
    struct {
        int16_t angle;       // 0-8191
        int16_t rpm;
        int16_t current;
        uint8_t temp;
        uint32_t last_rx_time;
    } RM_State[TOTAL_NUM_RM];

    // --- 4. BRT Encoder State ---
    struct {
        int32_t raw_value;
        uint32_t last_rx_time;
    } BRT_State[NUM_BRT];

    // --- 5. Bus Diagnostics ---
    // 0 = OK, 1 = Error Warning, 2 = Bus Off
    uint8_t Bus_Health[3]; // Index 0=Bus1, 1=Bus2...

} Robot_CAN_Manager_Struct;

// ------------------------------------------------------------------------------------------------------------------------------------

//typedef struct{
//	uint8_t TxData[NUM_ODRIVE][8];
//	float RPS_now[NUM_ODRIVE];
//	float RPS_target[NUM_ODRIVE];
//}Odrive_Variables;


void FDCAN_1_Init(FDCAN_HandleTypeDef *hfdcan1);
void FDCAN_3_Init(FDCAN_HandleTypeDef *hfdcan3);
HAL_StatusTypeDef FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len, uint32_t id_type);
void Process_RM_Msg(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, Robot_CAN_Manager_Struct *robot_manager, uint32_t id_type, uint8_t index_offset);
void Read_And_Unwrap_Encoder(int32_t current_angle, bool *first_reading, int32_t *continuous_angle, int32_t *prev_angle);
//void RM_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id, int16_t current_value, uint8_t index);

//void BRT_ReadAngle(CAN_Variable* CAN, float* shifted_angle, float* accumulated_angle);
//void RM_ReadRPM(CAN_Variable* CAN,int16_t* RPM);
//void OD_ReadRPS(CAN_Variable* CAN,float* RPS);


#endif /* INC_FDCAN_NEW_H_ */
