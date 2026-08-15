#include "fdcan.h"
#include <math.h>
//#include "vesc.h"

void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
	// // 1. Config Global Filter (Reject non-matching frames to save CPU) (Whitelist approach)
	//    Both Standard and Extended frames are rejected if they don't match a filter.
	HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	// 2. Configure Interrupt Lines (Optional but good practice)
	// This routes critical errors (like Bus Off) to Line 0, which might have higher priority in NVIC
	HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE0);

	// 3. Activate Notifications (Robust Set)
	// This includes all the safety checks from your snippet
	HAL_FDCAN_ActivateNotification(hfdcan,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE | // Essential: Received a packet
			FDCAN_IT_RX_FIFO0_FULL |        // Critical: CPU is too slow, buffer full
			FDCAN_IT_RX_FIFO0_MESSAGE_LOST| // Critical: Dropped packets
			FDCAN_IT_BUS_OFF |              // Critical: Cable disconnected or no 60 ohm resistor
			FDCAN_IT_ERROR_WARNING |        // Warning: Too much noise on bus
			FDCAN_IT_TX_COMPLETE,           // Optional: Confirm packet sent (good for debug)
			0);
}

// --- 2. VESC Filter (Extended ID) ---
void Add_Vesc_Filter(FDCAN_HandleTypeDef *hfdcan, uint32_t vesc_id, uint32_t filter_index)
{
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = filter_index;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	// Match the VESC ID in the last 8 bits
	sFilterConfig.FilterID1 = vesc_id;
	sFilterConfig.FilterID2 = 0x000000FF; // MASK: Only check the last byte

	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		printf("VESC Filter Config Failed (Index %lu)\r\n", filter_index);
	}
}

// --- 3. RoboMaster/C620 Filter (Standard ID Range) ---
// Example: start_id=0x201, count=4 matches 0x201, 0x202, 0x203, 0x204
void Add_RoboMaster_Filter(FDCAN_HandleTypeDef *hfdcan, uint32_t start_id, uint32_t count, uint32_t filter_index)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = filter_index;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE; // Range is efficient for RM
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    sFilterConfig.FilterID1 = start_id;
    sFilterConfig.FilterID2 = start_id + count - 1; // End of range

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        printf("RM Filter Config Failed (Index %lu)\r\n", filter_index);
    }
}

// --- 4. Generic Standard Filter (Single ID) ---
// Useful for BRT Encoders if they are not in a sequential range
void Add_Standard_Filter(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, uint32_t filter_index)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = filter_index;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    sFilterConfig.FilterID1 = can_id;
    sFilterConfig.FilterID2 = 0x7FF; // Mask: 0x7FF means "Match Exact ID"

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        printf("Std Filter Config Failed (Index %lu)\r\n", filter_index);
    }
}

void FDCAN_1_Init(FDCAN_HandleTypeDef *hfdcan1)
{
	    // Step A: Init (Global Reject, Start, Interrupts)
	    FDCAN_Init(hfdcan1);

	    // Step B: Add Filters
	    Add_Vesc_Filter(hfdcan1, VESC_1_ID, 0); // VESC ID 1 at Index 0
	    Add_Vesc_Filter(hfdcan1, VESC_2_ID, 1); // VESC ID 2 at Index 1
	    Add_Vesc_Filter(hfdcan1, VESC_3_ID, 2); // VESC ID 2 at Index 1
	    Add_Vesc_Filter(hfdcan1, VESC_4_ID, 3); // VESC ID 2 at Index 1

	    // RoboMaster Feedback (IDs 0x201 to 0x204) at Index 4 (Index 0, 1, 2 and 3 for VESC)
	    // Note: 0x201 is the start, 4 is the count.
	    Add_RoboMaster_Filter(hfdcan1, RM1_ID, 4, 4);

	    // START THE MODULE AT THE VERY END (after everything else is configured)
		if (HAL_FDCAN_Start(hfdcan1) != HAL_OK) {
			printf("FDCAN Start Failed\r\n");
		}

}

void FDCAN_3_Init(FDCAN_HandleTypeDef *hfdcan3)
{
	FDCAN_Init(hfdcan3);

	// RoboMaster Feedback (IDs 0x201 to 0x204) at Index 0 (a different bus starts from index 0)
	// Note: 0x201 is the start, 4 is the count.
	Add_RoboMaster_Filter(hfdcan3, RM1_ID, 4, 0);
	Add_RoboMaster_Filter(hfdcan3, RM2_ID, 4, 1);

	// START THE MODULE AT THE VERY END (after everything else is configured)
	if (HAL_FDCAN_Start(hfdcan3) != HAL_OK) {
		printf("FDCAN Start Failed\r\n");
	}
}

// Helper function to convert raw byte length to FDCAN DLC code
uint32_t Get_FDCAN_DLC(uint8_t length_in_bytes)
{
    switch(length_in_bytes) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_8; // Default to 8 if invalid
    }
}

// Returns: 0 -> OK, 1 -> Error, 2 -> FIFO Full
HAL_StatusTypeDef FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len, uint32_t id_type)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint32_t tickstart = HAL_GetTick();

    // 1. Configure the Header for this specific message
    TxHeader.Identifier = id;
    TxHeader.IdType = id_type;                 // Dynamic: Standard or Extended
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = Get_FDCAN_DLC(len);  // Fixes the Length Bug
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 2. Try to add to FIFO with a small Timeout (e.g., 2ms)
	// This handles the case where FIFO is momentarily full due to a burst of traffic.
	while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
	{
		// If we waited longer than 2ms, give up to prevent hanging the robot
		if ((HAL_GetTick() - tickstart) > 2)
		{
			return HAL_BUSY;
		}
	}

	// 3. Add to FIFO
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}


// Optimized Processing Function
// Call this IMMEDIATELY after HAL_FDCAN_GetRxMessage (in Callback or Polling loop)
void Process_RM_Msg(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, Robot_CAN_Manager_Struct *robot_manager, uint32_t id_type, uint8_t index_offset)
{
	if (RxHeader->IdType != FDCAN_STANDARD_ID) { // SECURITY CHECK: Ignore Extended IDs (VESC traffic)
		return;
	}

    // 1. Calculate the Index directly
    // If ID is 0x201, index = 0. If ID is 0x202, index = 1.
    int16_t index = RxHeader->Identifier - id_type;

    // Add the offset for CAN 2 bus (as we are using the same ID for M3508 and M2006 in different bus)
	int16_t final_index = index + index_offset;

    // 2. Security Check: Ensure the ID is actually a RoboMaster ID within range
    if (final_index >= 0 && final_index < TOTAL_NUM_RM)
    {
    	// 3. Update the specific motor in the struct
		// RoboMaster Data Format:
		// Byte 0-1: Angle, Byte 2-3: RPM, Byte 4-5: Current, Byte 6: Temp
		robot_manager->RM_State[final_index].angle   = (int16_t)(RxData[0] << 8 | RxData[1]);
		robot_manager->RM_State[final_index].rpm     = (int16_t)(RxData[2] << 8 | RxData[3]);
		robot_manager->RM_State[final_index].current = (int16_t)(RxData[4] << 8 | RxData[5]);
		robot_manager->RM_State[final_index].temp    = RxData[6];

		// 4. Update Timestamp (Crucial for safety checks!)
		robot_manager->RM_State[final_index].last_rx_time = HAL_GetTick();
    }
}

void Read_And_Unwrap_Encoder(int32_t current_angle, bool *first_reading, int32_t *continuous_angle, int32_t *prev_angle)
{
	if (*first_reading) {
		*prev_angle = current_angle;
		*continuous_angle = current_angle;
		*first_reading = false;
	} else {
		// Calculate the shortest path change
		int32_t delta_angle = current_angle - *prev_angle;

		// Unwrap the boundary crossing
		if (delta_angle > 4096) {
			delta_angle -= 8192;
		} else if (delta_angle < -4096) {
			delta_angle += 8192;
		}

		*continuous_angle += delta_angle;
		*prev_angle = current_angle;
	}
}

//void RM_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id, int16_t current_value, uint8_t index)
//{
//	uint8_t txData[8];
//
//	for(uint8_t i = 0; i < 8; i++)
//	{
//		if(i == index){
//			txData[i] = (uint8_t)(current_value >> 8);
//		}
//		else if(i == index+1)
//		{
//			txData[i] = (uint8_t)(current_value & 0xFF);
//		}
//		else
//		{
//			txData[i] = 0;
//		}
//	}
//
////	FDCAN_Transmit(hfdcan, RM1, txData, 8, FDCAN_STANDARD_ID);
//	FDCAN_Transmit(hfdcan, id, txData, 8, FDCAN_STANDARD_ID);
//}
