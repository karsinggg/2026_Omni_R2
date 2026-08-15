#include "as5047p.h"
#include <math.h>

// Function to calculate Even Parity
static uint16_t AS5047P_AddParity(uint16_t value) {
    uint16_t parity = 0;
    uint16_t temp = value;

    // Count 1s
    while (temp) {
        parity ^= (temp & 1);
        temp >>= 1;
    }

    // If parity is odd, set MSB (bit 15) to make it even
    if (parity) {
        value |= 0x8000;
    }

    return value;
}

// --- Helper: Check Parity of RECEIVED data ---
// Returns: 1 if Parity is OK, 0 if Parity Error
static uint8_t AS5047P_CheckRxParity(uint16_t received_frame) {
    // 1. Calculate parity of the lower 15 bits
    uint16_t count = 0;
    for (int i = 0; i < 15; i++) {
        if (received_frame & (1 << i)) count++;
    }

    // 2. Check if calculated Even Parity matches the received Parity Bit (Bit 15)
    uint16_t calculated_parity = (count % 2); // 1 if odd, 0 if even
    uint16_t received_parity_bit = (received_frame >> 15) & 0x01;

    // The Parity Bit is set to make the TOTAL number of 1s even.
    // So if count is odd (1), Parity Bit should be 1. (Total 1+1=2 Even).
    // If count is even (0), Parity Bit should be 0.

    return (calculated_parity == received_parity_bit);
}

// Function to read a specific register
// Note: You must call this TWICE or continuously to get the data requested.
// The first call sends the command, the second call receives the data.
uint16_t AS5047P_ReadRegister(uint16_t reg_address, SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin)
{
    uint16_t command = reg_address | AS5047P_READ;
    command = AS5047P_AddParity(command);

    uint16_t received_data = 0;

	// 1. Pull CS Low
	HAL_GPIO_WritePin(GPIOx, CS_Pin, GPIO_PIN_RESET);

	// 2. Transmit Command and Receive Data (from previous command)
	if (HAL_SPI_TransmitReceive(AS5047P_SPI_HANDLE, (uint8_t*)&command, (uint8_t*)&received_data, 1, 10) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOx, CS_Pin, GPIO_PIN_SET);
//		for(int i=0; i<50; i++) { __NOP(); }
		return 0xFFFF; // Return Error if HAL fails, don't return 0!
	}

	// 3. Pull CS High
	HAL_GPIO_WritePin(GPIOx, CS_Pin, GPIO_PIN_SET);

    // Small delay to ensure CS High time (350ns min requirement)
	// The H7 is 550MHz, so a few NOPs or a tiny delay helps stability.
	for(int i=0; i<200; i++) { __NOP(); }

	// --- SAFETY CHECKS (Now it is safe to return) ---
	// 1. "All Zeros" Check (Wire Failure) <--- ADD THIS HERE
	if (received_data == 0x0000) {
	    return 0xFFFF; // Treat exact 0 as an error
	}

	// 2. Parity Check
	if (!AS5047P_CheckRxParity(received_data)) {
		// Log error or just return error code
		return 0xFFFF; // Error Code: Parity Mismatch
	}

	// 3. Error Flag (Bit 14)
	if (received_data & 0x4000) {
		return 0xFFFF; // Error Code: Sensor reported internal error
	}

    // Remove Parity Bit (Bit 15) and Error Flag (Bit 14) to get raw value
    return (received_data & 0x3FFF);
}

// Higher level function to get Angle in Degrees
float AS5047P_ReadAngle(SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin)
{
    // We send the read command for ANGLECOM (0x3FFF)
	AS5047P_ReadRegister(AS5047P_ANGLECOM, AS5047P_SPI_HANDLE, GPIOx, CS_Pin);

    // Note: the data received is the ANGLE from the PREVIOUS call.
	uint16_t raw_data = AS5047P_ReadRegister(AS5047P_NOP, AS5047P_SPI_HANDLE, GPIOx, CS_Pin);

    if (raw_data == 0xFFFF) {
		return -1.0f; // Indicate Failure
	}

    // 14-bit resolution: 2^14 = 16384 steps
    float angle = (float)raw_data / 16384.0f * 360.0f;

    return angle;
}

void AS5047P_GetDiagnostics(AS5047P_Diagnostics_t *diag, SPI_HandleTypeDef* AS5047P_SPI_HANDLE, GPIO_TypeDef *GPIOx, uint16_t CS_Pin)
{
    // 1. Send Request for DIAAGC
    // The data we receive *now* is garbage (or previous angle), so we ignore it.
    AS5047P_ReadRegister(AS5047P_DIAAGC, AS5047P_SPI_HANDLE, GPIOx, CS_Pin);

    // 2. Send NOP to clock out the DIAAGC result
    // The sensor sends the data requested in step 1.
    uint16_t raw_data = AS5047P_ReadRegister(AS5047P_NOP, AS5047P_SPI_HANDLE, GPIOx, CS_Pin);

    // 3. Parse Error Flag (Bit 14) and Parity
    diag->ErrorFlag = (raw_data & 0x4000) >> 14;

    // 4. Parse Data (Bits 0-13)
    // DIAAGC Register Mapping:
    // Bit 11: MAGL, Bit 10: MAGH, Bit 9: COF, Bit 8: LF, Bits 7-0: AGC
    diag->MAGL = (raw_data >> 11) & 0x01;
    diag->MAGH = (raw_data >> 10) & 0x01;
    diag->COF  = (raw_data >> 9)  & 0x01;
    diag->LF   = (raw_data >> 8)  & 0x01;
    diag->AGC_Value = raw_data & 0xFF;
}

void auto_recovery(SPI_HandleTypeDef* SPIs, float raw_angle, uint8_t ADC_val, uint8_t* error_count, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	if (raw_angle == -1.0f || ADC_val == 255)
	{
	    (*error_count)++;

	    // If we get 10 errors in a row, the sensor is likely "stuck"
	    if (*error_count > 10) {
	        // 1. Force Reset the SPI Peripheral
	        HAL_SPI_DeInit(SPIs);
	        HAL_SPI_Init(SPIs);

	        // 2. Cycle the Chip Select to reset the Sensor's SPI logic
	        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	        for(int i=0; i<500; i++) { __NOP(); }
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
			for(int i=0; i<500; i++) { __NOP(); }
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

	        *error_count = 0; // Reset counter
	    }
	}
	else {
	    *error_count = 0; // Connection is good
	}
}

float apply_offset(float raw_angle, float offset)
{
	float corrected = raw_angle - offset;

	// Remove any extra full rotations (handles huge offsets if they exist)
//	corrected = fmod(corrected, 360.0); // fmod is a standard C library function for double-precision floats, it carries a lot of overhead

	// Fast wrapping to the (-180, 180] range
	while (corrected > 180.0f) {
		corrected -= 360.0f;
	}
	while (corrected <= -180.0f) {
		corrected += 360.0f;
	}

	return corrected;
}

// Reads the encoder angle register using the Group Struct and Concurrent SPI (for multiple SPIs sharing a single CS pin)
void AS5047P_Group_Read(AS5047P_CS_Group_t* group, float* output_angle, AS5047P_Diagnostics_t* diag, uint8_t* error_count, float* offset)
{
    uint16_t angle_cmd = AS5047P_ANGLECOM | AS5047P_READ;
    angle_cmd = AS5047P_AddParity(angle_cmd);

    uint16_t nop_cmd = AS5047P_NOP | AS5047P_READ;
    nop_cmd = AS5047P_AddParity(nop_cmd);

    // We use TransmitReceive for BOTH steps to keep clock syncing clean.
    // The TX arrays hold the same command duplicated for each SPI bus.
    uint16_t tx_cmd_array[4] = {angle_cmd, angle_cmd, angle_cmd, angle_cmd};
    uint16_t tx_nop_array[4] = {nop_cmd, nop_cmd, nop_cmd, nop_cmd};

    uint16_t dummy_rx[4] = {0}; // Holds garbage data from the first command
    uint16_t rx_data[4] = {0};  // Holds the actual angle data from the second command

    // ------------------------------------------------------------------------
    // STEP 1: Send ANGLECOM request concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    // Start all SPIs concurrently using non-blocking Interrupt (IT) mode
    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_cmd_array[i], (uint8_t*)&dummy_rx[i], 1);
    }

    // Wait for all SPIs to finish their 16-bit hardware transmission
    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {
            // Block until this specific SPI peripheral is finished
        }
    }

    // Pull CS High to execute the command simultaneously on all sensors
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);

    // Give sensor time to load the angle (t_CSn minimum 350ns requirement)
    // 200 iterations at 550MHz is safely ~360+ ns.
    for(volatile int d=0; d<200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 2: Send NOP to clock out the actual angle data concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_nop_array[i], (uint8_t*)&rx_data[i], 1);
    }

    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {
            // Block until this specific SPI peripheral is finished receiving
        }
    }

    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);

    // t_CSn minimum 350ns requirement before the next potential frame
    for(volatile int d=0; d<200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 3: Process the data individually
    // ------------------------------------------------------------------------
    for(int i = 0; i < group->num_spis; i++) {
        float raw_angle = -1.0f;

        // Run safety checks: Non-zero, Parity OK, Error Flag Clear
        if ((rx_data[i] != 0x0000) &&
            AS5047P_CheckRxParity(rx_data[i]) &&
            !(rx_data[i] & 0x4000))
        {
            // Extract the raw 14-bit value
            uint16_t clean_data = rx_data[i] & 0x3FFF;
            raw_angle = (float)clean_data / 16384.0f * 360.0f;
        }

        // Apply recovery or offset logic
        if(raw_angle < 0.0f) {
            auto_recovery(group->spis[i], raw_angle, diag[i].AGC_Value, &error_count[i], group->cs_port, group->cs_pin);
        }
        else {
            error_count[i] = 0;
            output_angle[i] = apply_offset(raw_angle, offset[i]);
        }
    }
}

// Reads the DIAAGC diagnostic register using the Group Struct and Concurrent SPI
void AS5047P_Group_GetDiagnostics(AS5047P_CS_Group_t* group, AS5047P_Diagnostics_t* diags)
{
    // Prepare the DIAAGC read command [cite: 353]
    uint16_t read_diag_cmd = AS5047P_DIAAGC | AS5047P_READ;
    read_diag_cmd = AS5047P_AddParity(read_diag_cmd);

    // Prepare the NOP read command to clock out the data [cite: 338]
    uint16_t nop_cmd = AS5047P_NOP | AS5047P_READ;
    nop_cmd = AS5047P_AddParity(nop_cmd);

    // Arrays to hold the identical commands for all SPI buses sharing this CS pin
    uint16_t tx_cmd_array[4] = {read_diag_cmd, read_diag_cmd, read_diag_cmd, read_diag_cmd};
    uint16_t tx_nop_array[4] = {nop_cmd, nop_cmd, nop_cmd, nop_cmd};

    uint16_t dummy_rx[4] = {0}; // Holds garbage data from the first command
    uint16_t rx_data[4] = {0};  // Holds the actual diagnostic data from the second command

    // ------------------------------------------------------------------------
    // STEP 1: Send DIAAGC read request concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    // Dispatch all SPI hardware concurrently
    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_cmd_array[i], (uint8_t*)&dummy_rx[i], 1);
    }

    // Wait for all SPI hardware to finish clocking out the 16 bits
    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {
            // Block until this specific SPI peripheral is finished
        }
    }

    // Pull CS High to execute the command simultaneously on all sensors
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);

    // t_CSn delay: Minimum 350ns requirement between frames
    for(volatile int d = 0; d < 200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 2: Send NOP to clock out the actual diagnostic data concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_nop_array[i], (uint8_t*)&rx_data[i], 1);
    }

    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {
            // Block until this specific SPI peripheral is finished receiving
        }
    }

    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);

    // t_CSn delay before any other functions can talk to the sensor
    for(volatile int d = 0; d < 200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 3: Parse the responses into your struct array
    // ------------------------------------------------------------------------
    for(int i = 0; i < group->num_spis; i++) {

        // Parse SPI Error Flag (Bit 14) [cite: 278]
        diags[i].ErrorFlag = (rx_data[i] >> 14) & 0x01;

        // Extract bits according to the AS5047P DIAAGC register map [cite: 354]
        diags[i].MAGL      = (rx_data[i] >> 11) & 0x01; // Magnetic field too low
        diags[i].MAGH      = (rx_data[i] >> 10) & 0x01; // Magnetic field too high
        diags[i].COF       = (rx_data[i] >> 9)  & 0x01; // CORDIC overflow
        diags[i].LF        = (rx_data[i] >> 8)  & 0x01; // Offset compensation finished
        diags[i].AGC_Value = rx_data[i] & 0xFF;         // Automatic Gain Control value
    }
}

// Initialization function
void AS5047P_SPI_Init(AS5047P_CS_Group_t* cs_groups, uint8_t num_groups)
{
    uint16_t nop_cmd = AS5047P_NOP | AS5047P_READ;
    nop_cmd = AS5047P_AddParity(nop_cmd);

    uint16_t angle_cmd = AS5047P_ANGLECOM | AS5047P_READ;
    angle_cmd = AS5047P_AddParity(angle_cmd);

    uint16_t tx_nop_array[4] = {nop_cmd, nop_cmd, nop_cmd, nop_cmd};
    uint16_t tx_angle_array[4] = {angle_cmd, angle_cmd, angle_cmd, angle_cmd};
    uint16_t dummy_rx[4] = {0};

    for(int i = 0; i < num_groups; i++)
    {
        // Flush the SPI line to clear startup garbage
        HAL_GPIO_WritePin(cs_groups[i].cs_port, cs_groups[i].cs_pin, GPIO_PIN_SET);
        HAL_Delay(2);

        // We run the dummy reads 4 times to clear the AS5047P internal pipelines
        for(int attempt = 0; attempt < 4; attempt++)
        {
            // 1. Send NOP concurrently
            HAL_GPIO_WritePin(cs_groups[i].cs_port, cs_groups[i].cs_pin, GPIO_PIN_RESET);

            for(int j = 0; j < cs_groups[i].num_spis; j++) {
                HAL_SPI_TransmitReceive_IT(cs_groups[i].spis[j], (uint8_t*)&tx_nop_array[j], (uint8_t*)&dummy_rx[j], 1);
            }
            for(int j = 0; j < cs_groups[i].num_spis; j++) {
                while (HAL_SPI_GetState(cs_groups[i].spis[j]) != HAL_SPI_STATE_READY) {}
            }

            HAL_GPIO_WritePin(cs_groups[i].cs_port, cs_groups[i].cs_pin, GPIO_PIN_SET);

            // Delay to satisfy minimum 350ns t_CSn requirement
            for(volatile int d = 0; d < 200; d++) { __NOP(); }

            // 2. Send ANGLECOM concurrently
            HAL_GPIO_WritePin(cs_groups[i].cs_port, cs_groups[i].cs_pin, GPIO_PIN_RESET);

            for(int j = 0; j < cs_groups[i].num_spis; j++) {
                HAL_SPI_TransmitReceive_IT(cs_groups[i].spis[j], (uint8_t*)&tx_angle_array[j], (uint8_t*)&dummy_rx[j], 1);
            }
            for(int j = 0; j < cs_groups[i].num_spis; j++) {
                while (HAL_SPI_GetState(cs_groups[i].spis[j]) != HAL_SPI_STATE_READY) {}
            }

            HAL_GPIO_WritePin(cs_groups[i].cs_port, cs_groups[i].cs_pin, GPIO_PIN_SET);

            // Delay to satisfy minimum 350ns t_CSn requirement
            for(volatile int d = 0; d < 200; d++) { __NOP(); }
        }
    }
}

// Reads and automatically clears the ERRFL (Error Flag) register
void AS5047P_Group_ReadAndClearErrors(AS5047P_CS_Group_t* group, AS5047P_SPI_Errors_t* errors)
{
    // Address of ERRFL is 0x0001
    uint16_t read_err_cmd = 0x0001 | AS5047P_READ;
    read_err_cmd = AS5047P_AddParity(read_err_cmd);

    uint16_t nop_cmd = AS5047P_NOP | AS5047P_READ;
    nop_cmd = AS5047P_AddParity(nop_cmd);

    uint16_t tx_cmd_array[4] = {read_err_cmd, read_err_cmd, read_err_cmd, read_err_cmd};
    uint16_t tx_nop_array[4] = {nop_cmd, nop_cmd, nop_cmd, nop_cmd};

    uint16_t dummy_rx[4] = {0};
    uint16_t rx_data[4] = {0};

    // ------------------------------------------------------------------------
    // STEP 1: Send ERRFL read request concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_cmd_array[i], (uint8_t*)&dummy_rx[i], 1);
    }
    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {}
    }

    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);

    // t_CSn delay: Minimum 350ns requirement
    for(volatile int d = 0; d < 200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 2: Send NOP to clock out the ERRFL data concurrently
    // ------------------------------------------------------------------------
    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_RESET);

    for(int i = 0; i < group->num_spis; i++) {
        HAL_SPI_TransmitReceive_IT(group->spis[i], (uint8_t*)&tx_nop_array[i], (uint8_t*)&rx_data[i], 1);
    }
    for(int i = 0; i < group->num_spis; i++) {
        while (HAL_SPI_GetState(group->spis[i]) != HAL_SPI_STATE_READY) {}
    }

    HAL_GPIO_WritePin(group->cs_port, group->cs_pin, GPIO_PIN_SET);
    for(volatile int d = 0; d < 200; d++) { __NOP(); }

    // ------------------------------------------------------------------------
    // STEP 3: Parse the error flags
    // ------------------------------------------------------------------------
    for(int i = 0; i < group->num_spis; i++) {
        // Parse Bits 0, 1, and 2 according to the ERRFL register map
        errors[i].FRERR   = rx_data[i] & 0x01;         // Bit 0: Framing error
        errors[i].INVCOMM = (rx_data[i] >> 1) & 0x01;  // Bit 1: Invalid command error
        errors[i].PARERR  = (rx_data[i] >> 2) & 0x01;  // Bit 2: Parity error
    }
}

float Read_And_Unwrap(float current_value, Unwrap_Tracker_t *tracker)
{
	if (tracker->first_reading) {
		tracker->prev_angle = current_value;
		tracker->continuous_angle = current_value;
		tracker->first_reading = false;
	} else {
		// Calculate the shortest path change
		float delta = current_value - tracker->prev_angle;
        float half_wrap = (tracker->full_resolution / 2.0f);

		// Unwrap the boundary crossing
		if (delta > half_wrap) {
			delta -= tracker->full_resolution;
		} else if (delta < -half_wrap) {
			delta += tracker->full_resolution;
		}

		tracker->continuous_angle += delta;
		tracker->prev_angle = current_value;
	}

	return tracker->continuous_angle;
}
