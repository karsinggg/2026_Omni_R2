#include "esp32_i2c.h"

#define buffer_size 		9
#define VESC_ESP32_I2C_ADDR (0x32 << 1)

extern I2C_HandleTypeDef hi2c5;
//extern I2C_HandleTypeDef hi2c1;

uint16_t esp32_I2C_addr = 0x16 << 1;	// this is the esp32 i2c address for the PS4
uint8_t i2c_rxdata[buffer_size];
int8_t LX, LY, RX, RY;
uint8_t L2, R2;
bool toggle_button[13] = {0};
int PS4_OK;

void PS4Read(void)
{
    if (HAL_I2C_Master_Receive(&hi2c5, esp32_I2C_addr, i2c_rxdata, buffer_size, 10) == HAL_OK)
    {
		if(i2c_rxdata[0] == 0x88)
		{
			LX = (abs((int8_t)i2c_rxdata[1]) > 10) ? (int8_t)i2c_rxdata[1] : 0;
			LY = (abs((int8_t)i2c_rxdata[2]) > 10) ? (int8_t)i2c_rxdata[2] : 0;
			L2 = i2c_rxdata[3];
			R2 = i2c_rxdata[4];
			RX = (abs((int8_t)i2c_rxdata[5]) > 10) ? (int8_t)i2c_rxdata[5] : 0;
			RY = (abs((int8_t)i2c_rxdata[6]) > 10) ? (int8_t)i2c_rxdata[6] : 0;

			for(int i = 0; i < 8; i++)
			{
				toggle_button[i] = (i2c_rxdata[7] & (0b10000000 >> i));
			}
			for(int i = 8; i < 13; i++)
			{
				toggle_button[i] = (i2c_rxdata[8] & (0b00010000 >> (i-8)));
			}
//	    			printf("LX: %d LY: %d L2: %d R2: %d\r\n", LX, LY, L2, R2);
			PS4_OK= 1;
		}
		else
		{
			LX = 0;
			LY = 0;
			L2 = 0;
			R2 = 0;
			RX = 0;
			RY = 0;
			for(int i = 0; i < sizeof(toggle_button); i++)
			{
				toggle_button[i] = 0;
			}
		}
    }
    else if(HAL_I2C_Master_Receive(&hi2c5, esp32_I2C_addr, i2c_rxdata, buffer_size, 10) != HAL_OK)
    {
		LX = 0;
		LY = 0;
		L2 = 0;
		R2 = 0;
		RX = 0;
		RY = 0;
		for(int i = 0; i < sizeof(toggle_button); i++)
		{
			toggle_button[i] = 0;
		}

		PS4_OK= 0;
    }
}

void DebounceButton(bool* status, bool trigger, bool* lastTrigger, uint32_t* prevTime)
{
	uint32_t currentTime = HAL_GetTick();

	if(trigger && (*lastTrigger != trigger)) // if the button trigger is edge-triggering (LOW to HIGH)
	{
		if((currentTime - *prevTime) > 500)
		{
			*prevTime = currentTime; // update the previous time
			*status = !*status; // toggle the status
		}
	}
	*lastTrigger = trigger;
}

void Button_Update(Button_Tracker_t *btn, bool current_raw_input)
{
    btn->raw_state = current_raw_input;

    // Detect Rising Edge (One-Shot Press)
    btn->pressed_edge = (btn->raw_state && !btn->prev_state);

    // Detect Falling Edge (One-Shot Release)
    btn->released_edge = (!btn->raw_state && btn->prev_state);

    // Flip the toggle state only on a pressed edge
    if (btn->pressed_edge)
    {
        btn->toggled_state = !btn->toggled_state;
    }

    // Save current state for the next RTOS tick
    btn->prev_state = btn->raw_state;
}

//void i2c_set_vesc_rpm(int32_t target_rpm, uint8_t* status)
//{
//	static uint8_t i2c_buf[4]; // MUST be static so the memory exists while the hardware sends it in the background!
//
//	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { // Prevent overwriting the buffer if the I2C hardware is still busy sending the last packet
//		*status = 2;
//		return; // Drop the packet and try again next loop
//	}
//
//	// Break the 32-bit integer into 4 bytes (Big-Endian format)
//	i2c_buf[0] = (target_rpm >> 24) & 0xFF;
//	i2c_buf[1] = (target_rpm >> 16) & 0xFF;
//	i2c_buf[2] = (target_rpm >> 8) & 0xFF;
//	i2c_buf[3] = target_rpm & 0xFF;
//
//	// Transmit over I2C using interrupt for non-blocking
//	HAL_StatusTypeDef res = HAL_I2C_Master_Transmit_IT(&hi2c1, VESC_ESP32_I2C_ADDR, i2c_buf, 4);
//	if (hi2c1.ErrorCode & HAL_I2C_ERROR_AF) {
//		*status = 5; // ESP32 is NOT ACKING! (Check pull-up resistors or address)
//	}
//	else if (res == HAL_OK) *status = 1;
//	else *status = 3;
//}
//
//void I2C_Ping_Test(uint8_t *status_1)
//{
//    uint8_t rx_buffer[1] = {0}; // Buffer to hold the incoming byte
//    HAL_StatusTypeDef status;
//
//    // Ping the ESP32 and ask for 1 byte of data (100ms timeout)
//    status = HAL_I2C_Master_Receive(&hi2c1, VESC_ESP32_I2C_ADDR, rx_buffer, 1, 100);
//
//    if (status == HAL_OK)
//    {
//        if (rx_buffer[0] == 66)
//        {
//        	*status_1 = 1;
//        }
//        else
//        {
//        	*status_1 = 2;
//        }
//    }
//    else
//    {
//        // If the ESP32 is off, wires are unplugged, or no pull-up resistors are present
//        if (hi2c1.ErrorCode & HAL_I2C_ERROR_AF) {
//        	*status_1 = 3;
//        } else {
//        	*status_1 = 4;
//        }
//
//        // Reset the hardware state so it doesn't lock up for the next loop
//        hi2c1.State = HAL_I2C_STATE_READY;
//        hi2c1.Mode = HAL_I2C_MODE_NONE;
//    }
//}
