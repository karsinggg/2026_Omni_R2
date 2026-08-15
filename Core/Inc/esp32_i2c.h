/*
 * esp32_i2c.h
 *
 *  Created on: Aug 21, 2025
 *      Author: Acer
 */

#ifndef INC_ESP32_I2C_H_
#define INC_ESP32_I2C_H_

#include "stm32h7xx_hal.h"
#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

typedef enum
{
	dpad_up,
	dpad_left,
	dpad_down,
	dpad_right,
	triangle,
	square,
	cross,
	circle,
	L1,
	R1,
	share,
	options,
	start_button,
} ps4_toggles;

typedef struct debounce_button_t
{
	bool state;
	bool prev_state;
	uint32_t prevTime;
} debounce_button_t;

typedef struct {
    bool raw_state;       // The current physical state of the button
    bool prev_state;      // The physical state in the previous RTOS tick

    // --- Evaluated Events ---
    bool pressed_edge;    // TRUE for exactly one tick when the button is pressed
    bool released_edge;   // TRUE for exactly one tick when the button is released
    bool toggled_state;   // Flips between TRUE and FALSE on every press
} Button_Tracker_t;

extern int8_t LX, LY, RX, RY;
extern uint8_t L2, R2;
extern bool toggle_button[13];
extern int PS4_OK;

void PS4Read(void);
void DebounceButton(bool* status, bool trigger, bool* lastTrigger, uint32_t* prevTime);
void Button_Update(Button_Tracker_t *btn, bool current_raw_input);
//void i2c_set_vesc_rpm(int32_t target_rpm, uint8_t *status);
//void I2C_Ping_Test(uint8_t *status_1);

#endif /* INC_ESP32_I2C_H_ */
