#ifndef RC_INPUT_H
#define RC_INPUT_H

#include "main.h"
#include <stdbool.h>

/*
 * Initialize all RC channel structures, default baselines, and DSP filters
 */
void RC_Input_Init(void);

/*
 * Process filtering math strictly once per frame arrival
 * Must be executed inside the main processing loop when new data is present
 */
void RC_Input_Update(bool new_frame);

/*
 * Pure getter for raw, unfiltered receiver values (Crucial for Calibration/Failsafe)
 */
uint16_t RC_GetRawChannel(uint8_t channel_index);

/*
 * Pure getter for smoothly filtered, stabilized channel values (Crucial for Actuators/UI)
 */
uint16_t RC_GetFilteredChannel(uint8_t channel_index);

void RC_Input_SetCenter(uint8_t channel_index, uint16_t center_val);

#endif /* RC_INPUT_H */
