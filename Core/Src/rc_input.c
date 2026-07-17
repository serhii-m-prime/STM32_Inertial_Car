#include "rc_input.h"
#include "crsf.h"
#include "signal_processing.h"
#include "vehicle_config.h"

/* Internal service workspace */
static ChannelFilter_t filters[16];
static uint16_t filtered_cache[16];

void RC_Input_Init(void) {
    /* Initialize all 16 channels with transparent fallback filters (no smoothing) */
    for (uint8_t i = 0; i < 16; i++) {
        Signal_InitFilter(&filters[i], 1500, 0, 1.0f, CHAN_CENTERED);
        filtered_cache[i] = 1500;
    }

    /* Configure customized DSP profiles for steering and throttle channels */
    Signal_InitFilter(&filters[CRSF_MAP_STEERING], 1500,
                      vehicle_config.steering_deadband_us,
                      vehicle_config.steering_filter_pct / 100.0f, CHAN_CENTERED);
    Signal_InitFilter(&filters[CRSF_MAP_THROTTLE], 1000,
                      vehicle_config.throttle_deadband_us,
                      vehicle_config.throttle_filter_pct / 100.0f, CHAN_BOTTOM_ANCHORED);

    /* Configure digital switch channels with zero deadband and direct mapping */
    Signal_InitFilter(&filters[CRSF_MAP_DIR_SWITCH], 150, 0, 1.0f, CHAN_CENTERED);
    Signal_InitFilter(&filters[CRSF_MAP_RTH_SWITCH], 1000, 0, 1.0f, CHAN_BOTTOM_ANCHORED);
}

void RC_Input_Update(bool new_frame) {
    /* Execute resource-heavy mathematical filters strictly upon fresh frame arrival */
    if (new_frame) {
        for (uint8_t i = 0; i < 16; i++) {
            uint16_t raw_val = CRSF_GetPWM(i);
            filtered_cache[i] = Signal_Process(&filters[i], raw_val);
        }
    }
}

uint16_t RC_GetRawChannel(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;
    return CRSF_GetPWM(channel_index);
}

uint16_t RC_GetFilteredChannel(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;
    return filtered_cache[channel_index];
}

void RC_Input_SetCenter(uint8_t channel_index, uint16_t center_val) {
    if (channel_index >= 16) return;

    /* Inject new calibration parameters safely into the private structure */
    filters[channel_index].center = center_val;

    /* Flush state memory to the new baseline to prevent filter settling artifacts */
    filters[channel_index].filtered_val = (float)center_val;
    filtered_cache[channel_index] = center_val;
}

uint16_t RC_Input_GetCenter(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;
    return filters[channel_index].center;
}
