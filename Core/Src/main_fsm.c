#include "main_fsm.h"
#include "crsf.h"
#include "signal_processing.h"
#include "main.h"

#define CALIB_SAMPLES       50
#define SAMPLE_INTERVAL_MS  20
#define ALLOWED_JITTER      30

#define SAFE_CENTER_MIN     1430
#define SAFE_CENTER_MAX     1570
#define SAFE_BOTTOM_MIN     970
#define SAFE_BOTTOM_MAX     1100

/* Switch hardware threshold boundaries */
#define SWITCH_LOW_THRESHOLD  1300
#define SWITCH_HIGH_THRESHOLD 1700

static VehicleState_t current_state = STATE_WAITING;
static DriveMode_t current_mode = MODE_MANUAL;
static DriveDirection_t current_direction = DIR_NEUTRAL;

static ChannelFilter_t filters[16];

// Calibration workspace
static uint16_t calib_count = 0;
static uint32_t calib_sum[2] = {0, 0};
static uint16_t calib_min[2] = {2000, 2000};
static uint16_t calib_max[2] = {1000, 1000};
static uint32_t last_sample_time = 0;

void MainFSM_Init(void) {
    current_state = STATE_WAITING;
    current_mode = MODE_MANUAL;
    current_direction = DIR_NEUTRAL;
    calib_count = 0;

    Signal_InitFilter(&filters[CRSF_MAP_STEERING], 1500, 15, 0.25f, CHAN_CENTERED);
    Signal_InitFilter(&filters[CRSF_MAP_THROTTLE], 1000, 15, 0.25f, CHAN_BOTTOM_ANCHORED);
    Signal_InitFilter(&filters[CRSF_MAP_DIR_SWITCH], 1500, 0, 1.0f, CHAN_CENTERED);
    Signal_InitFilter(&filters[CRSF_MAP_RTH_SWITCH], 1500, 0, 1.0f, CHAN_CENTERED);
}

static void MainFSM_ProcessSubStates(void) {
    // 1. Process Transmission Direction Switch (DIR)
    uint16_t dir_pwm = Signal_Process(&filters[CRSF_MAP_DIR_SWITCH], CRSF_GetPWM(CRSF_MAP_DIR_SWITCH));
    if (dir_pwm > SWITCH_HIGH_THRESHOLD) {
        current_direction = DIR_FORWARD;
    } else if (dir_pwm < SWITCH_LOW_THRESHOLD) {
        current_direction = DIR_REVERSE;
    } else {
        current_direction = DIR_NEUTRAL;
    }

    // 2. Process Return-To-Home Autopilot Switch (RTH)
    uint16_t rth_pwm = Signal_Process(&filters[CRSF_MAP_RTH_SWITCH], CRSF_GetPWM(CRSF_MAP_RTH_SWITCH));
    if (rth_pwm < SWITCH_LOW_THRESHOLD) {
        current_mode = MODE_MANUAL;
    } else if (rth_pwm <= SWITCH_HIGH_THRESHOLD) {
        current_mode = MODE_RTH_DIRECT;
    } else {
        current_mode = MODE_RTH_BACKTRACE;
    }
}

void MainFSM_Update(bool new_frame) {
    uint32_t now = HAL_GetTick();

    // Critical system escape route: drop everything on link failure
    if (!crsf.is_connected) {
        current_state = STATE_WAITING;
        current_mode = MODE_MANUAL;
        current_direction = DIR_NEUTRAL;
        calib_count = 0;
        return;
    }

    switch (current_state) {
        case STATE_WAITING:
            if (crsf.is_connected) {
                current_state = VEHICLE_CALIBRATING;
                calib_count = 0;
                calib_sum[0] = 0;   calib_sum[1] = 0;
                calib_min[0] = 2000; calib_min[1] = 2000;
                calib_max[0] = 1000; calib_max[1] = 1000;
                last_sample_time = now;
            }
            break;

        case VEHICLE_CALIBRATING:
            if (now - last_sample_time >= SAMPLE_INTERVAL_MS) {
                last_sample_time = now;

                uint16_t str_raw = CRSF_GetPWM(CRSF_MAP_STEERING);
                uint16_t thr_raw = CRSF_GetPWM(CRSF_MAP_THROTTLE);

                calib_sum[0] += str_raw;
                calib_sum[1] += thr_raw;

                if (str_raw < calib_min[0]) calib_min[0] = str_raw;
                if (str_raw > calib_max[0]) calib_max[0] = str_raw;
                if (thr_raw < calib_min[1]) calib_min[1] = thr_raw;
                if (thr_raw > calib_max[1]) calib_max[1] = thr_raw;

                calib_count++;

                if (calib_count >= CALIB_SAMPLES) {
                    uint16_t str_avg = calib_sum[0] / CALIB_SAMPLES;
                    uint16_t thr_avg = calib_sum[1] / CALIB_SAMPLES;

                    bool is_stable = (calib_max[0] - calib_min[0] < ALLOWED_JITTER) &&
                                     (calib_max[1] - calib_min[1] < ALLOWED_JITTER);

                    bool is_str_centered = (str_avg > SAFE_CENTER_MIN && str_avg < SAFE_CENTER_MAX);
                    bool is_thr_bottomed = (thr_avg > SAFE_BOTTOM_MIN && thr_avg < SAFE_BOTTOM_MAX);

                    if (is_stable && is_str_centered && is_thr_bottomed) {
                        filters[CRSF_MAP_STEERING].center = str_avg;
                        filters[CRSF_MAP_THROTTLE].center = thr_avg;
                        filters[CRSF_MAP_STEERING].filtered_val = (float)str_avg;
                        filters[CRSF_MAP_THROTTLE].filtered_val = (float)thr_avg;
                        current_state = STATE_READY;
                    } else {
                        current_state = STATE_ERROR;
                    }
                }
            }
            break;

        case STATE_ERROR: {
            uint16_t str_raw = CRSF_GetPWM(CRSF_MAP_STEERING);
            uint16_t thr_raw = CRSF_GetPWM(CRSF_MAP_THROTTLE);

            if (str_raw > SAFE_CENTER_MIN && str_raw < SAFE_CENTER_MAX &&
                thr_raw > SAFE_BOTTOM_MIN && thr_raw < SAFE_BOTTOM_MAX) {
                current_state = STATE_WAITING;
            }
            break;
        }

        case STATE_READY:
        	/* OPTIMIZATION, Only calculate heavy filters if a fresh frame arrived */
        	if (new_frame) {
				/* Run sub-state estimation inside operating loop context */
				MainFSM_ProcessSubStates();

				if (current_mode == MODE_MANUAL) {
					// Manual mode execution loop placeholder
				} else {
					// Autonomous navigation algorithms (RTH_DIRECT / RTH_BACKTRACE) will be injected here
				}
        	}
            break;
    }
}

VehicleState_t MainFSM_GetState(void) { return current_state; }
DriveMode_t MainFSM_GetMode(void) { return current_mode; }
DriveDirection_t MainFSM_GetDirection(void) { return current_direction; }

uint16_t MainFSM_GetOutput(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;

    // Hard hardware interlock lockout clamp if system lifecycle state is not ready
    if (current_state != STATE_READY) {
        if (channel_index == CRSF_MAP_STEERING) return 1500;
        if (channel_index == CRSF_MAP_THROTTLE) return 1000;
    }

    // Handle structural safety logic filters during active manual control state
    if (current_mode == MODE_MANUAL) {
        if (channel_index == CRSF_MAP_STEERING) {
            uint16_t raw_pwm = CRSF_GetPWM(CRSF_MAP_STEERING);
            return Signal_Process(&filters[CRSF_MAP_STEERING], raw_pwm);
        }

        if (channel_index == CRSF_MAP_THROTTLE) {
            // Neutral transmission guard isolation
            if (current_direction == DIR_NEUTRAL) {
                return 1000; // Hard engine idle state stop
            }

            uint16_t raw_pwm = CRSF_GetPWM(CRSF_MAP_THROTTLE);
            return Signal_Process(&filters[CRSF_MAP_THROTTLE], raw_pwm);
        }
    } else {
        // Autopilot route commands override layer output (to be structured on next steps)
        if (channel_index == CRSF_MAP_THROTTLE) return 1000;
        if (channel_index == CRSF_MAP_STEERING) return 1500;
    }

    return CRSF_GetPWM(channel_index);
}
