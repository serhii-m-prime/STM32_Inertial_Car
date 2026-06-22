#include "main_fsm.h"
#include "crsf.h"
#include "signal_processing.h"
#include "main.h"
#include "rc_input.h"

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "servo.h"
/* USER CODE END Includes */

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

/* Thread-safe software buffers for calculated execution commands */
static uint16_t output_throttle = 1000;
static uint16_t output_steering = 1500;

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

    output_throttle = 1000;
    output_steering = 1500;
}

static void MainFSM_ProcessSubStates(void) {
    // Process Transmission Direction Switch (DIR)
    uint16_t dir_pwm = RC_GetFilteredChannel(CRSF_MAP_DIR_SWITCH);
    if (dir_pwm > SWITCH_HIGH_THRESHOLD) {
        current_direction = DIR_FORWARD;
    } else if (dir_pwm < SWITCH_LOW_THRESHOLD) {
        current_direction = DIR_REVERSE;
    } else {
        current_direction = DIR_NEUTRAL;
    }

    // Process Return-To-Home Autopilot Switch (RTH)
    uint16_t rth_pwm = RC_GetFilteredChannel(CRSF_MAP_RTH_SWITCH);
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

        output_throttle = 1000;
        output_steering = 1500;

        /* Hardware interlock safety override: force stop actuators instantly */
        Motor_SetPower(0, 0);
        Servo_SetPulse(1500);
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

                uint16_t str_raw = RC_GetRawChannel(CRSF_MAP_STEERING);
                uint16_t thr_raw = RC_GetRawChannel(CRSF_MAP_THROTTLE);

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
                    	RC_Input_SetCenter(CRSF_MAP_STEERING, str_avg);
                    	RC_Input_SetCenter(CRSF_MAP_THROTTLE, thr_avg);
                        current_state = STATE_READY;
                    } else {
                        current_state = STATE_ERROR;
                    }
                }
            }
            break;

        case STATE_ERROR: {
            /* Read absolute raw positions to bypass LPF filter settling delay during recovery check */
            uint16_t str_raw = RC_GetRawChannel(CRSF_MAP_STEERING);
            uint16_t thr_raw = RC_GetRawChannel(CRSF_MAP_THROTTLE);

            if (str_raw > SAFE_CENTER_MIN && str_raw < SAFE_CENTER_MAX &&
                thr_raw > SAFE_BOTTOM_MIN && thr_raw < SAFE_BOTTOM_MAX) {
                current_state = STATE_WAITING;
            }
            break;
        }

        case STATE_READY:
        	/* OPTIMIZATION: Only calculate processing changes if a fresh frame arrived */
        	if (new_frame) {
				/* Run sub-state estimation inside operating loop context */
				MainFSM_ProcessSubStates();

				if (current_mode == MODE_MANUAL) {
					/* Cache smooth control commands directly into buffer workspace */
					output_steering = RC_GetFilteredChannel(CRSF_MAP_STEERING);

					if (current_direction == DIR_NEUTRAL) {
						output_throttle = 1000; /* Electronic transmission lock active */
					} else {
						output_throttle = RC_GetFilteredChannel(CRSF_MAP_THROTTLE);
					}
				} else {
					/* Autonomous navigation algorithms (RTH_DIRECT / RTH_BACKTRACE) will be injected here */
					output_throttle = 1000;
					output_steering = 1500;
				}
        	}
            break;
    }

    /* Hardware interlock fallback: reset buffers to absolute safe baselines if state machine is unready */
    if (current_state != STATE_READY) {
        output_throttle = 1000;
        output_steering = 1500;
    }

    /* --- HARDWARE INJECTION LAYER CONTROL CHOKEPOINT --- */

    /* 1. Send the steering target command straight to the servo driver pulse generator */
    Servo_SetPulse(output_steering);

    /* 2. Execute mathematical normalization to scale microseconds down to hardware timer ARR bounds */
    int32_t baseline_gas = (int32_t)output_throttle - 1000;
    int32_t target_pwm = (baseline_gas * 4999) / 1000;

    /* 3. Evaluate selected transmission gear and drive low-level PWM registers */
    if (current_direction == DIR_FORWARD) {
        Motor_SetPower((int16_t)target_pwm, (int16_t)target_pwm);
    } else if (current_direction == DIR_REVERSE) {
        Motor_SetPower(-(int16_t)target_pwm, -(int16_t)target_pwm);
    } else {
        Motor_SetPower(0, 0); /* Force complete motor short-brake torque */
    }
}

VehicleState_t MainFSM_GetState(void) { return current_state; }
DriveMode_t MainFSM_GetMode(void) { return current_mode; }
DriveDirection_t MainFSM_GetDirection(void) { return current_direction; }

/* Pure, side-effect free deterministic data getter */
uint16_t MainFSM_GetOutput(uint8_t channel_index) {
    if (channel_index == CRSF_MAP_STEERING) return output_steering;
    if (channel_index == CRSF_MAP_THROTTLE) return output_throttle;
    return RC_GetFilteredChannel(channel_index);
}
