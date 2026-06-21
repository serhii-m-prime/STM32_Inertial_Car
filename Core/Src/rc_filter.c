#include "rc_filter.h"
#include "crsf.h"

#define CALIB_SAMPLES 50
#define SAFE_CENTER_MIN 1400
#define SAFE_CENTER_MAX 1600
#define ALLOWED_JITTER 30

RC_SystemState_t rc_sys_state = RC_STATE_WAITING;
RC_Channel_Config_t rc_configs[16];

static uint16_t calib_count = 0;
static uint32_t calib_sum[2] = {0, 0};
static uint16_t calib_min[2] = {2000, 2000};
static uint16_t calib_max[2] = {1000, 1000};

void RC_Filter_Init(void) {
    for (int i = 0; i < 16; i++) {
        rc_configs[i].center = 1500;
        rc_configs[i].deadband = 12;
        rc_configs[i].alpha = 0.3f;       // Smooth analog inputs
        rc_configs[i].filtered_val = 1500.0f;
    }

    // Switches need raw instant speed, no smoothing
    rc_configs[CRSF_MAP_DIR_SWITCH].alpha = 1.0f;
    rc_configs[CRSF_MAP_RTH_SWITCH].alpha = 1.0f;
    rc_configs[CRSF_MAP_DIR_SWITCH].deadband = 0;
    rc_configs[CRSF_MAP_RTH_SWITCH].deadband = 0;
}

void RC_ProcessCalibration(void) {
    if (!crsf.is_connected) {
        rc_sys_state = RC_STATE_WAITING;
        calib_count = 0;
        return;
    }

    if (rc_sys_state == RC_STATE_WAITING) {
        rc_sys_state = RC_STATE_CALIBRATING;
        calib_count = 0;
        calib_sum[0] = 0; calib_sum[1] = 0;
        calib_min[0] = 2000; calib_min[1] = 2000;
        calib_max[0] = 1000; calib_max[1] = 1000;
    }

    if (rc_sys_state == RC_STATE_CALIBRATING) {
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

            bool is_centered = (str_avg > SAFE_CENTER_MIN && str_avg < SAFE_CENTER_MAX) &&
                               (thr_avg > SAFE_CENTER_MIN && thr_avg < SAFE_CENTER_MAX);

            if (is_stable && is_centered) {
                rc_configs[CRSF_MAP_STEERING].center = str_avg;
                rc_configs[CRSF_MAP_THROTTLE].center = thr_avg;

                rc_configs[CRSF_MAP_STEERING].filtered_val = (float)str_avg;
                rc_configs[CRSF_MAP_THROTTLE].filtered_val = (float)thr_avg;

                rc_sys_state = RC_STATE_READY;
            } else {
                rc_sys_state = RC_STATE_ERROR;
            }
        }
    }
}

uint16_t RC_GetProcessed(uint8_t channel_index) {
    if (channel_index >= 16) return 1500;

    uint16_t raw_pwm = CRSF_GetPWM(channel_index);
    RC_Channel_Config_t* cfg = &rc_configs[channel_index];

    // Block motors if FSM is not READY
    if (rc_sys_state != RC_STATE_READY) {
        if (channel_index == CRSF_MAP_THROTTLE || channel_index == CRSF_MAP_STEERING) {
            return 1500;
        }
    }

    // 1. Deadband logic
    uint16_t deadband_pwm = raw_pwm;
    int16_t deviation = (int16_t)raw_pwm - (int16_t)cfg->center;
    if (deviation >= -cfg->deadband && deviation <= cfg->deadband) {
        deadband_pwm = cfg->center;
    }

    // 2. EMA Filter (CSTR chemical tank analogy)
    cfg->filtered_val = (cfg->alpha * (float)deadband_pwm) + ((1.0f - cfg->alpha) * cfg->filtered_val);

    return (uint16_t)cfg->filtered_val;
}
