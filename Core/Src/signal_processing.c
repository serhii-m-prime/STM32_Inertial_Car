#include "signal_processing.h"
#include <stdlib.h>

void Signal_InitFilter(ChannelFilter_t* filter, uint16_t center, uint16_t deadband, float alpha, ChannelType_t type) {
    filter->center = center;
    filter->deadband = deadband;
    filter->alpha = alpha;
    filter->type = type;
    filter->filtered_val = (float)center;
}

uint16_t Signal_Process(ChannelFilter_t* filter, uint16_t raw_pwm) {
    float target_pwm = (float)filter->center;

    if (filter->type == CHAN_CENTERED) {
        int16_t deviation = (int16_t)raw_pwm - (int16_t)filter->center;

        // Centered deadband check
        if (deviation >= -filter->deadband && deviation <= filter->deadband) {
            filter->filtered_val = (float)filter->center;
            return filter->center;
        }

        // Centered linearization scaling math
        float max_throw = (deviation > 0) ? (2000.0f - filter->center) : (filter->center - 1000.0f);
        float deviation_linear;

        if (deviation > 0) {
            deviation_linear = (float)(deviation - filter->deadband) * (max_throw / (max_throw - filter->deadband));
        } else {
            deviation_linear = (float)(deviation + filter->deadband) * (max_throw / (max_throw - filter->deadband));
        }
        target_pwm = (float)filter->center + deviation_linear;

    } else if (filter->type == CHAN_BOTTOM_ANCHORED) {
        // Enforce hard clamp against lower noise under calibrated zero baseline
        if (raw_pwm < filter->center) raw_pwm = filter->center;

        int16_t deviation = (int16_t)raw_pwm - (int16_t)filter->center;

        // Bottom deadband isolation check
        if (deviation <= filter->deadband) {
            filter->filtered_val = (float)filter->center;
            return filter->center;
        }

        // Unidirectional bottom-anchored linearization scaling math
        float max_throw = 2000.0f - filter->center;
        float deviation_linear = (float)(deviation - filter->deadband) * (max_throw / (max_throw - filter->deadband));

        target_pwm = (float)filter->center + deviation_linear;
    }

    // Global Exponential Moving Average (EMA) low-pass smoothing step
    filter->filtered_val = (filter->alpha * target_pwm) + ((1.0f - filter->alpha) * filter->filtered_val);

    // Hard boundary safety constraints checks
    if (filter->filtered_val < 1000.0f) return 1000;
    if (filter->filtered_val > 2000.0f) return 2000;

    return (uint16_t)filter->filtered_val;
}
