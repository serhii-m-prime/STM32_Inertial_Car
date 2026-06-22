#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>

/* Define how the physical radio stick behaves hardware-wise */
typedef enum {
    CHAN_CENTERED,          /* Sticks with spring return to center (e.g., Steering) */
    CHAN_BOTTOM_ANCHORED    /* FPV style throttle sticks without spring return */
} ChannelType_t;

typedef struct {
    uint16_t center;        /* Neutral baseline (1500 for centered, ~1000 for FPV throttle) */
    uint16_t deadband;      /* Hard hardware noise suppression zone */
    float alpha;            /* EMA low-pass filter smoothing coefficient */
    float filtered_val;     /* History buffer for exponential moving average */
    ChannelType_t type;     /* Configuration type profile flag */
} ChannelFilter_t;

void Signal_InitFilter(ChannelFilter_t* filter, uint16_t center, uint16_t deadband, float alpha, ChannelType_t type);
uint16_t Signal_Process(ChannelFilter_t* filter, uint16_t raw_pwm);

#endif // SIGNAL_PROCESSING_H
