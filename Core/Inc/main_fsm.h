#ifndef MAIN_FSM_H
#define MAIN_FSM_H

#include <stdint.h>
#include <stdbool.h>

/* Global system safety states */
typedef enum {
    STATE_WAITING,
    VEHICLE_CALIBRATING,
    STATE_ERROR,
    STATE_READY
} VehicleState_t;

/* Supervisor navigation control modes */
typedef enum {
    MODE_MANUAL,
    MODE_RTH_DIRECT,
    MODE_RTH_BACKTRACE
} DriveMode_t;

/* Physical transmission direction profiles */
typedef enum {
    DIR_REVERSE,
    DIR_NEUTRAL,
    DIR_FORWARD
} DriveDirection_t;

void MainFSM_Init(void);
void MainFSM_Update(bool new_frame);

/* Public getters for UI architecture and hardware abstraction layer scaling */
VehicleState_t MainFSM_GetState(void);
DriveMode_t MainFSM_GetMode(void);
DriveDirection_t MainFSM_GetDirection(void);

uint16_t MainFSM_GetOutput(uint8_t channel_index);

#endif // MAIN_FSM_H
