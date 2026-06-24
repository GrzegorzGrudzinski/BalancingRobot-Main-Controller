/*
  control_state.h
*/

#ifndef CONTROL_STATE_H_
#define CONTROL_STATE_H_

#include <stdint.h>

typedef struct
{
    float pitch;
    float pitch_rate;
    float yaw;
    float yaw_rate;
    float velocity;
    float position;
    float left_wheel_pos;
    float right_wheel_pos;
} ControlState_t;

typedef enum
{
    CTRL_TYPE_PID,
    CTRL_TYPE_CASCADE_PID
} ControllerType_e;

void updateControlState(ControlState_t *state);


#endif /* CONTROL_STATE_H_ */
