/*
  control_state.h
*/

#ifndef CONTROL_STATE_H_
#define CONTROL_STATE_H_

#include <stdint.h>

#include "dualFOC_communication.h" // for feedback packet
typedef struct
{
    float pitch;
    float pitch_rate;
    float yaw;
    float yaw_rate;
    float position;
    float velocity;
    float left_wheel_pos;
    float right_wheel_pos;
} ControlState_t;

typedef struct
{
    float target_pitch;
    float target_yaw;
    float target_position;
    float target_velocity;
} TargetState_t;

/*
    Wewnętrzne wartości regulatora z ostatniego przebiegu (wypełnia je sam regulator).
    Razem z ControlState_t + TargetState_t opisują całą decyzję:
      target_position -> [PID pozycji]  -> position_output (= zadana prędkość)
      zadana prędkość -> [PID prędkości] -> velocity_output (surowe) -> pitch_setpoint = target_pitch - velocity_output
      pitch_setpoint  -> [PID pitch]    -> pitch_output -> out_l/out_r
    Człony P i D da się odtworzyć na PC ze stanu i wzmocnień, I nie
*/
typedef struct
{
    float position_output;  // wyjście pętli pozycji = zadana prędkość [m/s]
    float velocity_output;  // surowe wyjście pętli prędkości (przed zmianą znaku)
    float pitch_setpoint;   // końcowy setpoint kąta dla pętli pitch [deg]
    float pitch_output;     // wyjście pętli pitch = komenda silników
    float i_position;       // człon I pętli pozycji
    float i_velocity;       // człon I pętli prędkości
    float i_pitch;          // człon I pętli pitch
} ControlDebug_t;

void updateControlState(ControlState_t *state, const FeedbackPacket_t *fb);

#endif /* CONTROL_STATE_H_ */
