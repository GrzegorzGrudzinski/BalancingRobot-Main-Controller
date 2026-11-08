/*
    control_types.h


*/

#ifndef CONTROL_TYPES_H_
#define CONTROL_TYPES_H_

typedef struct
{
    float pitch;           // [deg]
    float pitch_rate;      // [deg/s]
    float yaw;             // [deg]
    float yaw_rate;        // [deg/s] 
    float position;        // [m]
    float velocity;        // [m/s]
    float left_wheel_pos;
    float right_wheel_pos;
} ControlState_t;

typedef struct
{
    float target_pitch;    // set by targetReset() (TARGET_ANGLE); offset of the pitch loop
    float target_yaw;      // set by targetReset() (current yaw); unused yet
    float target_position; // set by targetReset() and PC SET_TARGET TARGET_POS
    float target_velocity; // set by targetReset() (0); unused yet
} TargetState_t;

/*
 * Internal controller values from the last iteration (populated by the controller itself).
 * Together with ControlState_t and TargetState_t, they describe the complete control decision:
 *   target_position  -> [Position PID] -> position_output (= target velocity)
 *   target velocity  -> [Velocity PID] -> velocity_output (raw)
 *                    -> pitch_setpoint = target_pitch - velocity_output
 *   pitch_setpoint   -> [Pitch PID]    -> pitch_output -> out_l/out_r
 */
typedef struct
{
    float position_output;  // Position loop output = target velocity [m/s]
    float velocity_output;  // Raw velocity loop output (before sign inversion)
    float pitch_setpoint;   // Final angle setpoint for the pitch loop [deg]
    float pitch_output;     // Pitch loop output = motor commands
    float i_position;       // Integral term of the position loop
    float i_velocity;       // Integral term of the velocity loop
    float i_pitch;          // Integral term of the pitch loop
} ControlDebug_t;



#endif /* CONTROL_TYPES_H_ */
