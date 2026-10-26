/*
    control_api.h

    Flat C facade over the firmware controller core, for Python ctypes (SiL).
    PC only (uses malloc): do NOT add it to the firmware CMakeLists.

    Why a facade: Controller_t holds function pointers and firmware structs,
    and the firmware takes its gains from config.h. Here the caller passes the
    gains, gets an opaque handle and exchanges plain float structs, which
    ctypes can mirror field by field.

    Units as in the firmware:
      pitch [deg], pitch_rate [deg/s], position [m], velocity [m/s],
      out_l/out_r [A] (current command for the ESP), dt [s].
    The simulator turns the current into torque in its motor model (Kt, step 15.5).

    Typical use, one call of ctrl_step() per control period:
      h = ctrl_create(&params);
      ctrl_reset(h, &in);              // = entering BALANCING
      ctrl_step(h, &in, dt, &out);     // every period
      ctrl_destroy(h);
*/
#ifndef CONTROL_API_H_
#define CONTROL_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
  #define CTRL_API __declspec(dllexport)
#else
  #define CTRL_API __attribute__((visibility("default")))
#endif

/* Bump when a struct below or a function signature changes; control_core.py checks it */
#define CTRL_API_VERSION 1

/* Floats only -> no padding, so a ctypes.Structure with c_float fields matches 1:1.
   The field order must be the same as in control_core.py (_PidParams, _CascadeParams, _Input, _Output). */
typedef struct { float kp, ki, kd, max_output, max_i; } CtrlPidParams;
typedef struct { CtrlPidParams position, velocity, pitch; } CtrlCascadeParams;

/* Measured state: the four values the firmware puts into observed_state */
typedef struct { float pitch, pitch_rate, position, velocity; } CtrlInput;

typedef struct {
    float out_l, out_r;
    /* ControlDebug_t from the last step (the same values as in the telemetry frame) */
    float position_output, velocity_output, pitch_setpoint, pitch_output;
    float i_position, i_velocity, i_pitch;
} CtrlOutput;

typedef struct CtrlHandle CtrlHandle;   /* opaque: one controller instance */

CTRL_API int         ctrl_api_version(void);
CTRL_API CtrlHandle *ctrl_create(const CtrlCascadeParams *params);   /* NULL on error */
CTRL_API void        ctrl_destroy(CtrlHandle *h);                    /* NULL is allowed */

/* Same as entering BALANCING: reset all PIDs and set target_position = in->position */
CTRL_API void ctrl_reset(CtrlHandle *h, const CtrlInput *in);

/* Targets: target_pitch [deg], target_position [m]; higher-level commands (E3, E4) will come here */
CTRL_API void ctrl_set_target(CtrlHandle *h, float target_pitch, float target_position);

/* One controller step; dt [s] is given by the caller, there is no wall clock inside */
CTRL_API void ctrl_step(CtrlHandle *h, const CtrlInput *in, float dt, CtrlOutput *out);

#ifdef __cplusplus
}
#endif
#endif /* CONTROL_API_H_ */
