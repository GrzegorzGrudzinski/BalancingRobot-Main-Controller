/*
    control_api.c

    ctypes facade (see control_api.h). Builds the cascade from the given
    parameters instead of config.h, so the simulator can tune the gains
    without recompiling. Everything else is the firmware code:
    controlInit/controlRun/controlReset (control.c) and CASCADE_PID_* (cascade_pid.c).
*/
#include "control_api.h"

#include "control.h"
#include "cascade_pid.h"
#include "pid.h"
#include "config.h"   /* CTRL_TYPE_CASCADE_PID */

#include <stdlib.h>

/* One controller instance: the firmware Controller_t plus its cascade data */
struct CtrlHandle {
    Controller_t ctrl;
    CASCADE_t    cascade;
};

/* Flat parameters -> PID_Config_t -> the firmware pidInit() */
static void init_pid(PID_t *pid, const CtrlPidParams *p)
{
    PID_Config_t cfg = {
        .Kp = p->kp, .Ki = p->ki, .Kd = p->kd,
        .max_output = p->max_output, .max_i = p->max_i
    };
    pidInit(pid, &cfg);
}

int ctrl_api_version(void) { return CTRL_API_VERSION; }

/* Same wiring as controllers_config.c on the STM32, but with the gains from params */
CtrlHandle *ctrl_create(const CtrlCascadeParams *params)
{
    if (params == NULL) return NULL;

    CtrlHandle *h = (CtrlHandle *)calloc(1, sizeof(*h));
    if (h == NULL) return NULL;

    init_pid(&h->cascade.pid_position,    &params->position);
    init_pid(&h->cascade.pid_velocity,    &params->velocity);
    init_pid(&h->cascade.pid_pitch_angle, &params->pitch);

    h->ctrl.ctrl_type = CTRL_TYPE_CASCADE_PID;
    h->ctrl.init  = CASCADE_PID_InitFunct;
    h->ctrl.run   = CASCADE_PID_RunFunct;
    h->ctrl.reset = CASCADE_PID_ResetFunct;
    controlInit(&h->ctrl, &h->cascade);
    return h;
}

void ctrl_destroy(CtrlHandle *h) { free(h); }

/* Measured state goes where the firmware keeps it (updateControlState() on the STM32) */
static void load_input(Controller_t *c, const CtrlInput *in)
{
    c->observed_state.pitch      = in->pitch;
    c->observed_state.pitch_rate = in->pitch_rate;
    c->observed_state.position   = in->position;
    c->observed_state.velocity   = in->velocity;
}

void ctrl_reset(CtrlHandle *h, const CtrlInput *in)
{
    if (h == NULL || in == NULL) return;
    load_input(&h->ctrl, in);
    controlReset(&h->ctrl);                                  /* PIDs reset, as on CMD_START */
    h->ctrl.target_state.target_position = in->position;   /* hold the current position (controlReset() sets it too) */
}

void ctrl_set_target(CtrlHandle *h, float target_pitch, float target_position)
{
    if (h == NULL) return;
    h->ctrl.target_state.target_pitch    = target_pitch;
    h->ctrl.target_state.target_position = target_position;
}

void ctrl_step(CtrlHandle *h, const CtrlInput *in, float dt, CtrlOutput *out)
{
    if (h == NULL || in == NULL || out == NULL) return;

    load_input(&h->ctrl, in);

    /* firmware cascade: position -> velocity -> pitch -> current; out_l == out_r (no yaw loop yet) */
    float l = 0.0f, r = 0.0f;
    controlRun(&h->ctrl, dt, &l, &r);

    /* debug values of this step, for plots and tests */
    const ControlDebug_t *d = &h->ctrl.debug;
    out->out_l = l;
    out->out_r = r;
    out->position_output = d->position_output;
    out->velocity_output = d->velocity_output;
    out->pitch_setpoint  = d->pitch_setpoint;
    out->pitch_output    = d->pitch_output;
    out->i_position      = d->i_position;
    out->i_velocity      = d->i_velocity;
    out->i_pitch         = d->i_pitch;
}
