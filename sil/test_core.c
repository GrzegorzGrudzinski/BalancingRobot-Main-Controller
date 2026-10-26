/*
    test_core.c - unit tests of the controller core (no framework, no HAL)
    Run: make -C sil test

    Part 1 tests a single PID (pid.c), part 2 the whole cascade through the
    ctypes facade (control_api.c), i.e. exactly what the simulator calls.
*/
#include "pid.h"
#include "control_api.h"

#include <math.h>
#include <stdio.h>

static int g_fail = 0, g_checks = 0;

/* Counts every check; prints file, line and the failed condition, then a printf-style message */
#define CHECK(cond, ...) do { \
    g_checks++; \
    if (!(cond)) { g_fail++; printf("  FAIL %s:%d  %s  -> ", __FILE__, __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static PID_t make_pid(float kp, float ki, float kd, float max_out, float max_i)
{
    PID_t p;
    PID_Config_t c = { kp, ki, kd, max_out, max_i };
    pidInit(&p, &c);
    return p;
}

/* ---- PID: single step ---- */
static void test_p_only(void)
{
    PID_t p = make_pid(2.0f, 0, 0, 100, 100);
    float y = run_pid_rate(&p, 0.0f, 0.0f, 1.0f, 0.005f);
    CHECK(NEAR(y, 2.0f, 1e-6f), "y=%f", y);
}

static void test_d_on_rate_sign(void)
{
    /* D = -Kd * rate: a positive rate (measurement going up) damps the output */
    PID_t p = make_pid(0, 0, 0.04f, 100, 100);
    float y = run_pid_rate(&p, 0.0f, 50.0f, 0.0f, 0.005f);
    CHECK(NEAR(y, -2.0f, 1e-6f), "y=%f", y);
}

static void test_output_saturation(void)
{
    PID_t p = make_pid(10.0f, 0, 0, 2.0f, 1.0f);
    CHECK(NEAR(run_pid_rate(&p, 0, 0,  1, 0.005f),  2.0f, 1e-6f), "pos sat");
    CHECK(NEAR(run_pid_rate(&p, 0, 0, -1, 0.005f), -2.0f, 1e-6f), "neg sat");
}

static void test_integrator_accumulates(void)
{
    PID_t p = make_pid(0, 1.0f, 0, 100, 100);
    for (int i = 0; i < 200; i++) run_pid_rate(&p, 0, 0, 1.0f, 0.005f);   /* 1 s */
    CHECK(NEAR(p.I, 1.0f, 1e-3f), "I=%f", p.I);
}

static void test_antiwindup_recovers_immediately(void)
{
    /* back-calculation: at the limit error_sum is clamped too, so the integral leaves the limit at once */
    PID_t p = make_pid(0, 1.0f, 0, 100, 0.2f);
    for (int i = 0; i < 1000; i++) run_pid_rate(&p, 0, 0, 10.0f, 0.005f);  /* long time saturated */
    CHECK(NEAR(p.I, 0.2f, 1e-6f), "I=%f", p.I);
    CHECK(NEAR(p.error_sum, 0.2f, 1e-5f), "error_sum=%f (back-calculation)", p.error_sum);

    run_pid_rate(&p, 0, 0, -10.0f, 0.005f);                                 /* error changes sign */
    CHECK(p.I < 0.2f - 1e-3f, "I=%f - should leave the limit at once", p.I);
}

static void test_dt_nonpositive_is_noop(void)
{
    PID_t p = make_pid(2.0f, 1.0f, 0, 100, 100);
    float y1 = run_pid_rate(&p, 0, 0, 1.0f, 0.005f);
    float s1 = p.error_sum;
    float y2 = run_pid_rate(&p, 0, 0, 5.0f, 0.0f);
    float y3 = run_pid_rate(&p, 0, 0, 5.0f, -1.0f);
    CHECK(y2 == y1 && y3 == y1, "output must not change");
    CHECK(p.error_sum == s1, "state must not change");
}

static void test_reset(void)
{
    PID_t p = make_pid(1.0f, 1.0f, 0, 100, 100);
    for (int i = 0; i < 50; i++) run_pid_rate(&p, 0, 0, 1.0f, 0.005f);
    pid_reset(&p, 3.0f);
    CHECK(p.error_sum == 0 && p.I == 0 && p.output == 0 && p.lastPos == 3.0f, "reset");
}

/* ---- PID: closed loop with a simple plant x' = u - d ---- */
static void test_step_response_p(void)
{
    PID_t p = make_pid(2.0f, 0, 0, 100, 100);
    float x = 0, dt = 0.005f, xmax = 0;
    for (int i = 0; i < 500; i++) {                 /* 2.5 s = 5 time constants */
        float u = run_pid_rate(&p, x, 0, 1.0f, dt);
        x += u * dt;
        if (x > xmax) xmax = x;
    }
    CHECK(NEAR(x, 1.0f, 0.01f), "x(2.5s)=%f", x);
    CHECK(xmax <= 1.0f + 1e-4f, "no overshoot, xmax=%f", xmax);
}

static void test_integral_removes_steady_state_error(void)
{
    const float d = 0.3f, dt = 0.005f;              /* constant disturbance d */
    PID_t pp = make_pid(2.0f, 0.0f, 0, 100, 1.0f);
    PID_t pi = make_pid(2.0f, 2.0f, 0, 100, 1.0f);
    float xp = 0, xi = 0;
    for (int i = 0; i < 4000; i++) {                /* 20 s */
        xp += (run_pid_rate(&pp, xp, 0, 1.0f, dt) - d) * dt;
        xi += (run_pid_rate(&pi, xi, 0, 1.0f, dt) - d) * dt;
    }
    CHECK(NEAR(xp, 1.0f - d / 2.0f, 0.01f), "P keeps an error of d/Kp, xp=%f", xp);
    CHECK(NEAR(xi, 1.0f, 1e-3f), "PI removes the error, xi=%f", xi);
}

/* ---- Cascade through the facade ---- */
static CtrlCascadeParams cascade_params(void)   /* = config.h (keep in sync by hand until step 16.1) */
{
    CtrlCascadeParams p = {
        .position = { 0.40f, 0.0f,  0.0f,  1.0f, 0.3f * 1.0f },
        .velocity = { 6.00f, 0.50f, 0.0f,  8.0f, 0.3f * 8.0f },
        .pitch    = { 0.80f, 0.0f,  0.04f, 2.5f, 0.3f * 2.5f },
    };
    return p;
}

static void test_api_basics(void)
{
    CHECK(ctrl_api_version() == CTRL_API_VERSION, "version");
    CHECK(ctrl_create(NULL) == NULL, "NULL params");
    ctrl_destroy(NULL);                              /* must not crash */
}

static void test_cascade_first_step_after_reset(void)
{
    CtrlCascadeParams p = cascade_params();
    CtrlHandle *h = ctrl_create(&p);
    CtrlInput in = { .pitch = 1.0f };
    CtrlOutput o;
    ctrl_reset(h, &in);
    ctrl_step(h, &in, 0.005f, &o);
    /* 1 deg of tilt: Kp*(0-1) = -0.8 already in the first step (no time gate) */
    CHECK(NEAR(o.out_l, -0.8f, 1e-5f) && o.out_l == o.out_r, "out=%f/%f", o.out_l, o.out_r);
    ctrl_destroy(h);
}

static void test_cascade_independent_of_wall_clock(void)
{
    /* 200 calls in a tight loop: the old HAL_GetTick() gate skipped almost all of them.
       Now the integral has to grow by exactly N*dt*error, whatever the wall-clock time. */
    CtrlCascadeParams p = cascade_params();
    CtrlHandle *h = ctrl_create(&p);
    CtrlInput in = { .velocity = -1.0f };            /* velocity error = +1 */
    CtrlOutput o;
    ctrl_reset(h, &in);
    for (int i = 0; i < 200; i++) ctrl_step(h, &in, 0.005f, &o);
    CHECK(NEAR(o.i_velocity, 0.5f * 1.0f * 1.0f, 1e-3f), "i_velocity=%f", o.i_velocity);
    ctrl_destroy(h);
}

static void test_cascade_deterministic_and_dt_scaling(void)
{
    CtrlCascadeParams p = cascade_params();
    CtrlHandle *a = ctrl_create(&p), *b = ctrl_create(&p), *c = ctrl_create(&p);
    CtrlInput in = { .pitch = 0.5f, .pitch_rate = 3.0f, .position = 0.1f, .velocity = -0.2f };
    CtrlOutput oa, ob, oc;
    ctrl_reset(a, &in); ctrl_reset(b, &in); ctrl_reset(c, &in);
    for (int i = 0; i < 100; i++) {
        ctrl_step(a, &in, 0.005f, &oa);
        ctrl_step(b, &in, 0.005f, &ob);
        ctrl_step(c, &in, 0.0025f, &oc);             /* half dt -> slower integral */
    }
    CHECK(oa.out_l == ob.out_l && oa.i_velocity == ob.i_velocity, "same inputs -> same outputs");
    CHECK(oc.i_velocity < oa.i_velocity, "dt scales the integral: %f vs %f", oc.i_velocity, oa.i_velocity);
    ctrl_destroy(a); ctrl_destroy(b); ctrl_destroy(c);
}

static void test_cascade_reset_zeroes_state(void)
{
    CtrlCascadeParams p = cascade_params();
    CtrlHandle *h = ctrl_create(&p);
    CtrlInput in = { .velocity = -1.0f };
    CtrlOutput o;
    for (int i = 0; i < 100; i++) ctrl_step(h, &in, 0.005f, &o);
    CHECK(o.i_velocity > 0.1f, "integral has grown");
    CtrlInput at = { .position = 2.0f };
    ctrl_reset(h, &at);
    ctrl_step(h, &at, 0.005f, &o);
    CHECK(NEAR(o.i_velocity, 0.0f, 1e-6f) && NEAR(o.position_output, 0.0f, 1e-6f),
          "after reset target_position = position -> no jerk, vel_out=%f", o.position_output);
    ctrl_destroy(h);
}

int main(void)
{
    test_p_only();
    test_d_on_rate_sign();
    test_output_saturation();
    test_integrator_accumulates();
    test_antiwindup_recovers_immediately();
    test_dt_nonpositive_is_noop();
    test_reset();
    test_step_response_p();
    test_integral_removes_steady_state_error();
    test_api_basics();
    test_cascade_first_step_after_reset();
    test_cascade_independent_of_wall_clock();
    test_cascade_deterministic_and_dt_scaling();
    test_cascade_reset_zeroes_state();

    printf("%d/%d checks OK%s\n", g_checks - g_fail, g_checks, g_fail ? "  <-- FAILURES" : "");
    return g_fail ? 1 : 0;
}
