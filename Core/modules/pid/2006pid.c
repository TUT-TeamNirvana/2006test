/**
 * 2006pid.c
 *
 * Clean, self-contained PID implementation that honors per-feature enable flags
 * and per-instance parameters. This file implements the interface declared in
 * 2006pid.h:
 *
 *   void PID_Init(PID_t* pid, float kp, float ki, float kd, float max_output);
 *   void PID_SetFeedforward(PID_t* pid, float kff, float kaff);
 *   void PID_ConfigFeatures(PID_t* pid, bool enable_kp, bool enable_ki, bool enable_kd,
 *                           bool enable_kff, bool enable_kaff);
 *   void PID_SetParams(PID_t* pid, float deadband, float integral_limit);
 *   float PID_Calc(PID_t* pid, float ref, float feedback, float dt);
 *
 * Behavior:
 *  - Default configuration preserves original behavior: all features enabled,
 *    default deadband of 4.0f, integral_limit == 0 -> use output_max * 2.0f.
 *  - Each PID_t instance carries its own flags/params, so position and speed
 *    loops may be configured independently.
 *  - PID_Calc respects the feature flags and uses instance params.
 *
 * Notes:
 *  - This implementation intentionally preserves the "anti-windup by checking
 *    saturation direction" behavior from the existing code.
 *  - Does not change any external API signatures.
 */

#include "2006pid.h"

#ifndef PID_ERROR_DEADBAND
#define PID_ERROR_DEADBAND 4.0f
#endif

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    if (pid == NULL) return;

    /* Gains */
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* Feedforward defaults */
    pid->Kff = 0.0f;
    pid->Kaff = 0.0f;

    /* State */
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_ref = 0.0f;
    pid->output = 0.0f;
    pid->output_max = max_output;

    /* Feature flags: default to enabled for backward compatibility */
    pid->enable_kp   = true;
    pid->enable_ki   = true;
    pid->enable_kd   = true;
    pid->enable_kff  = true;
    pid->enable_kaff = true;

    /* Per-instance params */
    pid->deadband = PID_ERROR_DEADBAND;
    pid->integral_limit = 0.0f; /* 0 means use default (output_max * 2.0f) */
}

void PID_SetFeedforward(PID_t *pid, float kff, float kaff)
{
    if (pid == NULL) return;
    pid->Kff  = kff;
    pid->Kaff = kaff;
}

void PID_ConfigFeatures(PID_t* pid,
                        bool enable_kp,
                        bool enable_ki,
                        bool enable_kd,
                        bool enable_kff,
                        bool enable_kaff)
{
    if (pid == NULL) return;
    pid->enable_kp   = enable_kp;
    pid->enable_ki   = enable_ki;
    pid->enable_kd   = enable_kd;
    pid->enable_kff  = enable_kff;
    pid->enable_kaff = enable_kaff;
}

void PID_SetParams(PID_t* pid, float deadband, float integral_limit)
{
    if (pid == NULL) return;

    if (deadband > 0.0f) {
        pid->deadband = deadband;
    } else {
        pid->deadband = PID_ERROR_DEADBAND;
    }

    /* integral_limit: if <= 0, it's treated as "use default" */
    pid->integral_limit = integral_limit;
}

float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    if (pid == NULL) return 0.0f;

    /* Compute error */
    float error = ref - feedback;

    /* Determine effective deadband (instance override or default) */
    float deadband = (pid->deadband > 0.0f) ? pid->deadband : PID_ERROR_DEADBAND;

    /* Deadband: small errors treated as zero to suppress noise-driven output */
    if (fabsf(error) < deadband) {
        error = 0.0f;
        /* Reset last_error to avoid derivative kick when leaving deadband */
        pid->last_error = 0.0f;
    }

    /* Proportional term (respect feature flag) */
    float p_term = 0.0f;
    if (pid->enable_kp) {
        p_term = pid->Kp * error;
    }

    /* Derivative term (respect feature flag). Using simple discrete derivative:
       derivative = error - last_error. Note last_error is updated after computing derivative. */
    float derivative = error - pid->last_error;
    float d_term = 0.0f;
    if (pid->enable_kd) {
        d_term = pid->Kd * derivative;
    }

    /* Feedforward terms (respect flags) */
    float ff_term = 0.0f;
    if (pid->enable_kff) {
        ff_term = pid->Kff * ref;
    }

    float accel_term = 0.0f;
    if (pid->enable_kaff) {
        /* Compute acceleration-like term if dt is valid */
        if (dt > 0.000001f) {
            float acceleration = (ref - pid->last_ref) / dt;
            accel_term = pid->Kaff * acceleration;
        } else {
            accel_term = 0.0f;
        }
    }

    /* Use last_ref for next step */
    pid->last_ref = ref;

    /* Integral term contribution (only used in computing outputs; actual integral may be updated after anti-windup check) */
    float integral_term = pid->enable_ki ? (pid->Ki * pid->integral) : 0.0f;

    /* Preliminary current output with present integral (used to decide anti-windup) */
    float current_output = p_term + integral_term + d_term + ff_term + accel_term;

    /* Anti-windup: determine whether adding error to integral would cause/prolong saturation.
       Keep original behavior: allow integral when not saturating or when integral helps exit saturation. */
    int should_update_integral = 0;

    if (fabsf(current_output) <= pid->output_max) {
        /* Not currently saturated: check potential output after integral += error */
        float potential_integral = pid->integral + error;
        float potential_integral_term = pid->enable_ki ? (pid->Ki * potential_integral) : 0.0f;
        float potential_output = p_term + potential_integral_term + d_term + ff_term + accel_term;

        if (fabsf(potential_output) <= pid->output_max) {
            /* Safe to integrate */
            should_update_integral = 1;
        } else {
            /* If potential_output exceeds saturation but error direction is opposite to saturation,
               allow integration to help exit saturation */
            if ((potential_output > pid->output_max && error < 0.0f) ||
                (potential_output < -pid->output_max && error > 0.0f)) {
                should_update_integral = 1;
            } else {
                should_update_integral = 0;
            }
        }
    } else {
        /* Currently saturated: only integrate if error tends to move output back into range */
        if ((current_output > pid->output_max && error < 0.0f) ||
            (current_output < -pid->output_max && error > 0.0f)) {
            should_update_integral = 1;
        } else {
            should_update_integral = 0;
        }
    }

    /* Update integral if enabled and allowed */
    if (pid->enable_ki && should_update_integral) {
        pid->integral += error;
    }

    /* Integral clamping: either instance-specific limit or default based on output_max */
    float integral_max = (pid->integral_limit > 0.000001f) ? pid->integral_limit : (pid->output_max * 2.0f);
    if (pid->integral > integral_max) {
        pid->integral = integral_max;
    } else if (pid->integral < -integral_max) {
        pid->integral = -integral_max;
    }

    /* Recompute integral_term after potential update/clamping */
    integral_term = pid->enable_ki ? (pid->Ki * pid->integral) : 0.0f;

    /* Final output composition */
    pid->output = p_term + integral_term + d_term + ff_term + accel_term;

    /* Output limiting */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }

    /* Update last_error for next derivative computation (do after everything to match prior behavior) */
    pid->last_error = error;

    return pid->output;
}