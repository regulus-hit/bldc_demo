/**********************************
 * PID Auto-Tuning Module Implementation
 * 
 * Model-based auto-tuning of current loop PI controllers.
 * Uses identified motor parameters (R, L) to calculate optimal gains.
 * 
 * Algorithm (based on TI InstaSPIN and control theory):
 * - Kp = L × ωc (proportional gain)
 * - Ki = R × ωc (integral gain)
 * - Kb = ωc / 10 (anti-windup gain, empirical)
 * 
 * Where ωc is the desired closed-loop bandwidth in rad/s.
 * Typical bandwidth: 1/10 to 1/5 of PWM frequency (1-2 kHz for 10 kHz PWM)
 * 
 * References:
 * - Texas Instruments InstaSPIN-FOC User's Guide
 * - SimpleFOC library documentation
 * - "Modern Power Electronics and AC Drives" - Bose
 **********************************/
#include "pid_autotune.h"
#include "main.h"
#include <math.h>

/* Default configuration values */
#define DEFAULT_TARGET_BANDWIDTH_HZ     1000.0f     /* 1 kHz bandwidth (1/10 of 10 kHz PWM) */
#define DEFAULT_MIN_BANDWIDTH_HZ        500.0f      /* Minimum safe bandwidth */
#define DEFAULT_MAX_BANDWIDTH_HZ        2000.0f     /* Maximum safe bandwidth */
#define DEFAULT_SAFETY_MARGIN           0.8f        /* Use 80% of calculated bandwidth */
#define CONVERGENCE_THRESHOLD           0.05f       /* 5% parameter change threshold */
#define STABLE_TIME_MS                  2000        /* 2 seconds stability wait */
#define MAX_TUNE_TIME_MS                10000       /* 10 seconds maximum tuning time */

/* Mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define TWO_PI (2.0f * M_PI)

/* Parameter validation thresholds */
#define MIN_VALID_RESISTANCE            0.01f       /* 10 mΩ minimum */
#define MAX_VALID_RESISTANCE            10.0f       /* 10 Ω maximum */
#define MIN_VALID_INDUCTANCE            0.00001f    /* 10 µH minimum */
#define MAX_VALID_INDUCTANCE            0.1f        /* 100 mH maximum */

/* Global auto-tune instance */
PID_AUTOTUNE_DEF PID_Autotune;

/* Static helper functions */
static uint8_t validate_motor_parameters(real32_T Rs, real32_T Ls);
static uint8_t check_convergence(real32_T Rs, real32_T Ls);
static void calculate_pi_gains(real32_T Rs, real32_T Ls, real32_T bandwidth_hz);
static uint32_t get_system_time_ms(void);

/**
 * @brief Initialize auto-tuning module
 */
void pid_autotune_init(void)
{
    /* Initialize configuration */
    PID_Autotune.target_bandwidth_hz = DEFAULT_TARGET_BANDWIDTH_HZ;
    PID_Autotune.min_bandwidth_hz = DEFAULT_MIN_BANDWIDTH_HZ;
    PID_Autotune.max_bandwidth_hz = DEFAULT_MAX_BANDWIDTH_HZ;
    PID_Autotune.safety_margin = DEFAULT_SAFETY_MARGIN;
    PID_Autotune.convergence_threshold = CONVERGENCE_THRESHOLD;
    PID_Autotune.stable_time_ms = STABLE_TIME_MS;
    PID_Autotune.max_tune_time_ms = MAX_TUNE_TIME_MS;
    
    /* Initialize state */
    PID_Autotune.state = AUTOTUNE_IDLE;
    PID_Autotune.state_entry_time = 0;
    PID_Autotune.tune_start_time = 0;
    
    /* Initialize parameters */
    PID_Autotune.Rs_identified = 0.0f;
    PID_Autotune.Ls_identified = 0.0f;
    PID_Autotune.Rs_prev = 0.0f;
    PID_Autotune.Ls_prev = 0.0f;
    
    /* Initialize calculated gains */
    PID_Autotune.Kp_calculated = 0.0f;
    PID_Autotune.Ki_calculated = 0.0f;
    PID_Autotune.Kb_calculated = 0.0f;
    
    /* Initialize original gains */
    PID_Autotune.Kp_original = 0.0f;
    PID_Autotune.Ki_original = 0.0f;
    PID_Autotune.Kb_original = 0.0f;
    
    /* Initialize flags */
    PID_Autotune.enabled = 0;
    PID_Autotune.tune_triggered = 0;
    PID_Autotune.params_valid = 0;
    PID_Autotune.gains_applied = 0;
}

/**
 * @brief Start auto-tuning sequence
 */
uint8_t pid_autotune_start(void)
{
    /* Check if already running */
    if (PID_Autotune.state != AUTOTUNE_IDLE) {
        return 0;
    }
    
    /* Enable auto-tune */
    PID_Autotune.enabled = 1;
    PID_Autotune.tune_triggered = 1;
    PID_Autotune.state = AUTOTUNE_WAIT_STABLE;
    PID_Autotune.state_entry_time = get_system_time_ms();
    PID_Autotune.tune_start_time = get_system_time_ms();
    
    /* Clear flags */
    PID_Autotune.params_valid = 0;
    PID_Autotune.gains_applied = 0;
    
    return 1;
}

/**
 * @brief Stop auto-tuning and restore original gains
 */
void pid_autotune_stop(void)
{
    /* Only restore if gains were applied */
    if (PID_Autotune.gains_applied) {
        /* Gains will be restored by caller */
    }
    
    /* Reset state */
    PID_Autotune.state = AUTOTUNE_IDLE;
    PID_Autotune.enabled = 0;
    PID_Autotune.tune_triggered = 0;
    PID_Autotune.gains_applied = 0;
}

/**
 * @brief Execute one auto-tune cycle
 */
void pid_autotune_step(real32_T Rs_current, real32_T Ls_current,
                       real32_T* Kp_current, real32_T* Ki_current, real32_T* Kb_current)
{
    uint32_t current_time;
    uint32_t elapsed_time;
    
    /* Check if auto-tune is enabled */
    if (!PID_Autotune.enabled || PID_Autotune.state == AUTOTUNE_IDLE) {
        return;
    }
    
    current_time = get_system_time_ms();
    
    /* Check for timeout */
    if ((current_time - PID_Autotune.tune_start_time) > PID_Autotune.max_tune_time_ms) {
        PID_Autotune.state = AUTOTUNE_FAILED;
        PID_Autotune.enabled = 0;
        return;
    }
    
    /* State machine */
    switch (PID_Autotune.state) {
        
        case AUTOTUNE_WAIT_STABLE:
            /* Wait for motor to stabilize before starting parameter identification */
            elapsed_time = current_time - PID_Autotune.state_entry_time;
            
            if (elapsed_time >= PID_Autotune.stable_time_ms) {
                /* Save original gains */
                PID_Autotune.Kp_original = *Kp_current;
                PID_Autotune.Ki_original = *Ki_current;
                PID_Autotune.Kb_original = *Kb_current;
                
                /* Move to parameter identification */
                PID_Autotune.state = AUTOTUNE_IDENTIFY_PARAMS;
                PID_Autotune.state_entry_time = current_time;
                PID_Autotune.Rs_prev = Rs_current;
                PID_Autotune.Ls_prev = Ls_current;
            }
            break;
            
        case AUTOTUNE_IDENTIFY_PARAMS:
            /* Wait for parameter identification to converge */
            PID_Autotune.Rs_identified = Rs_current;
            PID_Autotune.Ls_identified = Ls_current;
            
            /* Validate parameters */
            if (!validate_motor_parameters(Rs_current, Ls_current)) {
                /* Parameters out of range */
                elapsed_time = current_time - PID_Autotune.state_entry_time;
                if (elapsed_time > (PID_Autotune.stable_time_ms * 2)) {
                    /* Timeout waiting for valid parameters */
                    PID_Autotune.state = AUTOTUNE_FAILED;
                    PID_Autotune.enabled = 0;
                }
                break;
            }
            
            /* Check convergence */
            if (check_convergence(Rs_current, Ls_current)) {
                PID_Autotune.params_valid = 1;
                PID_Autotune.state = AUTOTUNE_CALCULATE_GAINS;
                PID_Autotune.state_entry_time = current_time;
            }
            
            /* Update previous values */
            PID_Autotune.Rs_prev = Rs_current;
            PID_Autotune.Ls_prev = Ls_current;
            break;
            
        case AUTOTUNE_CALCULATE_GAINS:
            /* Calculate new PI gains based on identified parameters */
            calculate_pi_gains(PID_Autotune.Rs_identified, 
                             PID_Autotune.Ls_identified,
                             PID_Autotune.target_bandwidth_hz);
            
            /* Move to apply gains */
            PID_Autotune.state = AUTOTUNE_APPLY_GAINS;
            PID_Autotune.state_entry_time = current_time;
            break;
            
        case AUTOTUNE_APPLY_GAINS:
            /* Apply calculated gains */
            *Kp_current = PID_Autotune.Kp_calculated;
            *Ki_current = PID_Autotune.Ki_calculated;
            *Kb_current = PID_Autotune.Kb_calculated;
            
            PID_Autotune.gains_applied = 1;
            
            /* Wait a bit for system to stabilize with new gains */
            elapsed_time = current_time - PID_Autotune.state_entry_time;
            if (elapsed_time >= 500) {  /* 500 ms settling time */
                PID_Autotune.state = AUTOTUNE_COMPLETE;
                PID_Autotune.enabled = 0;
            }
            break;
            
        case AUTOTUNE_COMPLETE:
            /* Auto-tune completed successfully */
            PID_Autotune.enabled = 0;
            break;
            
        case AUTOTUNE_FAILED:
            /* Auto-tune failed - restore original gains */
            if (PID_Autotune.gains_applied) {
                *Kp_current = PID_Autotune.Kp_original;
                *Ki_current = PID_Autotune.Ki_original;
                *Kb_current = PID_Autotune.Kb_original;
            }
            PID_Autotune.enabled = 0;
            break;
            
        default:
            PID_Autotune.state = AUTOTUNE_IDLE;
            PID_Autotune.enabled = 0;
            break;
    }
}

/**
 * @brief Get auto-tune status
 */
AUTOTUNE_STATE pid_autotune_get_state(void)
{
    return PID_Autotune.state;
}

/**
 * @brief Check if auto-tune is complete
 */
uint8_t pid_autotune_is_complete(void)
{
    return (PID_Autotune.state == AUTOTUNE_COMPLETE || 
            PID_Autotune.state == AUTOTUNE_FAILED);
}

/**
 * @brief Get calculated gains
 */
uint8_t pid_autotune_get_gains(real32_T* Kp, real32_T* Ki, real32_T* Kb)
{
    if (!PID_Autotune.params_valid) {
        return 0;
    }
    
    *Kp = PID_Autotune.Kp_calculated;
    *Ki = PID_Autotune.Ki_calculated;
    *Kb = PID_Autotune.Kb_calculated;
    
    return 1;
}

/**
 * @brief Validate motor parameters
 */
static uint8_t validate_motor_parameters(real32_T Rs, real32_T Ls)
{
    /* Check resistance bounds */
    if (Rs < MIN_VALID_RESISTANCE || Rs > MAX_VALID_RESISTANCE) {
        return 0;
    }
    
    /* Check inductance bounds */
    if (Ls < MIN_VALID_INDUCTANCE || Ls > MAX_VALID_INDUCTANCE) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief Check parameter convergence
 */
static uint8_t check_convergence(real32_T Rs, real32_T Ls)
{
    real32_T Rs_change, Ls_change;
    
    /* Calculate relative change */
    if (PID_Autotune.Rs_prev > 0.0f) {
        Rs_change = fabsf((Rs - PID_Autotune.Rs_prev) / PID_Autotune.Rs_prev);
    } else {
        Rs_change = 1.0f;  /* No convergence yet */
    }
    
    if (PID_Autotune.Ls_prev > 0.0f) {
        Ls_change = fabsf((Ls - PID_Autotune.Ls_prev) / PID_Autotune.Ls_prev);
    } else {
        Ls_change = 1.0f;  /* No convergence yet */
    }
    
    /* Both parameters must converge */
    return (Rs_change < PID_Autotune.convergence_threshold && 
            Ls_change < PID_Autotune.convergence_threshold);
}

/**
 * @brief Calculate PI gains using model-based formulas
 * 
 * Based on TI InstaSPIN and control theory:
 * - Current loop transfer function: G(s) = 1/(Ls + Rs)
 * - PI controller zero cancels motor pole: Ti = L/R
 * - Bandwidth determines proportional gain: Kp = L × ωc
 * - Integral gain: Ki = R × ωc
 * - Anti-windup gain: Kb ≈ ωc/10 (empirical)
 */
static void calculate_pi_gains(real32_T Rs, real32_T Ls, real32_T bandwidth_hz)
{
    real32_T bandwidth_rad_s;
    real32_T effective_bandwidth;
    
    /* Convert bandwidth to rad/s */
    bandwidth_rad_s = TWO_PI * bandwidth_hz;
    
    /* Apply safety margin */
    effective_bandwidth = bandwidth_rad_s * PID_Autotune.safety_margin;
    
    /* Clamp bandwidth to safe range */
    real32_T min_bw_rad = TWO_PI * PID_Autotune.min_bandwidth_hz;
    real32_T max_bw_rad = TWO_PI * PID_Autotune.max_bandwidth_hz;
    
    if (effective_bandwidth < min_bw_rad) {
        effective_bandwidth = min_bw_rad;
    } else if (effective_bandwidth > max_bw_rad) {
        effective_bandwidth = max_bw_rad;
    }
    
    /* Calculate gains */
    PID_Autotune.Kp_calculated = Ls * effective_bandwidth;
    PID_Autotune.Ki_calculated = Rs * effective_bandwidth;
    PID_Autotune.Kb_calculated = effective_bandwidth / 10.0f;  /* Empirical value */
    
    /* Sanity check on calculated gains */
    if (PID_Autotune.Kp_calculated < 0.1f) {
        PID_Autotune.Kp_calculated = 0.1f;
    }
    if (PID_Autotune.Ki_calculated < 1.0f) {
        PID_Autotune.Ki_calculated = 1.0f;
    }
    if (PID_Autotune.Kb_calculated < 1.0f) {
        PID_Autotune.Kb_calculated = 1.0f;
    }
}

/**
 * @brief Get system time in milliseconds
 * 

 * NOTE: This is a placeholder. For production, replace with HAL_GetTick() or actual timer.
 *

 * NOTE: This is a placeholder. For production, replace with HAL_GetTick() or actual timer.
 *

 * NOTE: This is a placeholder. For production, replace with HAL_GetTick() or actual timer.
 *
static uint32_t get_system_time_ms(void)
{
    /* TODO: Replace with actual system timer */
    /* For now, use a static counter incremented at control frequency */
    static uint32_t time_counter_ms = 0;
    time_counter_ms++;
    return time_counter_ms / 10;  /* 10 kHz to 1 kHz conversion */
}
