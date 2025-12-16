/**********************************
 * PID Auto-Tuning Module
 * Automatic tuning of current loop PI controllers based on identified motor parameters
 * References: TI InstaSPIN, SimpleFOC, model-based tuning
 **********************************/
#ifndef __PID_AUTOTUNE_H__
#define __PID_AUTOTUNE_H__

#include <stdint.h>
#include "rtwtypes.h"

/**
 * Auto-tune state machine states
 */
typedef enum {
	AUTOTUNE_IDLE = 0,              /* Not running */
	AUTOTUNE_WAIT_STABLE,           /* Waiting for motor to reach stable state */
	AUTOTUNE_IDENTIFY_PARAMS,       /* Waiting for parameter identification convergence */
	AUTOTUNE_CALCULATE_GAINS,       /* Calculate new PI gains */
	AUTOTUNE_APPLY_GAINS,           /* Apply and verify new gains */
	AUTOTUNE_COMPLETE,              /* Auto-tune finished successfully */
	AUTOTUNE_FAILED                 /* Auto-tune failed */
} AUTOTUNE_STATE;

/**
 * Auto-tune configuration structure
 */
typedef struct {
	/* Configuration parameters */
	real32_T target_bandwidth_hz;       /* Desired current loop bandwidth (Hz) */
	real32_T min_bandwidth_hz;          /* Minimum safe bandwidth (Hz) */
	real32_T max_bandwidth_hz;          /* Maximum safe bandwidth (Hz) */
	real32_T safety_margin;             /* Safety margin factor (0.5-0.9) */
	real32_T convergence_threshold;     /* Parameter convergence threshold */
	uint32_t stable_time_ms;            /* Time to wait for stability (ms) */
	uint32_t max_tune_time_ms;          /* Maximum tuning time (ms) */
	
	/* State variables */
	AUTOTUNE_STATE state;               /* Current state */
	uint32_t state_entry_time;          /* Time when state was entered */
	uint32_t tune_start_time;           /* Time when tuning started */
	
	/* Identified parameters */
	real32_T Rs_identified;             /* Identified stator resistance (Ohm) */
	real32_T Ls_identified;             /* Identified stator inductance (H) */
	real32_T Rs_prev;                   /* Previous R for convergence check */
	real32_T Ls_prev;                   /* Previous L for convergence check */
	
	/* Calculated gains */
	real32_T Kp_calculated;             /* Calculated proportional gain */
	real32_T Ki_calculated;             /* Calculated integral gain */
	real32_T Kb_calculated;             /* Calculated anti-windup gain */
	
	/* Original gains (backup) */
	real32_T Kp_original;               /* Original Kp before tuning */
	real32_T Ki_original;               /* Original Ki before tuning */
	real32_T Kb_original;               /* Original Kb before tuning */
	
	/* Status flags */
	uint8_t enabled;                    /* Auto-tune enabled flag */
	uint8_t tune_triggered;             /* Tune request flag */
	uint8_t params_valid;               /* Identified parameters valid */
	uint8_t gains_applied;              /* New gains applied flag */
} PID_AUTOTUNE_DEF;

/**
 * Global auto-tune instance
 */
extern PID_AUTOTUNE_DEF PID_Autotune;

/**
 * @brief Initialize auto-tuning module
 * 
 * Sets up auto-tune configuration with default parameters.
 * Should be called once during system initialization.
 */
void pid_autotune_init(void);

/**
 * @brief Start auto-tuning sequence
 * 
 * Initiates the auto-tuning process. Should only be called when:
 * - Motor is running at low speed (<100 rad/s)
 * - No load or light load condition
 * - System is in steady state
 * 
 * @return 1 if started successfully, 0 if conditions not met
 */
uint8_t pid_autotune_start(void);

/**
 * @brief Stop auto-tuning and restore original gains
 * 
 * Aborts the auto-tuning process and restores the original PI gains.
 */
void pid_autotune_stop(void);

/**
 * @brief Execute one auto-tune cycle (call at control frequency)
 * 
 * State machine for auto-tuning process. Should be called every control cycle
 * (typically 10 kHz) when auto-tune is active.
 * 
 * Sequence:
 * 1. Wait for motor to stabilize
 * 2. Monitor parameter identification convergence
 * 3. Calculate new PI gains using model-based formulas
 * 4. Apply gains and verify stability
 * 
 * @param Rs_current Current identified resistance (Ohm)
 * @param Ls_current Current identified inductance (H)
 * @param Kp_current Pointer to current D/Q-axis Kp gain
 * @param Ki_current Pointer to current D/Q-axis Ki gain
 * @param Kb_current Pointer to current D/Q-axis Kb gain
 */
void pid_autotune_step(real32_T Rs_current, real32_T Ls_current,
					   real32_T* Kp_current, real32_T* Ki_current, real32_T* Kb_current);

/**
 * @brief Get auto-tune status
 * 
 * @return Current auto-tune state
 */
AUTOTUNE_STATE pid_autotune_get_state(void);

/**
 * @brief Check if auto-tune is complete
 * 
 * @return 1 if complete (success or failure), 0 if still running
 */
uint8_t pid_autotune_is_complete(void);

/**
 * @brief Get calculated gains
 * 
 * Retrieves the calculated PI gains after successful auto-tune.
 * 
 * @param Kp Output: Proportional gain
 * @param Ki Output: Integral gain
 * @param Kb Output: Anti-windup gain
 * @return 1 if gains valid, 0 otherwise
 */
uint8_t pid_autotune_get_gains(real32_T* Kp, real32_T* Ki, real32_T* Kb);

#endif /* __PID_AUTOTUNE_H__ */
