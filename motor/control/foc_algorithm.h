
#ifndef RTW_HEADER_foc_algorithm_h_
#define RTW_HEADER_foc_algorithm_h_
#include <stddef.h>
#ifndef foc_algorithm_COMMON_INCLUDES_
# define foc_algorithm_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                               

#include "MW_target_hardware_resources.h"
#include "mw_cmsis.h"


#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define foc_algorithm_M                (rtM)


typedef struct tag_RTM RT_MODEL;


typedef struct {
  real_T EKF_States[4];   
  real_T L_Ident_States;     
  real_T R_flux_Ident_States;    
  real32_T EKF_Interface[7];
  real32_T R_flux_Ident_Interface[3];
  real32_T L_Ident_Interface[2];
  real32_T R_flux_Ident_Output[2];       
  real32_T L_Ident_Output;          
} FOC_INTERFACE_STATES_DEF;


typedef struct {
  real32_T Id_ref;                     
  real32_T Iq_ref;                     
  real32_T speed_fdk;                  
  real32_T theta;                      
  real32_T ia;                         
  real32_T ib;                         
  real32_T ic;                         
  real32_T Udc;                        
  real32_T Tpwm;                       
  real32_T Rs;                         
  real32_T Ls;                         
  real32_T flux;                       
} FOC_INPUT_DEF;


typedef struct {
  real32_T Tcmp1;                      
  real32_T Tcmp2;                      
  real32_T Tcmp3;                      
  real32_T EKF[4];                     
  real32_T L_RF[3];                    
} FOC_OUTPUT_DEF;


typedef struct
{
  real32_T Ia;
  real32_T Ib;
  real32_T Ic;
}CURRENT_ABC_DEF;

typedef struct
{
  real32_T Ialpha;
  real32_T Ibeta;
}CURRENT_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Valpha;
  real32_T Vbeta;
}VOLTAGE_ALPHA_BETA_DEF;

typedef struct
{
  real32_T Cos;
  real32_T Sin;
}TRANSF_COS_SIN_DEF;

typedef struct
{
  real32_T Id;
  real32_T Iq;
}CURRENT_DQ_DEF;

typedef struct
{
  real32_T Vd;
  real32_T Vq;
}VOLTAGE_DQ_DEF;

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;
}CURRENT_PID_DEF;

extern CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;

struct tag_RTM {
  const char_T *errorStatus;
};

extern FOC_INTERFACE_STATES_DEF FOC_Interface_states;

extern FOC_INPUT_DEF FOC_Input;

extern FOC_OUTPUT_DEF FOC_Output;

/**
 * @brief Initialize FOC algorithm and all sub-components
 * 
 * Initializes:
 * - Current loop PI controllers (D-axis and Q-axis)
 * - Speed loop PI controller
 * - Extended Kalman Filter for sensorless position estimation
 * - Motor parameter identification algorithms
 * 
 * Must be called once before motor operation starts.
 */
extern void foc_algorithm_initialize(void);

/**
 * @brief Execute one complete FOC control cycle
 * 
 * Performs all FOC calculations in sequence:
 * - Clarke transform (abc -> alpha-beta frame)
 * - Park transform (alpha-beta -> dq frame)
 * - Current PI control in dq frame
 * - Inverse Park transform (dq -> alpha-beta frame)
 * - Space Vector PWM calculation
 * - EKF state observer update
 * - Motor parameter estimation
 * - PID auto-tuning (if enabled)
 * 
 * Should be called at PWM frequency (typically 10-20 kHz).
 */
extern void foc_algorithm_step(void);

#ifdef ENABLE_PID_AUTOTUNE
/**
 * @brief Start PID auto-tuning for current loop controllers
 * 
 * Initiates automatic tuning of Id and Iq PI controller gains.
 * Should be called when motor is running at low speed with no/light load.
 * 
 * @return 1 if started successfully, 0 if conditions not met
 */
extern uint8_t foc_start_pid_autotune(void);

/**
 * @brief Stop PID auto-tuning
 * 
 * Aborts the auto-tuning process and restores original gains.
 */
extern void foc_stop_pid_autotune(void);

/**
 * @brief Get PID auto-tune status
 * 
 * @return Auto-tune state (0=idle, 1=running, 2=complete, 3=failed)
 */
extern uint8_t foc_get_autotune_status(void);
#endif




extern real32_T D_PI_I;
extern real32_T D_PI_KB;
extern real32_T D_PI_LOW_LIMIT;
extern real32_T D_PI_P;
extern real32_T D_PI_UP_LIMIT;
extern real32_T Q_PI_I;
extern real32_T Q_PI_KB;
extern real32_T Q_PI_LOW_LIMIT;
extern real32_T Q_PI_P;
extern real32_T Q_PI_UP_LIMIT;


extern RT_MODEL *const rtM;


#endif                                

