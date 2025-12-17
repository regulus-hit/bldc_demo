#ifndef __PUBLIC_H__
#define __PUBLIC_H__

#define MATH_PI 		3.1415926536F
#define MATH_2PI 		6.2831853072F
#define MATH_sqrt_3		1.7320508075F
#define MATH_cos_30		0.8660254038F
//#define MATH_cos_30	0.866025388F
#define MATH_cos_60		0.500F

//#define COPILOT_BUGFIX_PI
#undef COPILOT_BUGFIX_PI

#define COPILOT_BUGFIX_VECTOR_SCALE
//#undef COPILOT_BUGFIX_VECTOR_SCALE

#define ADC_OFFSET_CHECK
//#undef ADC_OFFSET_CHECK

/*******************************************************************************
 * Speed Controller Selection
 * Select ONE of the following speed controllers:
 * - USE_SPEED_PID: Traditional PID controller (default, proven)
 * - USE_SPEED_ADRC: Linear Active Disturbance Rejection Control (advanced)
 * 
 * ADRC provides better disturbance rejection and faster response but requires
 * more computational resources. PID is simpler and well-tested.
 ******************************************************************************/
#define USE_SPEED_PID          // Traditional PID speed controller
//#define USE_SPEED_ADRC         // Linear ADRC speed controller

/* PI macro for compatibility with existing code */
#ifndef PI
#define PI              MATH_PI
#endif

#endif
