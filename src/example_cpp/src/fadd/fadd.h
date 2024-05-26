/*
 * File: fadd.h
 *
 * Code generated for Simulink model 'fadd'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sat May 25 17:02:42 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_fadd_h_
#define RTW_HEADER_fadd_h_
#ifndef fadd_COMMON_INCLUDES_
#define fadd_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* fadd_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_fadd_T RT_MODEL_fadd_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T x;                            /* '<Root>/x' */
  real_T y;                            /* '<Root>/y' */
} ExtU_fadd_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T z;                            /* '<Root>/z' */
} ExtY_fadd_T;

/* Real-time Model Data Structure */
struct tag_RTM_fadd_T {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_fadd_T fadd_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_fadd_T fadd_Y;

/* Model entry point functions */
extern void fadd_initialize(void);
extern void fadd_step(void);
extern void fadd_terminate(void);

/* Real-time Model object */
extern RT_MODEL_fadd_T *const fadd_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fadd'
 * '<S1>'   : 'fadd/Subsystem'
 */
#endif                                 /* RTW_HEADER_fadd_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
