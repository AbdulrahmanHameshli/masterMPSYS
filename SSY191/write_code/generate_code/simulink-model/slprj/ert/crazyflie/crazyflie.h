/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.h
 *
 * Code generated for Simulink model 'crazyflie'.
 *
 * Model version                  : 5.9
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Fri Apr  5 14:23:45 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_crazyflie_h_
#define RTW_HEADER_crazyflie_h_
#include <math.h>
#ifndef crazyflie_COMMON_INCLUDES_
#define crazyflie_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* crazyflie_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Block signals and states (default storage) for model 'crazyflie' */
typedef struct {
  real_T Delay_DSTATE[2];              /* '<S2>/Delay' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<Root>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S1>/Discrete-Time Integrator1' */
} DW_crazyflie_f_T;

typedef struct {
  DW_crazyflie_f_T rtdw;
} MdlrefDW_crazyflie_T;

extern void crazyflie(const real_T *rtu_Ref_Roll, const real_T *rtu_Ref_Pitch,
                      const real_T *rtu_Acc_x, const real_T *rtu_Acc_y, const
                      real_T *rtu_Acc_z, const real_T *rtu_Gyro_x, const real_T *
                      rtu_Gyro_y, uint16_T *rty_Motor_1, uint16_T *rty_Motor_2,
                      uint16_T *rty_Motor_4, DW_crazyflie_f_T *localDW);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S2>/Scope' : Unused code path elimination
 * Block '<S2>/Scope1' : Unused code path elimination
 * Block '<S2>/Scope2' : Unused code path elimination
 * Block '<S3>/Scope' : Unused code path elimination
 * Block '<S1>/Gain6' : Eliminated nontunable gain of 1
 */

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
 * '<Root>' : 'crazyflie'
 * '<S1>'   : 'crazyflie/Subsystem'
 * '<S2>'   : 'crazyflie/Subsystem1'
 * '<S3>'   : 'crazyflie/Subsystem1/Subsystem'
 * '<S4>'   : 'crazyflie/Subsystem1/Subsystem/MATLAB Function'
 * '<S5>'   : 'crazyflie/Subsystem1/Subsystem/MATLAB Function1'
 */
#endif                                 /* RTW_HEADER_crazyflie_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
