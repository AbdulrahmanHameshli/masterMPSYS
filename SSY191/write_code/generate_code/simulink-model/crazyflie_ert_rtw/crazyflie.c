/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.c
 *
 * Code generated for Simulink model 'crazyflie'.
 *
 * Model version                  : 5.12
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Fri Apr  5 15:02:15 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "crazyflie.h"

/* Block signals and states (default storage) */
DW_crazyflie_T crazyflie_DW;

/* External inputs (root inport signals with default storage) */
ExtU_crazyflie_T crazyflie_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_crazyflie_T crazyflie_Y;

/* Model step function */
void crazyflie_step(void)
{
  real_T rtb_Sum1_idx_0;
  real_T rtb_Sum1_idx_1;
  real_T rtb_Sum5;
  real_T rtb_Sum6;
  real_T rtb_Sum7;
  real_T rtb_Sum_k;
  real_T tmp;

  /* Sum: '<S2>/Sum1' incorporates:
   *  Delay: '<S2>/Delay'
   *  Gain: '<S2>/Gain'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S2>/Gain2'
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  Inport: '<Root>/Acc_x'
   *  Inport: '<Root>/Acc_y'
   *  Inport: '<Root>/Acc_z'
   *  Inport: '<Root>/Gyro_x'
   *  Inport: '<Root>/Gyro_y'
   *  MATLAB Function: '<S3>/MATLAB Function'
   *  MATLAB Function: '<S3>/MATLAB Function1'
   *  Sum: '<S2>/Sum'
   */
  /* MATLAB Function 'Subsystem1/Subsystem/MATLAB Function': '<S4>:1' */
  /* '<S4>:1:3' phi= atan2(Fy_B,Fz_b); */
  /*  fx or -fx ???? */
  /* MATLAB Function 'Subsystem1/Subsystem/MATLAB Function1': '<S5>:1' */
  /* '<S5>:1:3' theta = atan2(-Fx_B,sqrt((Fy_B)^2+(Fz_B)^2)); */
  rtb_Sum1_idx_0 = (0.1 * crazyflie_U.Gyro_x + crazyflie_DW.Delay_DSTATE[0]) *
    0.8 + 57.295779513082323 * atan2(crazyflie_U.Acc_y, crazyflie_U.Acc_z) * 0.2;
  rtb_Sum1_idx_1 = atan2(-crazyflie_U.Acc_x, sqrt(crazyflie_U.Acc_y *
    crazyflie_U.Acc_y + crazyflie_U.Acc_z * crazyflie_U.Acc_z)) *
    57.295779513082323 * 0.2 + (0.1 * crazyflie_U.Gyro_y +
    crazyflie_DW.Delay_DSTATE[1]) * 0.8;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/Ref_Roll'
   */
  rtb_Sum_k = crazyflie_U.Ref_Roll - rtb_Sum1_idx_0;

  /* Sum: '<Root>/Sum1' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Gain: '<Root>/Gain'
   */
  rtb_Sum5 = 1000.0 * rtb_Sum_k + crazyflie_DW.DiscreteTimeIntegrator_DSTATE;

  /* Sum: '<S1>/Sum6' incorporates:
   *  Inport: '<Root>/Ref_Pitch'
   */
  rtb_Sum6 = crazyflie_U.Ref_Pitch - rtb_Sum1_idx_1;

  /* Sum: '<S1>/Sum7' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   *  Gain: '<S1>/Gain6'
   */
  rtb_Sum7 = 1000.0 * rtb_Sum6 + crazyflie_DW.DiscreteTimeIntegrator1_DSTATE;

  /* DataTypeConversion: '<Root>/ToUint16' incorporates:
   *  Gain: '<Root>/Gain3'
   *  Gain: '<Root>/Gain5'
   *  Sum: '<Root>/Sum2'
   */
  tmp = -rtb_Sum5 + -rtb_Sum7;
  if (tmp < 65536.0) {
    if (tmp >= 0.0) {
      /* Outport: '<Root>/Motor_1' */
      crazyflie_Y.Motor_1 = (uint16_T)tmp;
    } else {
      /* Outport: '<Root>/Motor_1' */
      crazyflie_Y.Motor_1 = 0U;
    }
  } else {
    /* Outport: '<Root>/Motor_1' */
    crazyflie_Y.Motor_1 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16' */

  /* DataTypeConversion: '<Root>/ToUint16_1' incorporates:
   *  Gain: '<Root>/Gain1'
   *  Sum: '<Root>/Sum3'
   */
  tmp = -rtb_Sum5 + rtb_Sum7;
  if (tmp < 65536.0) {
    if (tmp >= 0.0) {
      /* Outport: '<Root>/Motor_2' */
      crazyflie_Y.Motor_2 = (uint16_T)tmp;
    } else {
      /* Outport: '<Root>/Motor_2' */
      crazyflie_Y.Motor_2 = 0U;
    }
  } else {
    /* Outport: '<Root>/Motor_2' */
    crazyflie_Y.Motor_2 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_1' */

  /* DataTypeConversion: '<Root>/ToUint16_2' incorporates:
   *  Sum: '<Root>/Sum4'
   */
  tmp = rtb_Sum5 + rtb_Sum7;
  if (tmp < 65536.0) {
    if (tmp >= 0.0) {
      /* Outport: '<Root>/Motor_3' */
      crazyflie_Y.Motor_3 = (uint16_T)tmp;
    } else {
      /* Outport: '<Root>/Motor_3' */
      crazyflie_Y.Motor_3 = 0U;
    }
  } else {
    /* Outport: '<Root>/Motor_3' */
    crazyflie_Y.Motor_3 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_2' */

  /* DataTypeConversion: '<Root>/ToUint16_3' incorporates:
   *  Gain: '<Root>/Gain6'
   *  Sum: '<Root>/Sum5'
   */
  tmp = rtb_Sum5 + -rtb_Sum7;
  if (tmp < 65536.0) {
    if (tmp >= 0.0) {
      /* Outport: '<Root>/Motor_4' */
      crazyflie_Y.Motor_4 = (uint16_T)tmp;
    } else {
      /* Outport: '<Root>/Motor_4' */
      crazyflie_Y.Motor_4 = 0U;
    }
  } else {
    /* Outport: '<Root>/Motor_4' */
    crazyflie_Y.Motor_4 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_3' */

  /* Update for Delay: '<S2>/Delay' */
  crazyflie_DW.Delay_DSTATE[0] = rtb_Sum1_idx_0;
  crazyflie_DW.Delay_DSTATE[1] = rtb_Sum1_idx_1;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  crazyflie_DW.DiscreteTimeIntegrator_DSTATE += 0.001 * rtb_Sum_k;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  crazyflie_DW.DiscreteTimeIntegrator1_DSTATE += 0.001 * rtb_Sum6;
}

/* Model initialize function */
void crazyflie_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void crazyflie_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
