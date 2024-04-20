/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.c
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

#include "crazyflie.h"

/* Output and update for referenced model: 'crazyflie' */
void crazyflie(const real_T *rtu_Ref_Roll, const real_T *rtu_Ref_Pitch, const
               real_T *rtu_Acc_x, const real_T *rtu_Acc_y, const real_T
               *rtu_Acc_z, const real_T *rtu_Gyro_x, const real_T *rtu_Gyro_y,
               uint16_T *rty_Motor_1, uint16_T *rty_Motor_2, uint16_T
               *rty_Motor_4, DW_crazyflie_f_T *localDW)
{
  real_T rtb_Gain_o_idx_0;
  real_T rtb_Gain_o_idx_1;
  real_T rtb_Sum1_i;
  real_T rtb_Sum1_idx_1;
  real_T rtb_Sum7;
  real_T rtb_phi;
  real_T tmp;

  /* MATLAB Function: '<S3>/MATLAB Function' */
  /* MATLAB Function 'Subsystem1/Subsystem/MATLAB Function': '<S4>:1' */
  /* '<S4>:1:3' phi= atan2(Fy_B,Fz_b); */
  rtb_Sum7 = *rtu_Acc_y;
  rtb_Sum1_i = *rtu_Acc_z;

  /* MATLAB Function: '<S3>/MATLAB Function1' */
  /*  fx or -fx ???? */
  /* MATLAB Function 'Subsystem1/Subsystem/MATLAB Function1': '<S5>:1' */
  /* '<S5>:1:3' theta = atan2(-Fx_B,sqrt((Fy_B)^2+(Fz_B)^2)); */
  rtb_Sum1_idx_1 = -*rtu_Acc_x;
  rtb_phi = sqrt(*rtu_Acc_y * *rtu_Acc_y + *rtu_Acc_z * *rtu_Acc_z);

  /* Gain: '<S2>/Gain' */
  rtb_Gain_o_idx_0 = 0.1 * *rtu_Gyro_x;
  rtb_Gain_o_idx_1 = 0.1 * *rtu_Gyro_y;

  /* Sum: '<S2>/Sum1' incorporates:
   *  Delay: '<S2>/Delay'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S2>/Gain2'
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  MATLAB Function: '<S3>/MATLAB Function'
   *  MATLAB Function: '<S3>/MATLAB Function1'
   *  Sum: '<S2>/Sum'
   */
  rtb_Gain_o_idx_0 = 57.295779513082323 * atan2(rtb_Sum7, rtb_Sum1_i) * 0.2 +
    (rtb_Gain_o_idx_0 + localDW->Delay_DSTATE[0]) * 0.8;
  rtb_Sum1_idx_1 = 57.295779513082323 * atan2(rtb_Sum1_idx_1, rtb_phi) * 0.2 +
    (rtb_Gain_o_idx_1 + localDW->Delay_DSTATE[1]) * 0.8;

  /* Sum: '<Root>/Sum' */
  rtb_phi = *rtu_Ref_Roll - rtb_Gain_o_idx_0;

  /* Sum: '<S1>/Sum6' */
  rtb_Gain_o_idx_1 = *rtu_Ref_Pitch - rtb_Sum1_idx_1;

  /* Sum: '<S1>/Sum7' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   */
  rtb_Sum7 = rtb_Gain_o_idx_1 + localDW->DiscreteTimeIntegrator1_DSTATE;

  /* Sum: '<Root>/Sum1' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Gain: '<Root>/Gain'
   */
  rtb_Sum1_i = 1000.0 * rtb_phi + localDW->DiscreteTimeIntegrator_DSTATE;

  /* DataTypeConversion: '<Root>/ToUint16' incorporates:
   *  Sum: '<Root>/Sum6'
   */
  tmp = rtb_Sum1_i + rtb_Sum7;
  if (tmp < 65536.0) {
    if (tmp >= 0.0) {
      *rty_Motor_1 = (uint16_T)tmp;
    } else {
      *rty_Motor_1 = 0U;
    }
  } else {
    *rty_Motor_1 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16' */

  /* DataTypeConversion: '<Root>/ToUint16_1' */
  if (rtb_Sum1_i < 65536.0) {
    if (rtb_Sum1_i >= 0.0) {
      *rty_Motor_2 = (uint16_T)rtb_Sum1_i;
    } else {
      *rty_Motor_2 = 0U;
    }
  } else {
    *rty_Motor_2 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_1' */

  /* DataTypeConversion: '<Root>/ToUint16_3' */
  if (rtb_Sum7 < 65536.0) {
    if (rtb_Sum7 >= 0.0) {
      *rty_Motor_4 = (uint16_T)rtb_Sum7;
    } else {
      *rty_Motor_4 = 0U;
    }
  } else {
    *rty_Motor_4 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_3' */

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_DSTATE += 0.001 * rtb_phi;

  /* Update for Delay: '<S2>/Delay' */
  localDW->Delay_DSTATE[0] = rtb_Gain_o_idx_0;
  localDW->Delay_DSTATE[1] = rtb_Sum1_idx_1;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  localDW->DiscreteTimeIntegrator1_DSTATE += 0.001 * rtb_Gain_o_idx_1;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
