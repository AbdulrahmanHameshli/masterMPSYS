/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflieModel.h
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

#ifndef RTW_HEADER_crazyflieModel_h_
#define RTW_HEADER_crazyflieModel_h_
#include <stdbool.h>
#include <stdint.h>
#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)

void crazyflieModelInit(StateEstimatorType estimator);
bool crazyflieModelTest(void);

#endif                                 /* RTW_HEADER_crazyflieModel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
