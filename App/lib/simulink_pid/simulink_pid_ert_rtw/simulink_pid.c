/*
 * File: simulink_pid.c
 *
 * Code generated for Simulink model 'simulink_pid'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Fri Jul 24 18:14:44 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink_pid.h"
#include <math.h>
#include "rtwtypes.h"
#include "simulink_pid_private.h"

/* Exported block signals */
real_T pwm_value;                      /* '<Root>/pwm_value' */

/* Block signals (default storage) */
B_simulink_pid_T simulink_pid_B;

/* Continuous states */
X_simulink_pid_T simulink_pid_X;

/* Disabled State Vector */
XDis_simulink_pid_T simulink_pid_XDis;

/* External inputs (root inport signals with default storage) */
ExtU_simulink_pid_T simulink_pid_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_simulink_pid_T simulink_pid_Y;

/* Real-time model */
static RT_MODEL_simulink_pid_T simulink_pid_M_;
RT_MODEL_simulink_pid_T *const simulink_pid_M = &simulink_pid_M_;

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  simulink_pid_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  simulink_pid_step();
  simulink_pid_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  simulink_pid_step();
  simulink_pid_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void simulink_pid_step(void)
{
  real_T rtb_Add;
  if (rtmIsMajorTimeStep(simulink_pid_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&simulink_pid_M->solverInfo,
                          ((simulink_pid_M->Timing.clockTick0+1)*
      simulink_pid_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(simulink_pid_M)) {
    simulink_pid_M->Timing.t[0] = rtsiGetT(&simulink_pid_M->solverInfo);
  }

  /* Clock: '<S1>/Clock1' */
  rtb_Add = simulink_pid_M->Timing.t[0];

  /* Outport: '<Root>/pwm_value' incorporates:
   *  Constant: '<S1>/initialFreq'
   *  Gain: '<Root>/Gain'
   *  Product: '<S1>/Product1'
   *  Product: '<S1>/Product2'
   *  Sum: '<S1>/Sum'
   *  Trigonometry: '<S1>/Output'
   */
  pwm_value = sin((rtb_Add * simulink_pid_ConstB.Gain + 0.62831853071795862) *
                  rtb_Add) * 1000.0;

  /* Gain: '<S39>/Filter Coefficient' incorporates:
   *  Constant: '<Root>/Constant'
   *  Gain: '<S29>/Derivative Gain'
   *  Inport: '<Root>/rpm_now'
   *  Integrator: '<S31>/Filter'
   *  Sum: '<Root>/Add'
   *  Sum: '<S31>/SumD'
   */
  simulink_pid_B.FilterCoefficient = ((700.0 - simulink_pid_U.rpm_now) *
    -0.00485109028407955 - simulink_pid_X.Filter_CSTATE) * 61.765222360892;

  /* Outport: '<Root>/pwm_ctrl' incorporates:
   *  Constant: '<Root>/Constant'
   *  Gain: '<S41>/Proportional Gain'
   *  Inport: '<Root>/rpm_now'
   *  Integrator: '<S36>/Integrator'
   *  Sum: '<Root>/Add'
   *  Sum: '<S45>/Sum'
   */
  simulink_pid_Y.pwm_ctrl = ((700.0 - simulink_pid_U.rpm_now) *
    -1.76391701674895 + simulink_pid_X.Integrator_CSTATE) +
    simulink_pid_B.FilterCoefficient;

  /* Gain: '<S33>/Integral Gain' incorporates:
   *  Constant: '<Root>/Constant'
   *  Inport: '<Root>/rpm_now'
   *  Sum: '<Root>/Add'
   */
  simulink_pid_B.IntegralGain = (700.0 - simulink_pid_U.rpm_now) *
    -48.3950240418852;
  if (rtmIsMajorTimeStep(simulink_pid_M)) {
    rt_ertODEUpdateContinuousStates(&simulink_pid_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++simulink_pid_M->Timing.clockTick0;
    simulink_pid_M->Timing.t[0] = rtsiGetSolverStopTime
      (&simulink_pid_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      simulink_pid_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void simulink_pid_derivatives(void)
{
  XDot_simulink_pid_T *_rtXdot;
  _rtXdot = ((XDot_simulink_pid_T *) simulink_pid_M->derivs);

  /* Derivatives for Integrator: '<S36>/Integrator' */
  _rtXdot->Integrator_CSTATE = simulink_pid_B.IntegralGain;

  /* Derivatives for Integrator: '<S31>/Filter' */
  _rtXdot->Filter_CSTATE = simulink_pid_B.FilterCoefficient;

  /* Derivatives for TransferFcn: '<Root>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE[0] = -22.972259894849586 *
    simulink_pid_X.TransferFcn_CSTATE[0];
  _rtXdot->TransferFcn_CSTATE[0] += -0.182730441441999 *
    simulink_pid_X.TransferFcn_CSTATE[1];
  _rtXdot->TransferFcn_CSTATE[1] = simulink_pid_X.TransferFcn_CSTATE[0];
}

/* Model initialize function */
void simulink_pid_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&simulink_pid_M->solverInfo,
                          &simulink_pid_M->Timing.simTimeStep);
    rtsiSetTPtr(&simulink_pid_M->solverInfo, &rtmGetTPtr(simulink_pid_M));
    rtsiSetStepSizePtr(&simulink_pid_M->solverInfo,
                       &simulink_pid_M->Timing.stepSize0);
    rtsiSetdXPtr(&simulink_pid_M->solverInfo, &simulink_pid_M->derivs);
    rtsiSetContStatesPtr(&simulink_pid_M->solverInfo, (real_T **)
                         &simulink_pid_M->contStates);
    rtsiSetNumContStatesPtr(&simulink_pid_M->solverInfo,
      &simulink_pid_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&simulink_pid_M->solverInfo,
      &simulink_pid_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&simulink_pid_M->solverInfo,
      &simulink_pid_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&simulink_pid_M->solverInfo,
      &simulink_pid_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&simulink_pid_M->solverInfo, (boolean_T**)
      &simulink_pid_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&simulink_pid_M->solverInfo, (&rtmGetErrorStatus
      (simulink_pid_M)));
    rtsiSetRTModelPtr(&simulink_pid_M->solverInfo, simulink_pid_M);
  }

  rtsiSetSimTimeStep(&simulink_pid_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&simulink_pid_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&simulink_pid_M->solverInfo, false);
  simulink_pid_M->intgData.y = simulink_pid_M->odeY;
  simulink_pid_M->intgData.f[0] = simulink_pid_M->odeF[0];
  simulink_pid_M->intgData.f[1] = simulink_pid_M->odeF[1];
  simulink_pid_M->intgData.f[2] = simulink_pid_M->odeF[2];
  simulink_pid_M->contStates = ((X_simulink_pid_T *) &simulink_pid_X);
  simulink_pid_M->contStateDisabled = ((XDis_simulink_pid_T *)
    &simulink_pid_XDis);
  simulink_pid_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&simulink_pid_M->solverInfo, (void *)
                    &simulink_pid_M->intgData);
  rtsiSetSolverName(&simulink_pid_M->solverInfo,"ode3");
  rtmSetTPtr(simulink_pid_M, &simulink_pid_M->Timing.tArray[0]);
  simulink_pid_M->Timing.stepSize0 = 0.001;

  /* InitializeConditions for Integrator: '<S36>/Integrator' */
  simulink_pid_X.Integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S31>/Filter' */
  simulink_pid_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<Root>/Transfer Fcn' */
  simulink_pid_X.TransferFcn_CSTATE[0] = 0.0;
  simulink_pid_X.TransferFcn_CSTATE[1] = 0.0;
}

/* Model terminate function */
void simulink_pid_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
