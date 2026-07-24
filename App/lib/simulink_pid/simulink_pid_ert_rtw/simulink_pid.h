/*
 * File: simulink_pid.h
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

#ifndef simulink_pid_h_
#define simulink_pid_h_
#ifndef simulink_pid_COMMON_INCLUDES_
#define simulink_pid_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* simulink_pid_COMMON_INCLUDES_ */

#include "simulink_pid_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T FilterCoefficient;            /* '<S39>/Filter Coefficient' */
  real_T IntegralGain;                 /* '<S33>/Integral Gain' */
} B_simulink_pid_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S36>/Integrator' */
  real_T Filter_CSTATE;                /* '<S31>/Filter' */
  real_T TransferFcn_CSTATE[2];        /* '<Root>/Transfer Fcn' */
} X_simulink_pid_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S36>/Integrator' */
  real_T Filter_CSTATE;                /* '<S31>/Filter' */
  real_T TransferFcn_CSTATE[2];        /* '<Root>/Transfer Fcn' */
} XDot_simulink_pid_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S36>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S31>/Filter' */
  boolean_T TransferFcn_CSTATE[2];     /* '<Root>/Transfer Fcn' */
} XDis_simulink_pid_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Product;                /* '<S1>/Product' */
  const real_T Gain;                   /* '<S1>/Gain' */
} ConstB_simulink_pid_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T rpm_now;                      /* '<Root>/rpm_now' */
} ExtU_simulink_pid_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T pwm_ctrl;                     /* '<Root>/pwm_ctrl' */
} ExtY_simulink_pid_T;

/* Real-time Model Data Structure */
struct tag_RTM_simulink_pid_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_simulink_pid_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_simulink_pid_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[3][4];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_simulink_pid_T simulink_pid_B;

/* Continuous states (default storage) */
extern X_simulink_pid_T simulink_pid_X;

/* Disabled states (default storage) */
extern XDis_simulink_pid_T simulink_pid_XDis;

/* External inputs (root inport signals with default storage) */
extern ExtU_simulink_pid_T simulink_pid_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_simulink_pid_T simulink_pid_Y;
extern const ConstB_simulink_pid_T simulink_pid_ConstB;/* constant block i/o */

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T pwm_value;               /* '<Root>/pwm_value' */

/* Model entry point functions */
extern void simulink_pid_initialize(void);
extern void simulink_pid_step(void);
extern void simulink_pid_terminate(void);

/* Real-time Model object */
extern RT_MODEL_simulink_pid_T *const simulink_pid_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
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
 * '<Root>' : 'simulink_pid'
 * '<S1>'   : 'simulink_pid/Chirp Signal'
 * '<S2>'   : 'simulink_pid/PID Controller'
 * '<S3>'   : 'simulink_pid/PID Controller/Anti-windup'
 * '<S4>'   : 'simulink_pid/PID Controller/D Gain'
 * '<S5>'   : 'simulink_pid/PID Controller/External Derivative'
 * '<S6>'   : 'simulink_pid/PID Controller/Filter'
 * '<S7>'   : 'simulink_pid/PID Controller/Filter ICs'
 * '<S8>'   : 'simulink_pid/PID Controller/I Gain'
 * '<S9>'   : 'simulink_pid/PID Controller/Ideal P Gain'
 * '<S10>'  : 'simulink_pid/PID Controller/Ideal P Gain Fdbk'
 * '<S11>'  : 'simulink_pid/PID Controller/Integrator'
 * '<S12>'  : 'simulink_pid/PID Controller/Integrator ICs'
 * '<S13>'  : 'simulink_pid/PID Controller/N Copy'
 * '<S14>'  : 'simulink_pid/PID Controller/N Gain'
 * '<S15>'  : 'simulink_pid/PID Controller/P Copy'
 * '<S16>'  : 'simulink_pid/PID Controller/Parallel P Gain'
 * '<S17>'  : 'simulink_pid/PID Controller/Reset Signal'
 * '<S18>'  : 'simulink_pid/PID Controller/Saturation'
 * '<S19>'  : 'simulink_pid/PID Controller/Saturation Fdbk'
 * '<S20>'  : 'simulink_pid/PID Controller/Sum'
 * '<S21>'  : 'simulink_pid/PID Controller/Sum Fdbk'
 * '<S22>'  : 'simulink_pid/PID Controller/Tracking Mode'
 * '<S23>'  : 'simulink_pid/PID Controller/Tracking Mode Sum'
 * '<S24>'  : 'simulink_pid/PID Controller/Tsamp - Integral'
 * '<S25>'  : 'simulink_pid/PID Controller/Tsamp - Ngain'
 * '<S26>'  : 'simulink_pid/PID Controller/postSat Signal'
 * '<S27>'  : 'simulink_pid/PID Controller/preSat Signal'
 * '<S28>'  : 'simulink_pid/PID Controller/Anti-windup/Passthrough'
 * '<S29>'  : 'simulink_pid/PID Controller/D Gain/Internal Parameters'
 * '<S30>'  : 'simulink_pid/PID Controller/External Derivative/Error'
 * '<S31>'  : 'simulink_pid/PID Controller/Filter/Cont. Filter'
 * '<S32>'  : 'simulink_pid/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'simulink_pid/PID Controller/I Gain/Internal Parameters'
 * '<S34>'  : 'simulink_pid/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'simulink_pid/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'simulink_pid/PID Controller/Integrator/Continuous'
 * '<S37>'  : 'simulink_pid/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'simulink_pid/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'simulink_pid/PID Controller/N Gain/Internal Parameters'
 * '<S40>'  : 'simulink_pid/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'simulink_pid/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S42>'  : 'simulink_pid/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'simulink_pid/PID Controller/Saturation/Passthrough'
 * '<S44>'  : 'simulink_pid/PID Controller/Saturation Fdbk/Disabled'
 * '<S45>'  : 'simulink_pid/PID Controller/Sum/Sum_PID'
 * '<S46>'  : 'simulink_pid/PID Controller/Sum Fdbk/Disabled'
 * '<S47>'  : 'simulink_pid/PID Controller/Tracking Mode/Disabled'
 * '<S48>'  : 'simulink_pid/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S49>'  : 'simulink_pid/PID Controller/Tsamp - Integral/Passthrough'
 * '<S50>'  : 'simulink_pid/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S51>'  : 'simulink_pid/PID Controller/postSat Signal/Forward_Path'
 * '<S52>'  : 'simulink_pid/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* simulink_pid_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
