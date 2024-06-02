/*
 * File: libsimodel.h
 *
 * Code generated for Simulink model 'libsimodel'.
 *
 * Model version                  : 1.493
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sun Jun  2 22:11:12 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-A
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_libsimodel_h_
#define RTW_HEADER_libsimodel_h_
#include "rtwtypes.h"
#include <stddef.h>
#include "zero_crossing_types.h"
#include <string.h>
#include <float.h>
#include <math.h>
#ifndef libsimodel_COMMON_INCLUDES_
#define libsimodel_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* libsimodel_COMMON_INCLUDES_ */

/* Model Code Variants */

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

/* Forward declaration for rtModel */
typedef struct tag_RTM_libsimodel_T RT_MODEL_libsimodel_T;

#ifndef DEFINED_TYPEDEF_FOR_HILGPS_
#define DEFINED_TYPEDEF_FOR_HILGPS_

typedef struct {
  uint8_T fix_type;
  uint8_T satellites_visible;
  real_T lat;
  real_T lon;
  real_T alt;
  real_T hAcc;
  real_T vAcc;
  real_T velN;
  real_T velE;
  real_T velD;
  real_T gSpeed;
  real_T headMot;
  real_T headVeh;
} HILGPS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavLinkGPS_
#define DEFINED_TYPEDEF_FOR_MavLinkGPS_

typedef struct {
  uint32_T time_usec;
  int32_T lat;
  int32_T lon;
  int32_T alt;
  uint16_T eph;
  uint16_T epv;
  uint16_T vel;
  int16_T vn;
  int16_T ve;
  int16_T vd;
  uint16_T cog;
  uint8_T fix_type;
  uint8_T satellites_visible;
} MavLinkGPS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavLinkSensor_
#define DEFINED_TYPEDEF_FOR_MavLinkSensor_

typedef struct {
  uint32_T time_usec;
  real32_T xacc;
  real32_T yacc;
  real32_T zacc;
  real32_T xgyro;
  real32_T ygyro;
  real32_T zgyro;
  real32_T xmag;
  real32_T ymag;
  real32_T zmag;
  real32_T abs_pressure;
  real32_T diff_pressure;
  real32_T pressure_alt;
  real32_T temperature;
  uint32_T fields_updated;
} MavLinkSensor;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MavVehileInfo_
#define DEFINED_TYPEDEF_FOR_MavVehileInfo_

typedef struct {
  int32_T copterID;
  int32_T vehicleType;
  real_T runnedTime;
  real32_T VelE[3];
  real32_T PosE[3];
  real32_T AngEuler[3];
  real32_T AngQuatern[4];
  real32_T MotorRPMS[8];
  real32_T AccB[3];
  real32_T RateB[3];
  real_T PosGPS[3];
} MavVehileInfo;

#endif

/* Block states (default storage) for system '<S111>/Negative Trace' */
typedef struct {
  int8_T FindMaximumDiagonalValue_Active;
                                      /* '<S118>/Find Maximum Diagonal Value' */
} DW_NegativeTrace_libsimodel_T;

/* Block signals for system '<S120>/If Warning//Error' */
typedef struct {
  real_T rtu_dcm[9];
} B_IfWarningError_libsimodel_T;

/* Block signals for system '<S305>/MATLAB Function' */
typedef struct {
  real_T rtu_K_m[9];
} B_MATLABFunction_libsimodel_T;

/* Block signals (default storage) */
typedef struct {
  real_T motorIn[40];                  /* '<S10>/Concatenate Motor Input' */
  real_T dv[40];
  MavVehileInfo BusCreator2;           /* '<S259>/Bus Creator2' */
  real_T MatrixConcatenation[18];      /* '<S217>/Matrix Concatenation' */
  real_T J_rt[9];                      /* '<S3>/AirframeFunction' */
  real_T VectorConcatenate_o[9];       /* '<S47>/Vector Concatenate' */
  real_T VectorConcatenate[9];
  real_T VectorConcatenate_k[9];
  real_T A[9];
  real_T rtb_Integrator_m[8];
  real_T MatrixMultiply[6];            /* '<S210>/MatrixMultiply' */
  real_T F_wind[3];
  real_T CMP_rt[3];                    /* '<S3>/AirframeFunction' */
  real_T Sum_ej[3];                    /* '<S326>/Sum' */
  real_T V_wind_b_relative[3];         /* '<S11>/MATLAB Function' */
  real_T IntegratorSecondOrderLimi_n[3];
                                 /* '<S371>/Integrator, Second-Order Limited' */
  real_T Sum_lv[3];                    /* '<S304>/Sum' */
  real_T IntegratorSecondOrderLimi_h[3];
                                 /* '<S325>/Integrator, Second-Order Limited' */
  real_T Product_lt[3];                /* '<S365>/Product' */
  real_T xeyeze[3];
  real_T rtb_IntegratorSecondOrderLim_cl[3];
  real_T runnedTime;                   /* '<S15>/Digital Clock' */
  real_T ModelInit_motorInitRate_b;    /* '<S62>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_j;    /* '<S63>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_c;    /* '<S64>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_l;    /* '<S65>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_lw;   /* '<S66>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_n;    /* '<S67>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_i;    /* '<S68>/ModelInit_motorInitRate' */
  real_T ModelInit_motorInitRate_ni;   /* '<S69>/ModelInit_motorInitRate' */
  real_T q0;                           /* '<S225>/q0' */
  real_T q1;                           /* '<S225>/q1' */
  real_T q2;                           /* '<S225>/q2' */
  real_T q3;                           /* '<S225>/q3' */
  real_T Product[3];                   /* '<S222>/Product' */
  real_T Merge[4];                     /* '<S112>/Merge' */
  real_T Merge_d[4];                   /* '<S111>/Merge' */
  real_T SumofElements[3];             /* '<S253>/Sum of Elements' */
  real_T Uk1;                          /* '<S8>/Delay Input1' */
  real_T Uk1_l;                        /* '<S7>/Delay Input1' */
  real_T Normalization[3];             /* '<S20>/Normalization' */
  real_T Normalization_c[3];           /* '<S21>/Normalization' */
  real_T q;                            /* '<S26>/q' */
  real_T q_f;                          /* '<S29>/q' */
  real_T Selector1[9];                 /* '<S216>/Selector1' */
  real_T Selector[9];                  /* '<S216>/Selector' */
  real_T Selector2[9];                 /* '<S216>/Selector2' */
  real_T Product2[3];                  /* '<S216>/Product2' */
  real_T UnaryMinus;                   /* '<S20>/Unary Minus' */
  real_T dot_q;                        /* '<S26>/Divide' */
  real_T UnaryMinus_n;                 /* '<S21>/Unary Minus' */
  real_T dot_q_k;                      /* '<S29>/Divide' */
  real_T Sum4[3];                      /* '<S306>/Sum4' */
  real_T RandomNumber5[3];             /* '<S301>/Random Number5' */
  real_T RandomNumber4[3];             /* '<S301>/Random Number4' */
  real_T Product_f[3];                 /* '<S301>/Product' */
  real_T Delay[3];                     /* '<S301>/Delay' */
  real_T Sum4_m[3];                    /* '<S364>/Sum4' */
  real_T RandomNumber4_f[3];           /* '<S304>/Random Number4' */
  real_T RandomNumber[3];              /* '<S304>/Random Number' */
  real_T Product_b[3];                 /* '<S304>/Product' */
  real_T Delay_m[3];                   /* '<S304>/Delay' */
  real_T RandomNumber_i[3];            /* '<S305>/Random Number' */
  real_T Product_d[3];                 /* '<S305>/Product' */
  real_T Delay_n[3];                   /* '<S305>/Delay' */
  real_T TmpRTBAtgps_location_noiseOutpo[6];/* '<S303>/gps_location_noise' */
  real_T Switch;                       /* '<S345>/Switch' */
  real_T Switch_m;                     /* '<S346>/Switch' */
  real_T Sum1_e;                       /* '<S338>/Sum1' */
  real_T Sum1_d[3];                    /* '<S303>/Sum1' */
  real_T Merge_o[4];                   /* '<S261>/Merge' */
  real_T Sum_h[3];                     /* '<S214>/Sum' */
  real_T Switch_b;                     /* '<S99>/Switch' */
  real_T TrigonometricFunction1;       /* '<S105>/Trigonometric Function1' */
  real_T TrigonometricFunction2;       /* '<S105>/Trigonometric Function2' */
  real_T Switch_m5;                    /* '<S100>/Switch' */
  real_T Divide;                       /* '<S71>/Divide' */
  real_T Divide_j;                     /* '<S73>/Divide' */
  real_T Divide_m;                     /* '<S75>/Divide' */
  real_T Divide_i;                     /* '<S77>/Divide' */
  real_T Divide_h;                     /* '<S79>/Divide' */
  real_T Divide_l;                     /* '<S81>/Divide' */
  real_T Divide_b;                     /* '<S83>/Divide' */
  real_T Divide_a;                     /* '<S85>/Divide' */
  real_T TmpSignalConversionAtq0q1q2q3_a[4];/* '<S215>/qdot' */
  real_T Sum2[3];                      /* '<S312>/Sum2' */
  real_T Sum4_f[3];                    /* '<S307>/Sum4' */
  real_T Sum2_n[3];                    /* '<S325>/Sum2' */
  real_T Sum2_o[3];                    /* '<S368>/Sum2' */
  real_T Sum4_c[3];                    /* '<S365>/Sum4' */
  real_T Sum2_d[3];                    /* '<S371>/Sum2' */
  real_T RandomNumber2[3];             /* '<S305>/Random Number2' */
  real_T gps_noise[6];                 /* '<S303>/gps_location_noise' */
  real_T vel;                          /* '<S303>/GenCogVel' */
  real_T cot;                          /* '<S303>/GenCogVel' */
  real_T F1[3];                        /* '<S87>/Ground Model' */
  real_T M1[3];                        /* '<S87>/Ground Model' */
  real_T m_rt;                         /* '<S3>/AirframeFunction' */
  real_T theta_1;                      /* '<S2>/MATLAB Function' */
  real_T omega_1d;                     /* '<S2>/MATLAB Function' */
  real_T theta_2;                      /* '<S2>/MATLAB Function' */
  real_T omega_d2;                     /* '<S2>/MATLAB Function' */
  real_T Rst;                          /* '<S2>/MATLAB Function' */
  real_T Merge_h;                      /* '<S242>/Merge' */
  real_T Merge_b;                      /* '<S53>/Merge' */
  real32_T TmpSignalConversionAtPayLoadInp[27];
  real_T Integrator;                   /* '<S71>/Integrator' */
  real_T Integrator_c;                 /* '<S73>/Integrator' */
  real_T Integrator_h;                 /* '<S75>/Integrator' */
  real_T Integrator_a;                 /* '<S77>/Integrator' */
  real_T Integrator_l;                 /* '<S79>/Integrator' */
  real_T Integrator_i;                 /* '<S81>/Integrator' */
  real_T Integrator_k;                 /* '<S83>/Integrator' */
  real_T Integrator_n;                 /* '<S85>/Integrator' */
  real_T jxi_h;                        /* '<S252>/j x i' */
  real_T Product1_aq;                  /* '<S52>/Product1' */
  real_T kxj_a;                        /* '<S252>/k x j' */
  real_T Theta;                        /* '<S29>/Theta' */
  real_T fcn3;                         /* '<S224>/fcn3' */
  real_T q3dot;                        /* '<S226>/q3dot' */
  real_T ixj;                          /* '<S332>/i x j' */
  real_T q0dot;                        /* '<S226>/q0dot' */
  real_T Product_mq;                   /* '<S302>/Product' */
  real_T rtb_IntegratorSecondOrderLimi_k;
  real_T rtb_IntegratorSecondOrderLim_cx;
  real_T rtb_IntegratorSecondOrderLimi_b;
  real_T rtb_IntegratorSecondOrderLimi_p;
  real_T rtb_IntegratorSecondOrderLim_cv;
  real_T rtb_IntegratorSecondOrderLimi_f;
  real_T rtb_ubvbwb_idx_2;
  real_T pqr_idx_2;
  real_T rtb_ubvbwb_idx_1;
  real_T pqr_idx_1;
  real_T rtb_ubvbwb_idx_0;
  real_T pqr_idx_0;
  real_T rtb_ZeroOrderHold3_a_idx_2;
  real_T rtb_ZeroOrderHold3_a_idx_1;
  real_T rtb_ZeroOrderHold3_a_idx_0;
  real_T rtb_q0q1q2q3_idx_3;
  real_T rtb_q0q1q2q3_idx_2;
  real_T rtb_q0q1q2q3_idx_1;
  real_T rtb_q0q1q2q3_idx_0;
  real_T rtb_Product1_l_idx_2;
  real_T rtb_ImpAsg_InsertedFor_F_at_i_g;
  real_T rtb_ImpAsg_InsertedFor_F_at__g1;
  real_T rtb_ImpAsg_InsertedFor_F_at_i_m;
  real_T rtb_Vector1_idx_2;
  real_T rtb_Vector1_idx_1;
  real_T rtb_Vector1_idx_0;
  real_T rtb_Sum_j_idx_2;
  real_T rtb_Sum_j_idx_1;
  real_T rtb_Sum_j_idx_0;
  real_T rtb_M_env_idx_2;
  real_T rtb_M_env_idx_1;
  real_T rtb_M_env_idx_0;
  real_T rtb_VectorConcatenate_idx_0;
  real_T q0_tmp;
  real_T rtb_ixk_h_tmp;
  real_T rtb_VectorConcatenate_o_tmp;
  real_T rtb_Vector2_idx_2;
  real_T rtb_Vector2_idx_1;
  real_T rtb_Vector2_idx_0;
  real_T rtb_ZeroOrderHold1_idx_2;
  real_T rtb_ZeroOrderHold1_idx_1;
  real_T rtb_ZeroOrderHold1_idx_0;
  real_T rtb_M_env_idx_0_tmp;
  real_T rtb_M_env_idx_1_tmp;
  real_T rtb_M_env_idx_2_tmp;
  real_T VectorConcatenate_tmp_tmp;
  real_T rtb_Product1_l_idx_2_tmp;
  real_T rtb_M_env_idx_1_tmp_n;
  real_T maxval;
  real_T a21;
  int32_T TmpSignalConversionAtPayLoadI_j[2];
  real32_T abs_pressure;               /* '<S258>/Gain' */
  real32_T rtb_Ameans0_idx_2;
  real32_T rtb_Ameans0_idx_1;
  real32_T rtb_Ameans0_idx_0;
  real32_T rtb_Wmeans0_idx_2;
  real32_T rtb_Wmeans0_idx_1;
  real32_T rtb_Wmeans0_idx_0;
  real32_T rtb_Gain4_idx_2;
  real32_T rtb_Gain4_idx_1;
  int32_T copterID;                    /* '<S259>/Data Type Conversion16' */
  int32_T vehicleType;                 /* '<S259>/Data Type Conversion17' */
  int32_T i;
  uint32_T time_usec;                  /* '<S15>/Data Type Conversion' */
  uint8_T fix_type;                    /* '<S257>/Data Type Conversion3' */
  uint8_T satellites_visible;          /* '<S257>/Data Type Conversion16' */
  uint8_T PackageHead[8];              /* '<S60>/Package Head' */
  uint8_T PayLoad[192];                /* '<S60>/PayLoad' */
  uint8_T S;                           /* '<S1>/Chart' */
  uint8_T Accel_Mag;                   /* '<S1>/Chart' */
  uint8_T step;                        /* '<S1>/Chart' */
  uint8_T status;                      /* '<S1>/Chart' */
  B_MATLABFunction_libsimodel_T sf_MATLABFunction1;/* '<S305>/MATLAB Function1' */
  B_MATLABFunction_libsimodel_T sf_MATLABFunction_b;/* '<S305>/MATLAB Function' */
  B_IfWarningError_libsimodel_T IfWarningError_k;/* '<S264>/If Warning//Error' */
  B_IfWarningError_libsimodel_T IfWarningError_g;/* '<S158>/If Warning//Error' */
  B_IfWarningError_libsimodel_T IfWarningError;/* '<S120>/If Warning//Error' */
} B_libsimodel_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput1_DSTATE;           /* '<S8>/Delay Input1' */
  real_T DelayInput1_DSTATE_e;         /* '<S7>/Delay Input1' */
  real_T Delay_DSTATE[300];            /* '<S301>/Delay' */
  real_T Delay_DSTATE_p[300];          /* '<S304>/Delay' */
  real_T Delay_DSTATE_k[300];          /* '<S305>/Delay' */
  real_T Product2_DWORK4[9];           /* '<S216>/Product2' */
  real_T NextOutput[3];                /* '<S301>/Random Number5' */
  real_T NextOutput_k[3];              /* '<S301>/Random Number4' */
  real_T NextOutput_l[3];              /* '<S304>/Random Number4' */
  real_T NextOutput_p[3];              /* '<S304>/Random Number' */
  real_T NextOutput_g[3];              /* '<S305>/Random Number' */
  real_T NextOutput_j;                 /* '<S302>/Random Number' */
  real_T TmpRTBAtgps_location_noiseOutpo[6];/* synthesized block */
  real_T NextOutput_i[3];              /* '<S305>/Random Number2' */
  real_T index;                        /* '<S303>/gps_location_noise' */
  real_T cot_buf;                      /* '<S303>/GenCogVel' */
  uint32_T RandSeed[3];                /* '<S301>/Random Number5' */
  uint32_T RandSeed_l[3];              /* '<S301>/Random Number4' */
  uint32_T RandSeed_p[3];              /* '<S304>/Random Number4' */
  uint32_T RandSeed_h[3];              /* '<S304>/Random Number' */
  uint32_T RandSeed_pg[3];             /* '<S305>/Random Number' */
  uint32_T RandSeed_d;                 /* '<S302>/Random Number' */
  uint32_T RandSeed_c[3];              /* '<S305>/Random Number2' */
  int_T Integrator_IWORK;              /* '<S71>/Integrator' */
  int_T Integrator_IWORK_d;            /* '<S73>/Integrator' */
  int_T Integrator_IWORK_h;            /* '<S75>/Integrator' */
  int_T Integrator_IWORK_k;            /* '<S77>/Integrator' */
  int_T Integrator_IWORK_dn;           /* '<S79>/Integrator' */
  int_T Integrator_IWORK_k4;           /* '<S81>/Integrator' */
  int_T Integrator_IWORK_b;            /* '<S83>/Integrator' */
  int_T Integrator_IWORK_ba;           /* '<S85>/Integrator' */
  int_T q0q1q2q3_IWORK;                /* '<S215>/q0 q1 q2 q3' */
  int_T IntegratorSecondOrderLimited_MO[3];
                                 /* '<S312>/Integrator, Second-Order Limited' */
  int_T IntegratorSecondOrderLimited__h[3];
                                 /* '<S368>/Integrator, Second-Order Limited' */
  int_T IntegratorSecondOrderLimited__j[3];
                                 /* '<S325>/Integrator, Second-Order Limited' */
  int_T IntegratorSecondOrderLimited__e[3];
                                 /* '<S371>/Integrator, Second-Order Limited' */
  int16_T RateTransition1_Buffer[3];   /* '<S1>/Rate Transition1' */
  int16_T RateTransition4_Buffer[3];   /* '<S1>/Rate Transition4' */
  int8_T If_ActiveSubsystem;           /* '<S242>/If' */
  int8_T If_ActiveSubsystem_d;         /* '<S112>/If' */
  int8_T If_ActiveSubsystem_o;         /* '<S111>/If' */
  int8_T If_ActiveSubsystem_a;         /* '<S53>/If' */
  int8_T If_ActiveSubsystem_b;         /* '<S261>/If' */
  int8_T If1_ActiveSubsystem;          /* '<S120>/If1' */
  int8_T If1_ActiveSubsystem_f;        /* '<S158>/If1' */
  int8_T If1_ActiveSubsystem_o;        /* '<S264>/If1' */
  int8_T takeoffFlag;                  /* '<S87>/Ground Model' */
  int8_T landFlag;                     /* '<S87>/Ground Model' */
  uint8_T is_c26_libsimodel;           /* '<S1>/Chart' */
  boolean_T IntegratorSecondOrderLimited_DW;
                                 /* '<S312>/Integrator, Second-Order Limited' */
  boolean_T IntegratorSecondOrderLimited__m;
                                 /* '<S368>/Integrator, Second-Order Limited' */
  boolean_T IntegratorSecondOrderLimited__l;
                                 /* '<S325>/Integrator, Second-Order Limited' */
  boolean_T IntegratorSecondOrderLimited__o;
                                 /* '<S371>/Integrator, Second-Order Limited' */
  DW_NegativeTrace_libsimodel_T NegativeTrace_a;/* '<S261>/Negative Trace' */
  DW_NegativeTrace_libsimodel_T NegativeTrace_i;/* '<S112>/Negative Trace' */
  DW_NegativeTrace_libsimodel_T NegativeTrace;/* '<S111>/Negative Trace' */
} DW_libsimodel_T;

/* Continuous states (default storage) */
typedef struct {
  real_T IntegratorSecondOrderLimited_CS[6];
                                 /* '<S312>/Integrator, Second-Order Limited' */
  real_T Integrator_CSTATE;            /* '<S71>/Integrator' */
  real_T Integrator_CSTATE_a;          /* '<S73>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S75>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S77>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S79>/Integrator' */
  real_T Integrator_CSTATE_mb;         /* '<S81>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S83>/Integrator' */
  real_T Integrator_CSTATE_fl;         /* '<S85>/Integrator' */
  real_T ubvbwb_CSTATE[3];             /* '<S214>/ub,vb,wb' */
  real_T pqr_CSTATE[3];                /* '<S214>/p,q,r ' */
  real_T q0q1q2q3_CSTATE[4];           /* '<S215>/q0 q1 q2 q3' */
  real_T xeyeze_CSTATE[3];             /* '<S214>/xe,ye,ze' */
  real_T Theta_CSTATE;                 /* '<S26>/Theta' */
  real_T Theta_CSTATE_b;               /* '<S29>/Theta' */
  real_T q_CSTATE;                     /* '<S26>/q' */
  real_T q_CSTATE_e;                   /* '<S29>/q' */
  real_T IntegratorSecondOrderLimited__j[6];
                                 /* '<S368>/Integrator, Second-Order Limited' */
  real_T IntegratorSecondOrderLimited__a[6];
                                 /* '<S325>/Integrator, Second-Order Limited' */
  real_T IntegratorSecondOrderLimited__h[6];
                                 /* '<S371>/Integrator, Second-Order Limited' */
} X_libsimodel_T;

/* Periodic continuous state vector (global) */
typedef int_T PeriodicIndX_libsimodel_T[2];
typedef real_T PeriodicRngX_libsimodel_T[4];

/* State derivatives (default storage) */
typedef struct {
  real_T IntegratorSecondOrderLimited_CS[6];
                                 /* '<S312>/Integrator, Second-Order Limited' */
  real_T Integrator_CSTATE;            /* '<S71>/Integrator' */
  real_T Integrator_CSTATE_a;          /* '<S73>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S75>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S77>/Integrator' */
  real_T Integrator_CSTATE_m;          /* '<S79>/Integrator' */
  real_T Integrator_CSTATE_mb;         /* '<S81>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S83>/Integrator' */
  real_T Integrator_CSTATE_fl;         /* '<S85>/Integrator' */
  real_T ubvbwb_CSTATE[3];             /* '<S214>/ub,vb,wb' */
  real_T pqr_CSTATE[3];                /* '<S214>/p,q,r ' */
  real_T q0q1q2q3_CSTATE[4];           /* '<S215>/q0 q1 q2 q3' */
  real_T xeyeze_CSTATE[3];             /* '<S214>/xe,ye,ze' */
  real_T Theta_CSTATE;                 /* '<S26>/Theta' */
  real_T Theta_CSTATE_b;               /* '<S29>/Theta' */
  real_T q_CSTATE;                     /* '<S26>/q' */
  real_T q_CSTATE_e;                   /* '<S29>/q' */
  real_T IntegratorSecondOrderLimited__j[6];
                                 /* '<S368>/Integrator, Second-Order Limited' */
  real_T IntegratorSecondOrderLimited__a[6];
                                 /* '<S325>/Integrator, Second-Order Limited' */
  real_T IntegratorSecondOrderLimited__h[6];
                                 /* '<S371>/Integrator, Second-Order Limited' */
} XDot_libsimodel_T;

/* State disabled  */
typedef struct {
  boolean_T IntegratorSecondOrderLimited_CS[6];
                                 /* '<S312>/Integrator, Second-Order Limited' */
  boolean_T Integrator_CSTATE;         /* '<S71>/Integrator' */
  boolean_T Integrator_CSTATE_a;       /* '<S73>/Integrator' */
  boolean_T Integrator_CSTATE_p;       /* '<S75>/Integrator' */
  boolean_T Integrator_CSTATE_i;       /* '<S77>/Integrator' */
  boolean_T Integrator_CSTATE_m;       /* '<S79>/Integrator' */
  boolean_T Integrator_CSTATE_mb;      /* '<S81>/Integrator' */
  boolean_T Integrator_CSTATE_f;       /* '<S83>/Integrator' */
  boolean_T Integrator_CSTATE_fl;      /* '<S85>/Integrator' */
  boolean_T ubvbwb_CSTATE[3];          /* '<S214>/ub,vb,wb' */
  boolean_T pqr_CSTATE[3];             /* '<S214>/p,q,r ' */
  boolean_T q0q1q2q3_CSTATE[4];        /* '<S215>/q0 q1 q2 q3' */
  boolean_T xeyeze_CSTATE[3];          /* '<S214>/xe,ye,ze' */
  boolean_T Theta_CSTATE;              /* '<S26>/Theta' */
  boolean_T Theta_CSTATE_b;            /* '<S29>/Theta' */
  boolean_T q_CSTATE;                  /* '<S26>/q' */
  boolean_T q_CSTATE_e;                /* '<S29>/q' */
  boolean_T IntegratorSecondOrderLimited__j[6];
                                 /* '<S368>/Integrator, Second-Order Limited' */
  boolean_T IntegratorSecondOrderLimited__a[6];
                                 /* '<S325>/Integrator, Second-Order Limited' */
  boolean_T IntegratorSecondOrderLimited__h[6];
                                 /* '<S371>/Integrator, Second-Order Limited' */
} XDis_libsimodel_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Theta_Reset_ZCE;          /* '<S29>/Theta' */
  ZCSigState q_Reset_ZCE;              /* '<S29>/q' */
} PrevZCX_libsimodel_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Divide;                 /* '<S4>/Divide' */
  const real_T Sum;                    /* '<S109>/Sum' */
  const real_T Product1;               /* '<S110>/Product1' */
  const real_T Sum1;                   /* '<S110>/Sum1' */
  const real_T sqrt_c;                 /* '<S110>/sqrt' */
  const real_T Product2;               /* '<S105>/Product2' */
  const real_T Sum1_p;                 /* '<S105>/Sum1' */
  const real_T UnitConversion;         /* '<S106>/Unit Conversion' */
  const real_T SinCos_o1;              /* '<S91>/SinCos' */
  const real_T SinCos_o2;              /* '<S91>/SinCos' */
  const real_T nT2Gauss[3];            /* '<S11>/nT2Gauss' */
  const real_T TmpSignalConversionAtForEac[3];
  const real_T Sum_n;                  /* '<S361>/Sum' */
  const real_T Product1_i;             /* '<S362>/Product1' */
  const real_T Sum1_a;                 /* '<S362>/Sum1' */
  const real_T sqrt_l;                 /* '<S362>/sqrt' */
  const real_T Product2_j;             /* '<S357>/Product2' */
  const real_T Sum1_g;                 /* '<S357>/Sum1' */
  const real_T UnitConversion_j;       /* '<S358>/Unit Conversion' */
  const real_T SinCos_o1_a;            /* '<S343>/SinCos' */
  const real_T SinCos_o2_j;            /* '<S343>/SinCos' */
} ConstB_libsimodel_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [0 0 0 0 0]
   * Referenced by: '<S60>/reserve'
   */
  real_T reserve_Value[5];

  /* Pooled Parameter (Expression: -eye(3))
   * Referenced by:
   *   '<S147>/Bias1'
   *   '<S185>/Bias1'
   *   '<S291>/Bias1'
   */
  real_T pooled9[9];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S301>/K_pulse0'
   *   '<S304>/K_pulse0'
   *   '<S305>/K_m0'
   *   '<S305>/K_m1'
   *   '<S305>/K_pulse0'
   *   '<S306>/Scale factors & Cross-coupling  errors'
   *   '<S307>/Scale factors & Cross-coupling  errors'
   *   '<S364>/Scale factors & Cross-coupling  errors '
   *   '<S365>/Scale factors & Cross-coupling  errors '
   */
  real_T pooled15[9];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S2>/Constant1'
   *   '<S2>/Constant2'
   *   '<S2>/Ve'
   *   '<S2>/Xe'
   *   '<S11>/Constant2'
   *   '<S210>/Constant'
   *   '<S301>/Random Number4'
   *   '<S301>/Random Number5'
   *   '<S304>/Random Number'
   *   '<S304>/Random Number4'
   *   '<S305>/Measurement bias0'
   *   '<S305>/Measurement bias1'
   *   '<S305>/Random Number'
   *   '<S305>/Random Number2'
   *   '<S306>/Measurement bias'
   *   '<S307>/Measurement bias'
   *   '<S364>/Measurement bias'
   *   '<S364>/g-sensitive bias'
   *   '<S365>/Measurement bias'
   *   '<S365>/g-sensitive bias'
   */
  real_T pooled21[3];

  /* Computed Parameter: len_Value
   * Referenced by: '<S60>/len'
   */
  int32_T len_Value;

  /* Computed Parameter: TargetType_Value
   * Referenced by: '<S60>/TargetType'
   */
  uint32_T TargetType_Value;
} ConstP_libsimodel_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T inPWMs[8];                    /* '<Root>/inPWMs' */
  real_T TerrainZ;                     /* '<Root>/TerrainZ' */
  real_T keyMode;                      /* '<Root>/keyMode' */
  real_T keyStep;                      /* '<Root>/keyStep' */
} ExtU_libsimodel_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  MavLinkSensor MavLinkSensorData;     /* '<Root>/MavLinkSensorData' */
  HILGPS HILGPSData;                   /* '<Root>/HILGPSData' */
  uint8_T UnRealData[200];             /* '<Root>/UnRealData' */
  uint8_T ModelStatus[2];              /* '<Root>/ModelStatus' */
  real_T TimeSecond;                   /* '<Root>/TimeSecond' */
  int16_T GyroSensorData[3];           /* '<Root>/GyroSensorData' */
  int16_T AccelSensorData[3];          /* '<Root>/AccelSensorData' */
  int16_T TempData;                    /* '<Root>/TempData' */
  int16_T MagSensorData[3];            /* '<Root>/MagSensorData' */
  uint32_T PressureTempData[2];        /* '<Root>/PressureTempData' */
} ExtY_libsimodel_T;

/* Parameters (default storage) */
struct P_libsimodel_T_ {
  real_T gps_fake_noise[17736];        /* Variable: gps_fake_noise
                                        * Referenced by: '<S303>/gps_location_noise'
                                        */
  uint32_T gps_fake_noise_length;      /* Variable: gps_fake_noise_length
                                        * Referenced by: '<S303>/gps_location_noise'
                                        */
};

/* Parameters (default storage) */
typedef struct P_libsimodel_T_ P_libsimodel_T;

/* Real-time Model Data Structure */
struct tag_RTM_libsimodel_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_libsimodel_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[49];
  real_T odeF[4][49];
  ODE4_IntgData intgData;

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
    boolean_T firstInitCondFlag;
    struct {
      uint16_T TID[5];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[5];
  } Timing;
};

/* Block parameters (default storage) */
extern P_libsimodel_T libsimodel_P;

/* Block signals (default storage) */
extern B_libsimodel_T libsimodel_B;

/* Continuous states (default storage) */
extern X_libsimodel_T libsimodel_X;

/* Block states (default storage) */
extern DW_libsimodel_T libsimodel_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_libsimodel_T libsimodel_PrevZCX;

/* External inputs (root inport signals with default storage) */
extern ExtU_libsimodel_T libsimodel_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_libsimodel_T libsimodel_Y;
extern const ConstB_libsimodel_T libsimodel_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_libsimodel_T libsimodel_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real_T BoardRotation[3];        /* Variable: BoardRotation
                                        * Referenced by: '<S1>/Constant'
                                        */
extern real_T EfficiencyMatrix[48];    /* Variable: EfficiencyMatrix
                                        * Referenced by: '<S210>/Constant1'
                                        */
extern real_T IST8310_ConvertRatio;    /* Variable: IST8310_ConvertRatio
                                        * Referenced by: '<S1>/IST8310_0.3uT_LSB'
                                        */
extern real_T ModelFail_Airframe_load_J[9];/* Variable: ModelFail_Airframe_load_J
                                            * Referenced by: '<S3>/ModelFail_Airframe_load_J'
                                            */
extern real_T ModelFail_Airframe_load_P[3];/* Variable: ModelFail_Airframe_load_P
                                            * Referenced by: '<S3>/ModelFail_Airframe_load_P'
                                            */
extern real_T ModelFail_Airframe_load_Type;/* Variable: ModelFail_Airframe_load_Type
                                            * Referenced by: '<S3>/ModelFail_Airframe_load_Type'
                                            */
extern real_T ModelFail_Airframe_load_m;/* Variable: ModelFail_Airframe_load_m
                                         * Referenced by: '<S3>/ModelFail_Airframe_load_m'
                                         */
extern real_T ModelFail_env_P_wind[3]; /* Variable: ModelFail_env_P_wind
                                        * Referenced by: '<S11>/ModelFail_env_P_wind'
                                        */
extern real_T ModelFail_motor_isEnable;/* Variable: ModelFail_motor_isEnable
                                        * Referenced by: '<S10>/ModelFail_motor_isEnable'
                                        */
extern real_T ModelFail_motor_kCt[8];  /* Variable: ModelFail_motor_kCt
                                        * Referenced by: '<S10>/ModelFail_motor_kCt'
                                        */
extern real_T ModelFail_motor_kTc[8];  /* Variable: ModelFail_motor_kTc
                                        * Referenced by: '<S10>/ModelFail_motor_kTc'
                                        */
extern real_T ModelFail_motor_kw[8];   /* Variable: ModelFail_motor_kw
                                        * Referenced by: '<S10>/ModelFail_motor_kw'
                                        */
extern real_T ModelInit_AngEuler[3];   /* Variable: ModelInit_AngEuler
                                        * Referenced by: '<S215>/Initial Euler Angles'
                                        */
extern real_T ModelInit_PosE[3];       /* Variable: ModelInit_PosE
                                        * Referenced by: '<S214>/xe,ye,ze'
                                        */
extern real_T ModelInit_RateB[3];      /* Variable: ModelInit_RateB
                                        * Referenced by: '<S214>/p,q,r '
                                        */
extern real_T ModelInit_VelB[3];       /* Variable: ModelInit_VelB
                                        * Referenced by: '<S214>/ub,vb,wb'
                                        */
extern real_T ModelInit_motorInitRate; /* Variable: ModelInit_motorInitRate
                                        * Referenced by:
                                        *   '<S62>/ModelInit_motorInitRate'
                                        *   '<S63>/ModelInit_motorInitRate'
                                        *   '<S64>/ModelInit_motorInitRate'
                                        *   '<S65>/ModelInit_motorInitRate'
                                        *   '<S66>/ModelInit_motorInitRate'
                                        *   '<S67>/ModelInit_motorInitRate'
                                        *   '<S68>/ModelInit_motorInitRate'
                                        *   '<S69>/ModelInit_motorInitRate'
                                        */
extern real_T ModelParam_Airframe_CMP[3];/* Variable: ModelParam_Airframe_CMP
                                          * Referenced by: '<S3>/ModelParam_Airframe_CenterOfMassPosition'
                                          */
extern real_T ModelParam_Airframe_J[9];/* Variable: ModelParam_Airframe_J
                                        * Referenced by: '<S3>/ModelParam_Airframe_J'
                                        */
extern real_T ModelParam_Airframe_m;   /* Variable: ModelParam_Airframe_m
                                        * Referenced by: '<S3>/ModelParam_Airframe_m'
                                        */
extern real_T ModelParam_GPSEphFinal;  /* Variable: ModelParam_GPSEphFinal
                                        * Referenced by: '<S303>/ModelParam.GPSEphFinal'
                                        */
extern real_T ModelParam_GPSEpvFinal;  /* Variable: ModelParam_GPSEpvFinal
                                        * Referenced by: '<S303>/ModelParam.GPSEpvFinal'
                                        */
extern real_T ModelParam_GPSFix3DFix;  /* Variable: ModelParam_GPSFix3DFix
                                        * Referenced by: '<S303>/ModelParam_GPSFix3DFix'
                                        */
extern real_T ModelParam_GPSLatLong[2];/* Variable: ModelParam_GPSLatLong
                                        * Referenced by:
                                        *   '<S86>/ref_position'
                                        *   '<S338>/ref_position'
                                        */
extern real_T ModelParam_GPSSatsVisible;/* Variable: ModelParam_GPSSatsVisible
                                         * Referenced by: '<S303>/ModelParam.GPSSatsVisible'
                                         */
extern real_T ModelParam_NoiseVarAcc0[3];/* Variable: ModelParam_NoiseVarAcc0
                                          * Referenced by:
                                          *   '<S301>/Random Number4'
                                          *   '<S301>/Random Number5'
                                          */
extern real_T ModelParam_NoiseVarGyro0[3];/* Variable: ModelParam_NoiseVarGyro0
                                           * Referenced by:
                                           *   '<S304>/Random Number'
                                           *   '<S304>/Random Number4'
                                           */
extern real_T ModelParam_NoiseVarMag0[3];/* Variable: ModelParam_NoiseVarMag0
                                          * Referenced by: '<S305>/Random Number'
                                          */
extern real_T ModelParam_NoiseVarMag1[3];/* Variable: ModelParam_NoiseVarMag1
                                          * Referenced by: '<S305>/Random Number2'
                                          */
extern real_T ModelParam_PositionAcc0[3];/* Variable: ModelParam_PositionAcc0
                                          * Referenced by: '<S306>/wl_ins'
                                          */
extern real_T ModelParam_PositionAcc1[3];/* Variable: ModelParam_PositionAcc1
                                          * Referenced by: '<S307>/wl_ins'
                                          */
extern real_T ModelParam_envAltitude;  /* Variable: ModelParam_envAltitude
                                        * Referenced by:
                                        *   '<S11>/ModelParam_envAltitude'
                                        *   '<S303>/ModelParam_envAltitude'
                                        */
extern real_T ModelParam_envC_d;       /* Variable: ModelParam_envC_d
                                        * Referenced by: '<S11>/ModelParam_envC_d'
                                        */
extern real_T ModelParam_envC_md[3];   /* Variable: ModelParam_envC_md
                                        * Referenced by: '<S11>/ModelParam_envC_md'
                                        */
extern real_T ModelParam_motorCr;      /* Variable: ModelParam_motorCr
                                        * Referenced by:
                                        *   '<S62>/ModelParam_motorCr'
                                        *   '<S63>/ModelParam_motorCr'
                                        *   '<S64>/ModelParam_motorCr'
                                        *   '<S65>/ModelParam_motorCr'
                                        *   '<S66>/ModelParam_motorCr'
                                        *   '<S67>/ModelParam_motorCr'
                                        *   '<S68>/ModelParam_motorCr'
                                        *   '<S69>/ModelParam_motorCr'
                                        */
extern real_T ModelParam_motorFitType; /* Variable: ModelParam_motorFitType
                                        * Referenced by:
                                        *   '<S62>/ModelParam_motorFitType'
                                        *   '<S63>/ModelParam_motorFitType'
                                        *   '<S64>/ModelParam_motorFitType'
                                        *   '<S65>/ModelParam_motorFitType'
                                        *   '<S66>/ModelParam_motorFitType'
                                        *   '<S67>/ModelParam_motorFitType'
                                        *   '<S68>/ModelParam_motorFitType'
                                        *   '<S69>/ModelParam_motorFitType'
                                        */
extern real_T ModelParam_motorJm;      /* Variable: ModelParam_motorJm
                                        * Referenced by: '<S210>/Gain'
                                        */
extern real_T ModelParam_motorMinThr;  /* Variable: ModelParam_motorMinThr
                                        * Referenced by:
                                        *   '<S62>/ESC_MotorSteadyState'
                                        *   '<S63>/ESC_MotorSteadyState'
                                        *   '<S64>/ESC_MotorSteadyState'
                                        *   '<S65>/ESC_MotorSteadyState'
                                        *   '<S66>/ESC_MotorSteadyState'
                                        *   '<S67>/ESC_MotorSteadyState'
                                        *   '<S68>/ESC_MotorSteadyState'
                                        *   '<S69>/ESC_MotorSteadyState'
                                        */
extern real_T ModelParam_motorRateCurveCoeffi[3];
                                    /* Variable: ModelParam_motorRateCurveCoeffi
                                     * Referenced by:
                                     *   '<S62>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S63>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S64>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S65>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S66>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S67>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S68>/ModelParam_motorRateCurveCoeffi'
                                     *   '<S69>/ModelParam_motorRateCurveCoeffi'
                                     */
extern real_T ModelParam_motorTc;      /* Variable: ModelParam_motorTc
                                        * Referenced by:
                                        *   '<S62>/ModelParam_motorTc'
                                        *   '<S63>/ModelParam_motorTc'
                                        *   '<S64>/ModelParam_motorTc'
                                        *   '<S65>/ModelParam_motorTc'
                                        *   '<S66>/ModelParam_motorTc'
                                        *   '<S67>/ModelParam_motorTc'
                                        *   '<S68>/ModelParam_motorTc'
                                        *   '<S69>/ModelParam_motorTc'
                                        */
extern real_T ModelParam_motorWb;      /* Variable: ModelParam_motorWb
                                        * Referenced by:
                                        *   '<S62>/ModelParam_motorWb'
                                        *   '<S63>/ModelParam_motorWb'
                                        *   '<S64>/ModelParam_motorWb'
                                        *   '<S65>/ModelParam_motorWb'
                                        *   '<S66>/ModelParam_motorWb'
                                        *   '<S67>/ModelParam_motorWb'
                                        *   '<S68>/ModelParam_motorWb'
                                        *   '<S69>/ModelParam_motorWb'
                                        */
extern real_T ModelParam_rotorCt;      /* Variable: ModelParam_rotorCt
                                        * Referenced by:
                                        *   '<S62>/ModelParam_rotorCt'
                                        *   '<S63>/ModelParam_rotorCt'
                                        *   '<S64>/ModelParam_rotorCt'
                                        *   '<S65>/ModelParam_rotorCt'
                                        *   '<S66>/ModelParam_rotorCt'
                                        *   '<S67>/ModelParam_rotorCt'
                                        *   '<S68>/ModelParam_rotorCt'
                                        *   '<S69>/ModelParam_rotorCt'
                                        */
extern real_T RflySimCopterID;         /* Variable: RflySimCopterID
                                        * Referenced by: '<S259>/CopterID'
                                        */
extern real_T RflySimDisplayUAVType;   /* Variable: RflySimDisplayUAVType
                                        * Referenced by: '<S259>/UAVType'
                                        */
extern real_T RotorDirectionVector[8]; /* Variable: RotorDirectionVector
                                        * Referenced by: '<S210>/Constant2'
                                        */

/* Model entry point functions */
extern void libsimodel_initialize(void);
extern void libsimodel_step(void);
extern void libsimodel_terminate(void);

/* Real-time Model object */
extern RT_MODEL_libsimodel_T *const libsimodel_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S28>/Data Type Duplicate' : Unused code path elimination
 * Block '<S28>/Data Type Propagation' : Unused code path elimination
 * Block '<S31>/Data Type Duplicate' : Unused code path elimination
 * Block '<S31>/Data Type Propagation' : Unused code path elimination
 * Block '<S2>/Vb' : Unused code path elimination
 * Block '<S4>/Constant' : Unused code path elimination
 * Block '<S4>/Constant1' : Unused code path elimination
 * Block '<S11>/Constant' : Unused code path elimination
 * Block '<S11>/Saturation' : Unused code path elimination
 * Block '<S11>/magDec' : Unused code path elimination
 * Block '<S257>/AngleScale' : Unused code path elimination
 * Block '<S257>/Bus Creator1' : Unused code path elimination
 * Block '<S257>/Data Type Conversion10' : Unused code path elimination
 * Block '<S257>/Data Type Conversion11' : Unused code path elimination
 * Block '<S257>/Data Type Conversion12' : Unused code path elimination
 * Block '<S257>/Data Type Conversion13' : Unused code path elimination
 * Block '<S257>/Data Type Conversion14' : Unused code path elimination
 * Block '<S257>/Data Type Conversion15' : Unused code path elimination
 * Block '<S257>/Data Type Conversion4' : Unused code path elimination
 * Block '<S257>/Data Type Conversion5' : Unused code path elimination
 * Block '<S257>/Data Type Conversion6' : Unused code path elimination
 * Block '<S257>/Data Type Conversion7' : Unused code path elimination
 * Block '<S257>/Data Type Conversion8' : Unused code path elimination
 * Block '<S257>/Data Type Conversion9' : Unused code path elimination
 * Block '<S257>/Gain3' : Unused code path elimination
 * Block '<S257>/Gain6' : Unused code path elimination
 * Block '<S257>/VdScale' : Unused code path elimination
 * Block '<S257>/VeScale' : Unused code path elimination
 * Block '<S257>/VelScale' : Unused code path elimination
 * Block '<S257>/VnScale' : Unused code path elimination
 * Block '<S257>/altScale' : Unused code path elimination
 * Block '<S257>/latScale' : Unused code path elimination
 * Block '<S257>/lonScale' : Unused code path elimination
 * Block '<S301>/Delay1' : Unused code path elimination
 * Block '<S301>/DelayLength1' : Unused code path elimination
 * Block '<S301>/Enable1' : Unused code path elimination
 * Block '<S301>/K_pulse1' : Unused code path elimination
 * Block '<S301>/Product1' : Unused code path elimination
 * Block '<S301>/Random Number2' : Unused code path elimination
 * Block '<S301>/Rate Transition1' : Unused code path elimination
 * Block '<S301>/Sum1' : Unused code path elimination
 * Block '<S321>/Constant' : Unused code path elimination
 * Block '<S321>/Switch' : Unused code path elimination
 * Block '<S307>/Saturation' : Unused code path elimination
 * Block '<S307>/Sum1' : Unused code path elimination
 * Block '<S336>/Unit Conversion' : Unused code path elimination
 * Block '<S304>/Delay1' : Unused code path elimination
 * Block '<S304>/DelayLength1' : Unused code path elimination
 * Block '<S304>/Enable1' : Unused code path elimination
 * Block '<S304>/K_pulse1' : Unused code path elimination
 * Block '<S304>/Product1' : Unused code path elimination
 * Block '<S304>/Random Number2' : Unused code path elimination
 * Block '<S304>/Rate Transition1' : Unused code path elimination
 * Block '<S304>/Sum1' : Unused code path elimination
 * Block '<S369>/Constant' : Unused code path elimination
 * Block '<S369>/Switch' : Unused code path elimination
 * Block '<S365>/Saturation' : Unused code path elimination
 * Block '<S365>/Sum1' : Unused code path elimination
 * Block '<S305>/Delay1' : Unused code path elimination
 * Block '<S305>/DelayLength1' : Unused code path elimination
 * Block '<S305>/Enable1' : Unused code path elimination
 * Block '<S305>/K_pulse1' : Unused code path elimination
 * Block '<S305>/Product1' : Unused code path elimination
 * Block '<S305>/Rate Transition1' : Unused code path elimination
 * Block '<S47>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S1>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S111>/Reshape 3x3 -> 9' : Reshape block reduction
 * Block '<S147>/Reshape' : Reshape block reduction
 * Block '<S112>/Reshape 3x3 -> 9' : Reshape block reduction
 * Block '<S185>/Reshape' : Reshape block reduction
 * Block '<S203>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S210>/Reshape1' : Reshape block reduction
 * Block '<S236>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S226>/High Gain Quaternion Normalization' : Eliminated nontunable gain of 1
 * Block '<S249>/Reshape1' : Reshape block reduction
 * Block '<S249>/Reshape2' : Reshape block reduction
 * Block '<S250>/Reshape1' : Reshape block reduction
 * Block '<S250>/Reshape2' : Reshape block reduction
 * Block '<S216>/Reshape' : Reshape block reduction
 * Block '<S216>/Reshape1' : Reshape block reduction
 * Block '<S217>/Reshape' : Reshape block reduction
 * Block '<S219>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S220>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S221>/Unit Conversion' : Eliminated nontunable gain of 1
 * Block '<S222>/Reshape1' : Reshape block reduction
 * Block '<S222>/Reshape2' : Reshape block reduction
 * Block '<S257>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain2' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain4' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain5' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain7' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain8' : Eliminated nontunable gain of 1
 * Block '<S257>/Gain9' : Eliminated nontunable gain of 1
 * Block '<S259>/Data Type Conversion18' : Eliminate redundant data type conversion
 * Block '<S259>/Data Type Conversion25' : Eliminate redundant data type conversion
 * Block '<S261>/Reshape 3x3 -> 9' : Reshape block reduction
 * Block '<S291>/Reshape' : Reshape block reduction
 * Block '<S1>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition5' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition6' : Eliminated since input and output rates are identical
 * Block '<S300>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S301>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S306>/Reshape1' : Reshape block reduction
 * Block '<S306>/Saturation' : Eliminated Saturate block
 * Block '<S306>/Zero-Order Hold4' : Eliminated since input and output rates are identical
 * Block '<S307>/Reshape1' : Reshape block reduction
 * Block '<S307>/Zero-Order Hold4' : Eliminated since input and output rates are identical
 * Block '<S302>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S302>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S303>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S303>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S304>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S364>/Saturation' : Eliminated Saturate block
 * Block '<S305>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S308>/Constant' : Unused code path elimination
 * Block '<S366>/Constant' : Unused code path elimination
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
 * '<Root>' : 'libsimodel'
 * '<S1>'   : 'libsimodel/Model'
 * '<S2>'   : 'libsimodel/Model/6DOF Calibration'
 * '<S3>'   : 'libsimodel/Model/Airframe Model'
 * '<S4>'   : 'libsimodel/Model/Battery Model'
 * '<S5>'   : 'libsimodel/Model/Chart'
 * '<S6>'   : 'libsimodel/Model/Degrees to Radians'
 * '<S7>'   : 'libsimodel/Model/Detect Decrease'
 * '<S8>'   : 'libsimodel/Model/Detect Decrease1'
 * '<S9>'   : 'libsimodel/Model/DisplayPack'
 * '<S10>'  : 'libsimodel/Model/ESC&Motor Model'
 * '<S11>'  : 'libsimodel/Model/Environment Model'
 * '<S12>'  : 'libsimodel/Model/Force and Moment Model'
 * '<S13>'  : 'libsimodel/Model/Kinematics 6DOF Model'
 * '<S14>'  : 'libsimodel/Model/MATLAB Function'
 * '<S15>'  : 'libsimodel/Model/OutputHub'
 * '<S16>'  : 'libsimodel/Model/Rotation Angles to Direction Cosine Matrix'
 * '<S17>'  : 'libsimodel/Model/Sensor Model'
 * '<S18>'  : 'libsimodel/Model/6DOF Calibration/MATLAB Function'
 * '<S19>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU'
 * '<S20>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate'
 * '<S21>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate1'
 * '<S22>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Multiplication'
 * '<S23>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Normalize'
 * '<S24>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix'
 * '<S25>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles'
 * '<S26>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate/1DOF'
 * '<S27>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate/GenQuate'
 * '<S28>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate/Saturation Dynamic'
 * '<S29>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate1/1DOF'
 * '<S30>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate1/GenQuate'
 * '<S31>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/OneAxisRotate1/Saturation Dynamic'
 * '<S32>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Multiplication/q0'
 * '<S33>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Multiplication/q1'
 * '<S34>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Multiplication/q2'
 * '<S35>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Multiplication/q3'
 * '<S36>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Normalize/Quaternion Modulus'
 * '<S37>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S38>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A11'
 * '<S39>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A12'
 * '<S40>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A13'
 * '<S41>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A21'
 * '<S42>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A22'
 * '<S43>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A23'
 * '<S44>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A31'
 * '<S45>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A32'
 * '<S46>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/A33'
 * '<S47>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S48>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S49>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S50>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S51>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Angle Calculation'
 * '<S52>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Quaternion Normalize'
 * '<S53>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
 * '<S54>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
 * '<S55>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
 * '<S56>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
 * '<S57>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
 * '<S58>'  : 'libsimodel/Model/6DOF Calibration/Quaternion2IMU/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S59>'  : 'libsimodel/Model/Airframe Model/AirframeFunction'
 * '<S60>'  : 'libsimodel/Model/DisplayPack/UnRealDataPackege'
 * '<S61>'  : 'libsimodel/Model/ESC&Motor Model/Concatenate Motor Input'
 * '<S62>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic1'
 * '<S63>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic2'
 * '<S64>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic3'
 * '<S65>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic4'
 * '<S66>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic5'
 * '<S67>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic6'
 * '<S68>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic7'
 * '<S69>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic8'
 * '<S70>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic1/ESC_MotorSteadyState'
 * '<S71>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic1/MotorDynamic'
 * '<S72>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic2/ESC_MotorSteadyState'
 * '<S73>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic2/MotorDynamic'
 * '<S74>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic3/ESC_MotorSteadyState'
 * '<S75>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic3/MotorDynamic'
 * '<S76>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic4/ESC_MotorSteadyState'
 * '<S77>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic4/MotorDynamic'
 * '<S78>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic5/ESC_MotorSteadyState'
 * '<S79>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic5/MotorDynamic'
 * '<S80>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic6/ESC_MotorSteadyState'
 * '<S81>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic6/MotorDynamic'
 * '<S82>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic7/ESC_MotorSteadyState'
 * '<S83>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic7/MotorDynamic'
 * '<S84>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic8/ESC_MotorSteadyState'
 * '<S85>'  : 'libsimodel/Model/ESC&Motor Model/MotorNonlinearDynamic8/MotorDynamic'
 * '<S86>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA'
 * '<S87>'  : 'libsimodel/Model/Environment Model/Ground Model'
 * '<S88>'  : 'libsimodel/Model/Environment Model/MATLAB Function'
 * '<S89>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap'
 * '<S90>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1'
 * '<S91>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset'
 * '<S92>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/pos_deg'
 * '<S93>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90'
 * '<S94>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Wrap Longitude'
 * '<S95>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S96>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S97>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S98>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S99>'  : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90'
 * '<S100>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Wrap Longitude'
 * '<S101>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Compare To Constant'
 * '<S102>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180'
 * '<S103>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S104>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Compare To Constant'
 * '<S105>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/Find Radian//Distance'
 * '<S106>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/rotation_rad'
 * '<S107>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Angle Conversion2'
 * '<S108>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/denom'
 * '<S109>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e'
 * '<S110>' : 'libsimodel/Model/Environment Model/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e^4'
 * '<S111>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions'
 * '<S112>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1'
 * '<S113>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM'
 * '<S114>' : 'libsimodel/Model/Environment Model/Ground Model/Ground Model'
 * '<S115>' : 'libsimodel/Model/Environment Model/Ground Model/OnGroundFaceup'
 * '<S116>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Inverse'
 * '<S117>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Multiplication'
 * '<S118>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace'
 * '<S119>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Positive Trace'
 * '<S120>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM'
 * '<S121>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/trace(DCM)'
 * '<S122>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
 * '<S123>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
 * '<S124>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
 * '<S125>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
 * '<S126>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S127>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S128>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S129>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
 * '<S130>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
 * '<S131>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S132>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S133>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S134>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
 * '<S135>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
 * '<S136>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S137>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S138>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S139>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
 * '<S140>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
 * '<S141>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S142>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S143>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S144>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
 * '<S145>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S146>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
 * '<S147>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S148>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
 * '<S149>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S150>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S151>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S152>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S153>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S154>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S155>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 * '<S156>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace'
 * '<S157>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Positive Trace'
 * '<S158>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM'
 * '<S159>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/trace(DCM)'
 * '<S160>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)'
 * '<S161>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)'
 * '<S162>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)'
 * '<S163>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/diag(DCM)'
 * '<S164>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S165>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S166>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S167>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
 * '<S168>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
 * '<S169>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S170>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S171>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S172>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
 * '<S173>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
 * '<S174>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S175>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S176>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S177>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
 * '<S178>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
 * '<S179>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S180>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S181>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S182>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error'
 * '<S183>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S184>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper'
 * '<S185>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S186>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper'
 * '<S187>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S188>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S189>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S190>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S191>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S192>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S193>' : 'libsimodel/Model/Environment Model/Ground Model/Direction Cosine Matrix  to Quaternions1/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 * '<S194>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A11'
 * '<S195>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A12'
 * '<S196>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A13'
 * '<S197>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A21'
 * '<S198>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A22'
 * '<S199>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A23'
 * '<S200>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A31'
 * '<S201>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A32'
 * '<S202>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/A33'
 * '<S203>' : 'libsimodel/Model/Environment Model/Ground Model/Euler to DCM/Create Transformation Matrix'
 * '<S204>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Inverse/Quaternion Conjugate'
 * '<S205>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Inverse/Quaternion Norm'
 * '<S206>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Multiplication/q0'
 * '<S207>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Multiplication/q1'
 * '<S208>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Multiplication/q2'
 * '<S209>' : 'libsimodel/Model/Environment Model/Ground Model/Quaternion Multiplication/q3'
 * '<S210>' : 'libsimodel/Model/Force and Moment Model/Allocation Model'
 * '<S211>' : 'libsimodel/Model/Force and Moment Model/Allocation Model/3x3 Cross Product'
 * '<S212>' : 'libsimodel/Model/Force and Moment Model/Allocation Model/3x3 Cross Product/Subsystem'
 * '<S213>' : 'libsimodel/Model/Force and Moment Model/Allocation Model/3x3 Cross Product/Subsystem1'
 * '<S214>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)'
 * '<S215>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles'
 * '<S216>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot'
 * '<S217>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia'
 * '<S218>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Vbxw'
 * '<S219>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion'
 * '<S220>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion1'
 * '<S221>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Velocity Conversion2'
 * '<S222>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/transform to Inertial axes '
 * '<S223>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix'
 * '<S224>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles'
 * '<S225>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Rotation Angles to Quaternions'
 * '<S226>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/qdot'
 * '<S227>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A11'
 * '<S228>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A12'
 * '<S229>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A13'
 * '<S230>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A21'
 * '<S231>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A22'
 * '<S232>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A23'
 * '<S233>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A31'
 * '<S234>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A32'
 * '<S235>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/A33'
 * '<S236>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S237>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S238>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S239>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S240>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation'
 * '<S241>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize'
 * '<S242>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
 * '<S243>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
 * '<S244>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
 * '<S245>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
 * '<S246>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
 * '<S247>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate DCM & Euler Angles/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S248>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product'
 * '<S249>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/I x w'
 * '<S250>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/I x w1'
 * '<S251>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S252>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S253>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia/Mass input//output  momentum'
 * '<S254>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Determine Force,  Mass & Inertia/Mass input//output  momentum/For Each Subsystem'
 * '<S255>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Vbxw/Subsystem'
 * '<S256>' : 'libsimodel/Model/Kinematics 6DOF Model/Custom Variable Mass 6DOF (Quaternion)/Vbxw/Subsystem1'
 * '<S257>' : 'libsimodel/Model/OutputHub/MavLinkGPSBus'
 * '<S258>' : 'libsimodel/Model/OutputHub/MavLinkSensorBus'
 * '<S259>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus'
 * '<S260>' : 'libsimodel/Model/OutputHub/MavLinkGPSBus/-pi-pi---->0-2pi'
 * '<S261>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions'
 * '<S262>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace'
 * '<S263>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Positive Trace'
 * '<S264>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM'
 * '<S265>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/trace(DCM)'
 * '<S266>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
 * '<S267>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
 * '<S268>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
 * '<S269>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
 * '<S270>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S271>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S272>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S273>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
 * '<S274>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
 * '<S275>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S276>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S277>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S278>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
 * '<S279>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
 * '<S280>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S281>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S282>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S283>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
 * '<S284>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
 * '<S285>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S286>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S287>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S288>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
 * '<S289>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S290>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
 * '<S291>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S292>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
 * '<S293>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S294>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S295>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S296>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S297>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S298>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S299>' : 'libsimodel/Model/OutputHub/MavVehile3DInfoBus/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 * '<S300>' : 'libsimodel/Model/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S301>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub'
 * '<S302>' : 'libsimodel/Model/Sensor Model/Barometer'
 * '<S303>' : 'libsimodel/Model/Sensor Model/GPS'
 * '<S304>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub'
 * '<S305>' : 'libsimodel/Model/Sensor Model/Magnetometer Hub'
 * '<S306>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0'
 * '<S307>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1'
 * '<S308>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/Dynamics'
 * '<S309>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)'
 * '<S310>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/wdot x d'
 * '<S311>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/Dynamics/No Dynamics'
 * '<S312>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/Dynamics/Second-order Dynamics'
 * '<S313>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x (w x d)'
 * '<S314>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x d'
 * '<S315>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x (w x d)/Subsystem'
 * '<S316>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x (w x d)/Subsystem1'
 * '<S317>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x d/Subsystem'
 * '<S318>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/w x (w x d)/w x d/Subsystem1'
 * '<S319>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/wdot x d/Subsystem'
 * '<S320>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer0/wdot x d/Subsystem1'
 * '<S321>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/Dynamics'
 * '<S322>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)'
 * '<S323>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/wdot x d'
 * '<S324>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/Dynamics/No Dynamics'
 * '<S325>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/Dynamics/Second-order Dynamics'
 * '<S326>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x (w x d)'
 * '<S327>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x d'
 * '<S328>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x (w x d)/Subsystem'
 * '<S329>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x (w x d)/Subsystem1'
 * '<S330>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x d/Subsystem'
 * '<S331>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/w x (w x d)/w x d/Subsystem1'
 * '<S332>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/wdot x d/Subsystem'
 * '<S333>' : 'libsimodel/Model/Sensor Model/Accelerometer Hub/Three-axis Accelerometer1/wdot x d/Subsystem1'
 * '<S334>' : 'libsimodel/Model/Sensor Model/Barometer/Dynamic Pressure'
 * '<S335>' : 'libsimodel/Model/Sensor Model/Barometer/Pressure Function'
 * '<S336>' : 'libsimodel/Model/Sensor Model/Barometer/Temperature Conversion'
 * '<S337>' : 'libsimodel/Model/Sensor Model/Barometer/Dynamic Pressure/dot'
 * '<S338>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA'
 * '<S339>' : 'libsimodel/Model/Sensor Model/GPS/GenCogVel'
 * '<S340>' : 'libsimodel/Model/Sensor Model/GPS/gps_location_noise'
 * '<S341>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap'
 * '<S342>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1'
 * '<S343>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset'
 * '<S344>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/pos_deg'
 * '<S345>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90'
 * '<S346>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Wrap Longitude'
 * '<S347>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Compare To Constant'
 * '<S348>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180'
 * '<S349>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S350>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap/Wrap Longitude/Compare To Constant'
 * '<S351>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90'
 * '<S352>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Wrap Longitude'
 * '<S353>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Compare To Constant'
 * '<S354>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180'
 * '<S355>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Latitude Wrap 90/Wrap Angle 180/Compare To Constant'
 * '<S356>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LatLong wrap1/Wrap Longitude/Compare To Constant'
 * '<S357>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/Find Radian//Distance'
 * '<S358>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/rotation_rad'
 * '<S359>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/Angle Conversion2'
 * '<S360>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/denom'
 * '<S361>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e'
 * '<S362>' : 'libsimodel/Model/Sensor Model/GPS/Flat Earth to LLA/LongLat_offset/Find Radian//Distance/e^4'
 * '<S363>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Acceleration Conversion'
 * '<S364>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope0'
 * '<S365>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope1'
 * '<S366>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope0/Dynamics'
 * '<S367>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope0/Dynamics/No Dynamics'
 * '<S368>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope0/Dynamics/Second-order Dynamics'
 * '<S369>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope1/Dynamics'
 * '<S370>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope1/Dynamics/No Dynamics'
 * '<S371>' : 'libsimodel/Model/Sensor Model/Gyroscope Hub/Three-axis Gyroscope1/Dynamics/Second-order Dynamics'
 * '<S372>' : 'libsimodel/Model/Sensor Model/Magnetometer Hub/MATLAB Function'
 * '<S373>' : 'libsimodel/Model/Sensor Model/Magnetometer Hub/MATLAB Function1'
 */
#endif                                 /* RTW_HEADER_libsimodel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
