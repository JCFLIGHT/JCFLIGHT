/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#include "PIDXYZ.h"
#include "Common/VARIABLES.h"
#include "RCPID.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Yaw/HEADINGHOLD.h"
#include "RadioControl/CURVESRC.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/BIQUADFILTER.h"
#include "Filters/PT1.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "RadioControl/RCSTATES.h"
#include "MotorsControl/THRCLIPPING.h"
#include "TPA.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PIDXYZClass PIDXYZ;

static BiquadFilter_Struct Derivative_Roll_Smooth;
static BiquadFilter_Struct Derivative_Pitch_Smooth;
static BiquadFilter_Struct ControlDerivative_Roll_Smooth;
static BiquadFilter_Struct ControlDerivative_Pitch_Smooth;
static BiquadFilter_Struct ControlDerivative_Yaw_Smooth;
PT1_Filter_Struct Angle_Smooth_Roll;
PT1_Filter_Struct Angle_Smooth_Pitch;

//STABILIZE PARA MULTIROTORES
#define STAB_COPTER_PITCH_ANGLE_MAX 30 //GRAUS
#define STAB_COPTER_ROLL_ANGLE_MAX 30  //GRAUS

//SPORT PARA MULTIROTORES
#define ATTACK_COPTER_PITCH_ANGLE_MAX 40 //GRAUS
#define ATTACK_COPTER_ROLL_ANGLE_MAX 40  //GRAUS

//STABILIZE PARA AEROS
#define STAB_PLANE_PITCH_ANGLE_MAX 35 //GRAUS
#define STAB_PLANE_ROLL_ANGLE_MAX 35  //GRAUS

//RATE MAXIMO DE SAÍDA DO PID YAW PARA AEROS E ASA-FIXA
#define YAW_RATE_MAX_FOR_PLANE 36 //GRAUS

//SAÍDA MAXIMA DE PITCH E ROLL
#define MAX_PID_SUM_LIMIT 500

//SAÍDA MAXIMA DO YAW
#define MAX_YAW_PID_SUM_LIMIT 350

//FREQUENCIA DE CORTE DO LPF DO kCD
#define CONTROL_DERIVATIVE_CUTOFF 30 //Hz

//MIGRAR ESSE PARAMETRO PARA A LISTA COMPLETA DE PARAMETROS
int16_t ReferenceAirSpeed = 1000; //VALOR DE 36KM/H CASO NÃO TENHA UM TUBO DE PITOT INSTALADO

int16_t FixedWingIntegralTermThrowLimit = 165; //AJUSTAVEL PELO USUARIO

float FixedWingIntegralTermLimitOnStickPosition = 0.5f; //AJUSTAVEL PELO USUARIO

float ErrorGyroIntegral[3];
float ErrorGyroIntegralLimit[3];

void PIDXYZClass::DerivativeLPF_Update()
{
  Get_LPF_Derivative_Value = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
  BIQUADFILTER.Settings(&Derivative_Roll_Smooth, Get_LPF_Derivative_Value, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&Derivative_Pitch_Smooth, Get_LPF_Derivative_Value, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Roll_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Pitch_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Yaw_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
}

void PIDXYZClass::Update(float DeltaTime)
{
  CalcedRateTargetRoll = RCControllerToRate(RCController[ROLL], RCRate);
  CalcedRateTargetPitch = RCControllerToRate(RCController[PITCH], RCRate);
  CalcedRateTargetYaw = RCControllerToRate(RCController[YAW], RCRate);

  CalcedRateTargetRollToGCS = CalcedRateTargetRoll;
  CalcedRateTargetPitchToGCS = CalcedRateTargetPitch;
  CalcedRateTargetYawToGCS = CalcedRateTargetYaw;

  if (GetSafeStateOfHeadingHold())
  {
    CalcedRateTargetYaw = GetHeadingHoldValue(DeltaTime);
  }
  else
  {
    UpdateStateOfHeadingHold();
  }

  if (Do_Stabilize_Mode)
  {
    CalcedRateTargetRoll = PIDLevelRoll(DeltaTime);
    CalcedRateTargetPitch = PIDLevelPitch(DeltaTime);
  }

  CalcedRateTargetYaw = PIDXYZ.GetNewYawControllerForPlane(CalcedRateTargetYaw);

  if (GetFrameStateOfMultirotor())
  {
    PIDApplyMulticopterRateControllerRoll(DeltaTime);
    PIDApplyMulticopterRateControllerPitch(DeltaTime);
    PIDApplyMulticopterRateControllerYaw(DeltaTime);
  }
  else if (GetFrameStateOfAirPlane())
  {
    PIDApplyFixedWingRateControllerRoll(DeltaTime);
    PIDApplyFixedWingRateControllerPitch(DeltaTime);
    PIDApplyFixedWingRateControllerYaw(DeltaTime);
  }

  if (GetActualThrottleStatus(THROTTLE_LOW) ||
      !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE) /*|| !IS_FLIGHT_MODE_ACTIVE(PRIMARY_ARM_DISARM)*/)
  {
    Reset_Integral_Accumulators();
  }
}

float PIDXYZClass::ProportionalTermProcess(uint8_t kP, float RateError)
{
  const float NewPTermCalced = (kP / 31.0f * TPA_Parameters.CalcedValue) * RateError;
  return NewPTermCalced;
}

float PIDXYZClass::DerivativeTermProcessRoll(float GyroDiffInput)
{
  float NewDTermCalced;

  if (PIDXYZ.Get_LPF_Derivative_Value > 0)
  {
    NewDTermCalced = BIQUADFILTER.FilterApplyAndGet(&Derivative_Roll_Smooth, GyroDiffInput);
  }
  else
  {
    NewDTermCalced = GyroDiffInput;
  }
  return NewDTermCalced;
}

float PIDXYZClass::DerivativeTermProcessPitch(float GyroDiffInput)
{
  float NewDTermCalced;

  if (PIDXYZ.Get_LPF_Derivative_Value > 0)
  {
    NewDTermCalced = BIQUADFILTER.FilterApplyAndGet(&Derivative_Pitch_Smooth, GyroDiffInput);
  }
  else
  {
    NewDTermCalced = GyroDiffInput;
  }
  return NewDTermCalced;
}

float PIDXYZClass::ApplyIntegralTermRelax(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  float NewIntegralCalced = IntegralTermErrorRate;
  //CODIGO
  return NewIntegralCalced;
}

float PIDXYZClass::ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral)
{
  if ((MixerIsOutputSaturated() && GetFrameStateOfMultirotor()) ||
      (GetFrameStateOfAirPlane() && FixedWingIntegralTermLimitActive(Axis)))
  {
    ErrorGyroIntegral = Constrain_Float(ErrorGyroIntegral, -ErrorGyroIntegralLimit[Axis], ErrorGyroIntegralLimit[Axis]);
  }
  else
  {
    ErrorGyroIntegralLimit[Axis] = ABS(ErrorGyroIntegral);
  }
  return ErrorGyroIntegral;
}

float PIDXYZClass::PIDLevelRoll(float DeltaTime)
{
  float RcControllerAngle = 0;

  if (GetFrameStateOfMultirotor())
  {
    if (IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
    {
      RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(ATTACK_COPTER_ROLL_ANGLE_MAX));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(STAB_COPTER_ROLL_ANGLE_MAX));
    }
  }
  else
  {
    RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(STAB_PLANE_ROLL_ANGLE_MAX));
  }

  const float AngleErrorInDegrees = ConvertDeciDegreesToDegrees((RcControllerAngle + GPS_Angle[ROLL]) - ATTITUDE.AngleOut[ROLL]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].ProportionalVector / 6.56f), -500, 500);

  if (GET_SET[PI_AUTO_LEVEL].IntegralVector > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, 1.0f / 1000.0f);
#endif
  }
  return AngleRateTarget;
}

float PIDXYZClass::PIDLevelPitch(float DeltaTime)
{
  float RcControllerAngle = 0;

  if (GetFrameStateOfMultirotor())
  {
    if (IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
    {
      RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(ATTACK_COPTER_PITCH_ANGLE_MAX));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(STAB_COPTER_PITCH_ANGLE_MAX));
    }
  }
  else
  {
    RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(STAB_PLANE_PITCH_ANGLE_MAX));
  }

  const float AngleErrorInDegrees = ConvertDeciDegreesToDegrees((RcControllerAngle + GPS_Angle[PITCH]) - ATTITUDE.AngleOut[PITCH]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].ProportionalVector / 6.56f), -500, 500);

  if (GET_SET[PI_AUTO_LEVEL].IntegralVector > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, 1.0f / 1000.0f);
#endif
  }
  return AngleRateTarget;
}

void PIDXYZClass::PIDApplyMulticopterRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetRoll - IMU.GyroscopeRead[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetRoll - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.FilterApplyAndGet(&ControlDerivative_Roll_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (Do_Stabilize_Mode)
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_ROLL].FeedForwardVector / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  float GyroDifference = PreviousRateGyro - IMU.GyroscopeRead[ROLL];

  GyroDifference = PIDXYZ.DerivativeTermProcessRoll(GyroDifference);

  NewDerivativeTerm = GyroDifference * ((GET_SET[PID_ROLL].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * 1;

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[ROLL] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelax(PIDXYZ.CalcedRateTargetRoll, RateError);
  float AntiWindUpScaler = 1;

  if ((GET_SET[PID_ROLL].ProportionalVector != 0) && (GET_SET[PID_ROLL].IntegralVector != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_ROLL].ProportionalVector / 31.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_ROLL].IntegralVector / 4.0f * TPA_Parameters.CalcedValue)) +
                                 ((GET_SET[PID_ROLL].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_ROLL].ProportionalVector / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[ROLL] += (IntegralTermErrorRate * (GET_SET[PID_ROLL].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, ErrorGyroIntegral[ROLL]);

  PIDControllerApply[ROLL] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetRoll;
  PreviousRateGyro = IMU.GyroscopeRead[ROLL];
}

void PIDXYZClass::PIDApplyMulticopterRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetPitch - IMU.GyroscopeRead[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetPitch - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.FilterApplyAndGet(&ControlDerivative_Pitch_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (Do_Stabilize_Mode)
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_PITCH].FeedForwardVector / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  float GyroDifference = PreviousRateGyro - IMU.GyroscopeRead[PITCH];

  GyroDifference = PIDXYZ.DerivativeTermProcessPitch(GyroDifference);

  NewDerivativeTerm = GyroDifference * ((GET_SET[PID_PITCH].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * 1;

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[PITCH] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelax(PIDXYZ.CalcedRateTargetPitch, RateError);
  float AntiWindUpScaler = 1;

  if ((GET_SET[PID_PITCH].ProportionalVector != 0) && (GET_SET[PID_PITCH].IntegralVector != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_PITCH].ProportionalVector / 31.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_PITCH].IntegralVector / 4.0f * TPA_Parameters.CalcedValue)) +
                                 ((GET_SET[PID_PITCH].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_PITCH].ProportionalVector / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[PITCH] += (IntegralTermErrorRate * (GET_SET[PID_PITCH].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, ErrorGyroIntegral[PITCH]);

  PIDControllerApply[PITCH] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetPitch;
  PreviousRateGyro = IMU.GyroscopeRead[PITCH];
}

void PIDXYZClass::PIDApplyMulticopterRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetYaw - IMU.GyroscopeRead[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[YAW].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetYaw - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.FilterApplyAndGet(&ControlDerivative_Yaw_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (Do_Stabilize_Mode)
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_YAW].FeedForwardVector / 7270.0f * 1.0f) / DeltaTime);
  }

  if (GET_SET[PID_YAW].DerivativeVector == 0)
  {
    NewDerivativeTerm = 0;
  }
  else
  {
    float GyroDifference = PreviousRateGyro - IMU.GyroscopeRead[YAW];

    //GyroDifference = PIDXYZ.DerivativeTermProcessYaw(GyroDifference, DeltaTime);

    NewDerivativeTerm = GyroDifference * ((GET_SET[PID_YAW].DerivativeVector / 1905.0f * 1.0f) / DeltaTime) * 1;
  }

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[YAW] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_YAW_PID_SUM_LIMIT, +MAX_YAW_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelax(PIDXYZ.CalcedRateTargetYaw, RateError);
  float AntiWindUpScaler = 1;

  if ((GET_SET[PID_YAW].ProportionalVector != 0) && (GET_SET[PID_YAW].IntegralVector != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_YAW].ProportionalVector / 31.0f * 1.0f) /
                                  (GET_SET[PID_YAW].IntegralVector / 4.0f * 1.0f)) +
                                 ((GET_SET[PID_YAW].DerivativeVector / 1905.0f * 1.0f) /
                                  (GET_SET[PID_YAW].ProportionalVector / 31.0f * 1.0f)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[YAW] += (IntegralTermErrorRate * (GET_SET[PID_YAW].IntegralVector / 4.0f * 1.0f) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, ErrorGyroIntegral[YAW]);

  PIDControllerApply[YAW] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetYaw;
  PreviousRateGyro = IMU.GyroscopeRead[YAW];
}

void PIDXYZClass::PIDApplyFixedWingRateControllerRoll(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetRoll - IMU.GyroscopeRead[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetRoll * (GET_SET[PID_ROLL].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[ROLL] += RateError * (GET_SET[PID_ROLL].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, ErrorGyroIntegral[ROLL]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[ROLL] = Constrain_Float(ErrorGyroIntegral[ROLL], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDControllerApply[ROLL] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[ROLL], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

void PIDXYZClass::PIDApplyFixedWingRateControllerPitch(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetPitch - IMU.GyroscopeRead[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetPitch * (GET_SET[PID_PITCH].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[PITCH] += RateError * (GET_SET[PID_PITCH].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, ErrorGyroIntegral[PITCH]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[PITCH] = Constrain_Float(ErrorGyroIntegral[PITCH], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDControllerApply[PITCH] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[PITCH], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

void PIDXYZClass::PIDApplyFixedWingRateControllerYaw(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetYaw - IMU.GyroscopeRead[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_YAW].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetYaw * (GET_SET[PID_YAW].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[YAW] += RateError * (GET_SET[PID_YAW].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, ErrorGyroIntegral[YAW]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[YAW] = Constrain_Float(ErrorGyroIntegral[YAW], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDControllerApply[YAW] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[YAW], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

bool PIDXYZClass::FixedWingIntegralTermLimitActive(uint8_t Axis)
{
  float StickPosition = (float)Constrain_16Bits(RadioControllOutput[Axis] - MIDDLE_STICKS_PULSE, -500, 500) / 500.0f;
  if (Do_Stabilize_Mode)
  {
    return false;
  }
  return fabsf(StickPosition) > FixedWingIntegralTermLimitOnStickPosition;
}

int16_t PIDXYZClass::GetNewYawControllerForPlane(int16_t RateTargetInput)
{
  if (GetFrameStateOfAirPlane())
  {
    return TurnControllerForAirPlane(RateTargetInput);
  }
  return RateTargetInput;
}

int16_t PIDXYZClass::TurnControllerForAirPlane(int16_t RadioControlToTurn)
{
  if (!IS_FLIGHT_MODE_ACTIVE(TURN_MODE))
  {
    return (RadioControlToTurn - IMU.GyroscopeRead[YAW]);
  }
  else
  {
    if (AHRS.CheckAnglesInclination(10)) //10 GRAUS DE INCLINAÇÃO
    {
      //SE O PITOT NÃO ESTIVER A BORDO,UTILIZE O VALOR PADRÃO DE 1000CM/S = 36KM/H
      int16_t AirSpeedForCoordinatedTurn = Get_AirSpeed_State() ? AIRSPEED.CalcedInCM : ReferenceAirSpeed;
      //10KM/H - 216KM/H
      AirSpeedForCoordinatedTurn = Constrain_16Bits(AirSpeedForCoordinatedTurn, 300, 6000);
      CoordinatedTurnRateEarthFrame = ConvetToDegrees(980.665f * Fast_Tangent(-ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL])) / AirSpeedForCoordinatedTurn);
      return (RadioControlToTurn + CoordinatedTurnRateEarthFrame);
    }
    else
    {
      return (RadioControlToTurn - IMU.GyroscopeRead[YAW]);
    }
  }
}

void PIDXYZClass::Reset_Integral_Accumulators()
{
  ErrorGyroIntegral[ROLL] = 0;
  ErrorGyroIntegral[PITCH] = 0;
  ErrorGyroIntegral[YAW] = 0;
  ErrorGyroIntegralLimit[ROLL] = 0;
  ErrorGyroIntegralLimit[PITCH] = 0;
  ErrorGyroIntegralLimit[YAW] = 0;
}