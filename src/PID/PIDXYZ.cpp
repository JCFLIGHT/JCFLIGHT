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
#include "GPSNavigation/NAVIGATION.h"
#include "FlightModes/FLIGHTMODES.h"
#include "AHRS/AHRS.h"
#include "AHRS/QUATERNION.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDPARAMS.h"
#include "TECS/TECS.h"
#include "GPS/GPSSTATES.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PIDXYZClass PIDXYZ;
PID_Resources_Struct PID_Resources;

static BiquadFilter_Struct Derivative_Roll_Smooth;
static BiquadFilter_Struct Derivative_Pitch_Smooth;
static BiquadFilter_Struct Derivative_Yaw_Smooth;
static BiquadFilter_Struct ControlDerivative_Roll_Smooth;
static BiquadFilter_Struct ControlDerivative_Pitch_Smooth;
static BiquadFilter_Struct ControlDerivative_Yaw_Smooth;
#ifdef USE_DERIVATIVE_BOOST_PID
static BiquadFilter_Struct DerivativeBoost_Roll_Smooth;
static BiquadFilter_Struct DerivativeBoost_Pitch_Smooth;
#endif

PT1_Filter_Struct Angle_Smooth_Roll;
PT1_Filter_Struct Angle_Smooth_Pitch;
PT1_Filter_Struct WindUpRollLPF;
PT1_Filter_Struct WindUpPitchLPF;
#ifdef USE_DERIVATIVE_BOOST_PID
PT1_Filter_Struct DerivativeBoost_Roll_LPF;
PT1_Filter_Struct DerivativeBoost_Pitch_LPF;
#endif

//MIGRAR ESSES PARAMETROS PARA A LISTA COMPLETA DE PARAMETROS
/////////////////////////////////////////////////////////////////

//SAÍDA MAXIMA DE PITCH E ROLL
#define MAX_PID_SUM_LIMIT 500

//SAÍDA MAXIMA DO YAW
#define MAX_YAW_PID_SUM_LIMIT 350

//FREQUENCIA DE CORTE DO GYRO APLICADO AO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_GYRO_CUTOFF 80 //Hz

//FREQUENCIA DE CORTE DA ACELERAÇÃO CALCULADA PELO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_CUTOFF 10 //Hz

uint8_t IntegralTermWindUpPercent = 50;        //AJUSTAVEL PELO USUARIO -> (0 a 90)
int16_t ReferenceAirSpeed = 1000;              //AJUSTAVEL PELO USUARIO - VALOR DE 36KM/H CASO NÃO TENHA UM TUBO DE PITOT INSTALADO
int16_t FixedWingIntegralTermThrowLimit = 165; //AJUSTAVEL PELO USUARIO -> (0 a 500)
int16_t Cruise_Throttle = 1400;                //AJUSTAVEL PELO USUARIO -> (1000 a 2000)
int16_t MinThrottleDownPitchAngle = 0;         //AJUSTAVEL PELO USUARIO -> (0 a 450)
float CoordinatedPitchGain = 1.0f;             //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float CoordinatedYawGain = 1.0f;               //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float DerivativeBoostFactor = 1.25f;           //AJUSTAVEL PELO USUARIO -> (-1.0 a 3.0 (float))
float DerivativeBoostMaxAceleration = 7500.0f; //AJUSTAVEL PELO USUARIO -> (1000 a 16000)

///////////////////////////////////////////////////////////////

float PitchLevelTrim = 0;
float MotorIntegralTermWindUpPoint;
float AntiWindUpScaler;
float CoordinatedTurnRateEarthFrame;
float ErrorGyroIntegral[3];
float ErrorGyroIntegralLimit[3];

void PIDXYZClass::Initialization()
{
  PitchLevelTrim = STORAGEMANAGER.Read_16Bits(PITCH_LEVEL_TRIM_ADDR);
  PID_Resources.Filter.DerivativeCutOff = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
  PID_Resources.Filter.IntegralRelaxCutOff = STORAGEMANAGER.Read_16Bits(INTEGRAL_RELAX_LPF_ADDR);
  PID_Resources.Filter.ControlDerivativeCutOff = STORAGEMANAGER.Read_16Bits(KCD_OR_FF_LPF_ADDR);
  BIQUADFILTER.Settings(&Derivative_Roll_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Derivative_Pitch_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Derivative_Yaw_Smooth, PID_Resources.Filter.DerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Roll_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Pitch_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Yaw_Smooth, PID_Resources.Filter.ControlDerivativeCutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
#ifdef USE_DERIVATIVE_BOOST_PID
  BIQUADFILTER.Settings(&DerivativeBoost_Roll_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&DerivativeBoost_Pitch_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
#endif
  PT1FilterInit(&WindUpRollLPF, PID_Resources.Filter.IntegralRelaxCutOff, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
  PT1FilterInit(&WindUpPitchLPF, PID_Resources.Filter.IntegralRelaxCutOff, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
  MotorIntegralTermWindUpPoint = 1.0f - (IntegralTermWindUpPercent / 100.0f);
}

void PIDXYZClass::Update(float DeltaTime)
{
  PID_Resources.RcRateTarget.Roll = RCControllerToRate(RCController[ROLL], RCRate);
  PID_Resources.RcRateTarget.Pitch = RCControllerToRate(RCController[PITCH], RCRate);
  PID_Resources.RcRateTarget.Yaw = RCControllerToRate(RCController[YAW], YawRate);

  PID_Resources.RcRateTarget.GCS.Roll = PID_Resources.RcRateTarget.Roll;
  PID_Resources.RcRateTarget.GCS.Pitch = PID_Resources.RcRateTarget.Pitch;
  PID_Resources.RcRateTarget.GCS.Yaw = PID_Resources.RcRateTarget.Yaw;

  if (GetSafeStateOfHeadingHold())
  {
    PID_Resources.RcRateTarget.Yaw = GetHeadingHoldValue(DeltaTime);
  }
  else
  {
    UpdateStateOfHeadingHold();
  }

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    PID_Resources.RcRateTarget.Roll = PIDXYZ.LevelRoll(DeltaTime);
    PID_Resources.RcRateTarget.Pitch = PIDXYZ.LevelPitch(DeltaTime);
  }

  if (GetFrameStateOfMultirotor())
  {
    AntiWindUpScaler = Constrain_Float((1.0f - GetMotorMixRange()) / MotorIntegralTermWindUpPoint, 0.0f, 1.0f);
    PIDXYZ.ApplyMulticopterRateControllerRoll(DeltaTime);
    PIDXYZ.ApplyMulticopterRateControllerPitch(DeltaTime);
    PIDXYZ.ApplyMulticopterRateControllerYaw(DeltaTime);
  }
  else if (GetFrameStateOfAirPlane())
  {
    PIDXYZ.GetNewControllerForPlaneWithTurn();
    PIDXYZ.ApplyFixedWingRateControllerRoll(DeltaTime);
    PIDXYZ.ApplyFixedWingRateControllerPitch(DeltaTime);
    PIDXYZ.ApplyFixedWingRateControllerYaw(DeltaTime);
  }

  if (GetActualThrottleStatus(THROTTLE_LOW) || !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE) || !IS_FLIGHT_MODE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    PIDXYZ.Reset_Integral_Accumulators();
  }
}

float PIDXYZClass::ProportionalTermProcess(uint8_t kP, float RateError)
{
  const float NewPTermCalced = (kP / 31.0f * TPA_Parameters.CalcedValue) * RateError;
  return NewPTermCalced;
}

float PIDXYZClass::DerivativeTermProcessRoll(int16_t ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime)
{
  float NewDTermCalced;

  float GyroDifference = PreviousRateGyro - ActualGyro;

  if (PID_Resources.Filter.DerivativeCutOff > 0)
  {
    GyroDifference = BIQUADFILTER.ApplyAndGet(&Derivative_Roll_Smooth, GyroDifference);
  }

  NewDTermCalced = GyroDifference * ((GET_SET[PID_ROLL].kD / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostRoll(IMU.Gyroscope.Read[ROLL], PreviousRateGyro, PID_Resources.RcRateTarget.Roll, PreviousRateTarget, DeltaTime);

  return NewDTermCalced;
}

float PIDXYZClass::DerivativeTermProcessPitch(int16_t ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime)
{
  float NewDTermCalced;

  float GyroDifference = PreviousRateGyro - ActualGyro;

  if (PID_Resources.Filter.DerivativeCutOff > 0)
  {
    NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Pitch_Smooth, GyroDifference);
  }

  NewDTermCalced = GyroDifference * ((GET_SET[PID_PITCH].kD / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostPitch(IMU.Gyroscope.Read[PITCH], PreviousRateGyro, PID_Resources.RcRateTarget.Pitch, PreviousRateTarget, DeltaTime);

  return NewDTermCalced;
}

float PIDXYZClass::DerivativeTermProcessYaw(int16_t ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime)
{
  float NewDTermCalced;

  float GyroDifference = PreviousRateGyro - ActualGyro;

  if (PID_Resources.Filter.DerivativeCutOff > 0)
  {
    NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Yaw_Smooth, GyroDifference);
  }

  NewDTermCalced = GyroDifference * ((GET_SET[PID_YAW].kD / 1905.0f * 1.0f) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostPitch(IMU.Gyroscope.Read[YAW], PreviousRateGyro, PID_Resources.RcRateTarget.Yaw, PreviousRateTarget, DeltaTime);

  return NewDTermCalced;
}

float PIDXYZClass::ApplyIntegralTermRelaxRoll(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpRollLPF, CurrentPIDSetpoint);
  const float SetPointHPF = ABS(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermRelaxPitch(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpPitchLPF, CurrentPIDSetpoint);
  const float SetPointHPF = ABS(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral)
{
  if ((MixerIsOutputSaturated() && GetFrameStateOfMultirotor()) || (GetFrameStateOfAirPlane() && PIDXYZ.FixedWingIntegralTermLimitActive(Axis)))
  {
    ErrorGyroIntegral = Constrain_Float(ErrorGyroIntegral, -ErrorGyroIntegralLimit[Axis], ErrorGyroIntegralLimit[Axis]);
  }
  else
  {
    ErrorGyroIntegralLimit[Axis] = ABS(ErrorGyroIntegral);
  }
  return ErrorGyroIntegral;
}

float PIDXYZClass::ApplyDerivativeBoostRoll(int16_t ActualGyro, int16_t PrevGyro, int16_t ActualRateTagert, int16_t PrevRateTagert, float DeltaTime)
{
  float DerivativeBoost = 1.0f;

#ifdef USE_DERIVATIVE_BOOST_PID

  if (DerivativeBoostFactor > 1)
  {
    const float DerivativeBoostGyroDelta = (ActualGyro - PrevGyro) / DeltaTime;
    const float DerivativeBoostGyroAcceleration = ABS(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Roll_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = ABS((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_Roll_LPF, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::ApplyDerivativeBoostPitch(int16_t ActualGyro, int16_t PrevGyro, int16_t ActualRateTagert, int16_t PrevRateTagert, float DeltaTime)
{
  float DerivativeBoost = 1.0f;

#ifdef USE_DERIVATIVE_BOOST_PID

  if (DerivativeBoostFactor > 1)
  {
    const float DerivativeBoostGyroDelta = (ActualGyro - PrevGyro) / DeltaTime;
    const float DerivativeBoostGyroAcceleration = ABS(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Pitch_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = ABS((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_Pitch_LPF, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::LevelRoll(float DeltaTime)
{
  float RcControllerAngle = 0;

  RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[MAX_ROLL_LEVEL].MinMaxValue));

  float AngleErrorInDegrees = ConvertDeciDegreesToDegrees(RcControllerAngle - Attitude.Raw[ROLL]);

  if (Get_GPS_Flight_Modes_And_Navigation_In_Use())
  {
    AngleErrorInDegrees = GPS_Parameters.Navigation.AutoPilot.Control.Angle[ROLL];
  }

  int16_t ThisBankAngleMax = ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValue);

  if (GetFrameStateOfMultirotor() && IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
  {
    ThisBankAngleMax = ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MinMaxValue);
  }

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].kP / 6.56f), -ThisBankAngleMax, ThisBankAngleMax);

  if (GET_SET[PI_AUTO_LEVEL].kI > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
#endif
  }
  return AngleRateTarget;
}

float PIDXYZClass::LevelPitch(float DeltaTime)
{
  float RcControllerAngle = 0;

  RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[MAX_PITCH_LEVEL].MinMaxValue));

  RcControllerAngle += TECS.AutoPitchDown(Cruise_Throttle, MinThrottleDownPitchAngle);

  if (GetFrameStateOfAirPlane())
  {
    RcControllerAngle -= ConvertDegreesToDecidegrees(PitchLevelTrim);
  }

  float AngleErrorInDegrees = ConvertDeciDegreesToDegrees(RcControllerAngle - Attitude.Raw[PITCH]);

  if (Get_GPS_Flight_Modes_And_Navigation_In_Use())
  {
    AngleErrorInDegrees = GPS_Parameters.Navigation.AutoPilot.Control.Angle[PITCH];
  }

  int16_t ThisBankAngleMax = ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue);

  if (GetFrameStateOfMultirotor() && IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
  {
    ThisBankAngleMax = ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MinMaxValue);
  }

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].kP / 6.56f), -ThisBankAngleMax, ThisBankAngleMax);

  if (GET_SET[PI_AUTO_LEVEL].kI > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].kI, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
#endif
  }
  return AngleRateTarget;
}

void PIDXYZClass::ApplyMulticopterRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Roll - IMU.Gyroscope.Read[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].kP, RateError);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Roll - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Roll_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_ROLL].kFF / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  NewDerivativeTerm = PIDXYZ.DerivativeTermProcessRoll(IMU.Gyroscope.Read[ROLL], PreviousRateGyro, PreviousRateTarget, DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[ROLL] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxRoll(PID_Resources.RcRateTarget.Roll, RateError);

  if ((GET_SET[PID_ROLL].kP != 0) && (GET_SET[PID_ROLL].kI != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_ROLL].kP / 31.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_ROLL].kI / 4.0f * TPA_Parameters.CalcedValue)) +
                                 ((GET_SET[PID_ROLL].kD / 1905.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_ROLL].kP / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[ROLL] += (IntegralTermErrorRate * (GET_SET[PID_ROLL].kI / 4.0f * TPA_Parameters.CalcedValue) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, ErrorGyroIntegral[ROLL]);

  PID_Resources.Controller.Output.Calced[ROLL] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Roll;
  PreviousRateGyro = IMU.Gyroscope.Read[ROLL];
}

void PIDXYZClass::ApplyMulticopterRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Pitch - IMU.Gyroscope.Read[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].kP, RateError);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Pitch - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Pitch_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_PITCH].kFF / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  NewDerivativeTerm = PIDXYZ.DerivativeTermProcessPitch(IMU.Gyroscope.Read[PITCH], PreviousRateGyro, PreviousRateTarget, DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[PITCH] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxPitch(PID_Resources.RcRateTarget.Pitch, RateError);

  if ((GET_SET[PID_PITCH].kP != 0) && (GET_SET[PID_PITCH].kI != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_PITCH].kP / 31.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_PITCH].kI / 4.0f * TPA_Parameters.CalcedValue)) +
                                 ((GET_SET[PID_PITCH].kD / 1905.0f * TPA_Parameters.CalcedValue) /
                                  (GET_SET[PID_PITCH].kP / 31.0f * TPA_Parameters.CalcedValue)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[PITCH] += (IntegralTermErrorRate * (GET_SET[PID_PITCH].kI / 4.0f * TPA_Parameters.CalcedValue) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, ErrorGyroIntegral[PITCH]);

  PID_Resources.Controller.Output.Calced[PITCH] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Pitch;
  PreviousRateGyro = IMU.Gyroscope.Read[PITCH];
}

void PIDXYZClass::ApplyMulticopterRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Yaw - IMU.Gyroscope.Read[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[YAW].kP, RateError);

  const float RateTargetDelta = PID_Resources.RcRateTarget.Yaw - PreviousRateTarget;
  const float RateTargetDeltaFiltered = BIQUADFILTER.ApplyAndGet(&ControlDerivative_Yaw_Smooth, RateTargetDelta);

  float NewControlDerivativeTerm;
  float NewDerivativeTerm;
  float NewControlTracking;

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    NewControlDerivativeTerm = 0.0f;
  }
  else
  {
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_YAW].kFF / 7270.0f * 1.0f) / DeltaTime);
  }

  if (GET_SET[PID_YAW].kD == 0)
  {
    NewDerivativeTerm = 0;
  }
  else
  {
    NewDerivativeTerm = PIDXYZ.DerivativeTermProcessYaw(IMU.Gyroscope.Read[YAW], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  }

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[YAW] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_YAW_PID_SUM_LIMIT, +MAX_YAW_PID_SUM_LIMIT);

  //float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxYaw(PID_Resources.RcRateTarget.Yaw, RateError);

  float IntegralTermErrorRate = RateError;

  if ((GET_SET[PID_YAW].kP != 0) && (GET_SET[PID_YAW].kI != 0))
  {
    NewControlTracking = 2.0f / (((GET_SET[PID_YAW].kP / 31.0f * 1.0f) /
                                  (GET_SET[PID_YAW].kI / 4.0f * 1.0f)) +
                                 ((GET_SET[PID_YAW].kD / 1905.0f * 1.0f) /
                                  (GET_SET[PID_YAW].kP / 31.0f * 1.0f)));
  }
  else
  {
    NewControlTracking = 0;
  }

  ErrorGyroIntegral[YAW] += (IntegralTermErrorRate * (GET_SET[PID_YAW].kI / 4.0f * 1.0f) * AntiWindUpScaler * DeltaTime) + ((NewOutputLimited - NewOutput) * NewControlTracking * AntiWindUpScaler * DeltaTime);

  ErrorGyroIntegral[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, ErrorGyroIntegral[YAW]);

  PID_Resources.Controller.Output.Calced[YAW] = NewOutputLimited;

  PreviousRateTarget = PID_Resources.RcRateTarget.Yaw;
  PreviousRateGyro = IMU.Gyroscope.Read[YAW];
}

void PIDXYZClass::ApplyFixedWingRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Roll - IMU.Gyroscope.Read[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].kP, RateError);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessRoll(IMU.Gyroscope.Read[ROLL], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Roll * (GET_SET[PID_ROLL].kFF / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[ROLL] += RateError * (GET_SET[PID_ROLL].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, ErrorGyroIntegral[ROLL]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[ROLL] = Constrain_Float(ErrorGyroIntegral[ROLL], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[ROLL] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[ROLL] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Roll;
  PreviousRateGyro = IMU.Gyroscope.Read[ROLL];
}

void PIDXYZClass::ApplyFixedWingRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Pitch - IMU.Gyroscope.Read[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].kP, RateError);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessPitch(IMU.Gyroscope.Read[PITCH], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Pitch * (GET_SET[PID_PITCH].kFF / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[PITCH] += RateError * (GET_SET[PID_PITCH].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, ErrorGyroIntegral[PITCH]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[PITCH] = Constrain_Float(ErrorGyroIntegral[PITCH], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[PITCH] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[PITCH] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Pitch;
  PreviousRateGyro = IMU.Gyroscope.Read[PITCH];
}

void PIDXYZClass::ApplyFixedWingRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PID_Resources.RcRateTarget.Yaw - IMU.Gyroscope.Read[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_YAW].kP, RateError);
  const float NewDerivativeTerm = PIDXYZ.DerivativeTermProcessYaw(IMU.Gyroscope.Read[YAW], PreviousRateGyro, PreviousRateTarget, DeltaTime);
  const float NewFeedForwardTerm = PID_Resources.RcRateTarget.Yaw * (GET_SET[PID_YAW].kFF / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[YAW] += RateError * (GET_SET[PID_YAW].kI / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, ErrorGyroIntegral[YAW]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[YAW] = Constrain_Float(ErrorGyroIntegral[YAW], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PID_Resources.Controller.Output.Calced[YAW] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[YAW] + NewDerivativeTerm, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  PreviousRateTarget = PID_Resources.RcRateTarget.Yaw;
  PreviousRateGyro = IMU.Gyroscope.Read[YAW];
}

bool PIDXYZClass::FixedWingIntegralTermLimitActive(uint8_t Axis)
{
  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    return false;
  }
  float StickPosition = (float)Constrain_16Bits(DECODE.GetRxChannelOutput(Axis) - MIDDLE_STICKS_PULSE, -500, 500) / 500.0f;
  return ABS(StickPosition) > 0.5f; //MAIS DE 50% DE DEFLEXÃO
}

void PIDXYZClass::GetNewControllerForPlaneWithTurn()
{
  Vector3x3_Struct TurnControllerRates;
  TurnControllerRates.Roll = 0;
  TurnControllerRates.Pitch = 0;

  if (!IS_FLIGHT_MODE_ACTIVE(TURN_MODE))
  {
    return;
  }
  else
  {
    if (AHRS.CheckAnglesInclination(10)) //10 GRAUS DE INCLINAÇÃO
    {
      //SE O PITOT NÃO ESTIVER A BORDO,UTILIZE O VALOR PADRÃO DE 1000CM/S = 36KM/H
      int16_t AirSpeedForCoordinatedTurn = Get_AirSpeed_Enabled() ? AirSpeed.Raw.IASPressureInCM : ReferenceAirSpeed;
      //LIMITE DE 10KM/H - 216KM/H
      AirSpeedForCoordinatedTurn = Constrain_16Bits(AirSpeedForCoordinatedTurn, 300, 6000);
      float BankAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValue)));
      float FinalBankAngleTarget = Constrain_Float(BankAngleTarget, -ConvertToRadians(60), ConvertToRadians(60));
      float PitchAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue)));
      float TurnRatePitchAdjustmentFactor = Fast_Cosine(ABS(PitchAngleTarget));
      CoordinatedTurnRateEarthFrame = ConvertToDegrees(980.665f * Fast_Tangent(-FinalBankAngleTarget) / AirSpeedForCoordinatedTurn * TurnRatePitchAdjustmentFactor);
      TurnControllerRates.Yaw = CoordinatedTurnRateEarthFrame;
    }
    else
    {
      return;
    }
  }

  //CONVERTE DE EARTH-FRAME PARA BODY-FRAME
  AHRS.TransformVectorEarthFrameToBodyFrame(&TurnControllerRates);

  //LIMITA O VALOR MINIMO E MAXIMO DE SAÍDA A PARTIR DOS VALOR DE RATE DEFINIDO PELO USUARIO NO GCS
  PID_Resources.RcRateTarget.Roll = Constrain_16Bits(PID_Resources.RcRateTarget.Roll + TurnControllerRates.Roll, -ConvertDegreesToDecidegrees(RCRate), ConvertDegreesToDecidegrees(RCRate));
  PID_Resources.RcRateTarget.Pitch = Constrain_16Bits(PID_Resources.RcRateTarget.Pitch + TurnControllerRates.Pitch * CoordinatedPitchGain, -ConvertDegreesToDecidegrees(RCRate), ConvertDegreesToDecidegrees(RCRate));
  PID_Resources.RcRateTarget.Yaw = Constrain_16Bits(PID_Resources.RcRateTarget.Yaw + TurnControllerRates.Yaw * CoordinatedYawGain, -ConvertDegreesToDecidegrees(YawRate), ConvertDegreesToDecidegrees(YawRate));
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