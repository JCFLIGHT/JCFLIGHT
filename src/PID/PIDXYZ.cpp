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
#include "GPS/GPSORIENTATION.h"
#include "FlightModes/FLIGHTMODES.h"
#include "AHRS/AHRS.h"
#include "AHRS/QUATERNION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDPARAMS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PIDXYZClass PIDXYZ;

static BiquadFilter_Struct Derivative_Roll_Smooth;
static BiquadFilter_Struct Derivative_Pitch_Smooth;
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

//FREQUENCIA DE CORTE DO LPF DO kCD
#define CONTROL_DERIVATIVE_CUTOFF 30 //Hz

//FREQUENCIA DE CORTE DO RELAXAMENTO DO TERMO INTEGRAL
#define INTEGRAL_TERM_RELAX_CUTOFF 15 //Hz

//FREQUENCIA DE CORTE DO GYRO APLICADO AO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_GYRO_CUTOFF 80 //Hz

//FREQUENCIA DE CORTE DA ACELERAÇÃO CALCULADA PELO DERIVATIVE BOOST
#define DERIVATIVE_BOOST_CUTOFF 10 //Hz

uint8_t IntegralTermWindUpPercent = 50;        //AJUSTAVEL PELO USUARIO -> (0 a 90)
int16_t ReferenceAirSpeed = 1000;              //AJUSTAVEL PELO USUARIO - VALOR DE 36KM/H CASO NÃO TENHA UM TUBO DE PITOT INSTALADO
int16_t FixedWingIntegralTermThrowLimit = 165; //AJUSTAVEL PELO USUARIO -> (0 a 500)
int16_t Cruise_Throttle = 1400;                //AJUSTAVEL PELO USUARIO -> (1000 a 2000)
int16_t MinThrottleDownPitchAngle = 0;         //AJUSTAVEL PELO USUARIO -> (0 a 450)
float PitchLevelTrim = 0;                      //AJUSTAVEL PELO USUARIO -> (-10 a +10)
float CoordinatedPitchGain = 1.0f;             //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float CoordinatedYawGain = 1.0f;               //AJUSTAVEL PELO USUARIO -> (0.0 a 2.0 (float))
float DerivativeBoostFactor = 1.25f;           //AJUSTAVEL PELO USUARIO -> (-1.0 a 3.0 (float))
float DerivativeBoostMaxAceleration = 7500.0f; //AJUSTAVEL PELO USUARIO -> (1000 a 16000)

///////////////////////////////////////////////////////////////

float FixedWingIntegralTermLimitOnStickPosition = 0.5f;
float MotorIntegralTermWindUpPoint;
float AntiWindUpScaler;
float CoordinatedTurnRateEarthFrame;
float ErrorGyroIntegral[3];
float ErrorGyroIntegralLimit[3];

void PIDXYZClass::Initialization()
{
  PIDXYZ.Get_LPF_Derivative_Value = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
  BIQUADFILTER.Settings(&Derivative_Roll_Smooth, PIDXYZ.Get_LPF_Derivative_Value, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Derivative_Pitch_Smooth, PIDXYZ.Get_LPF_Derivative_Value, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Roll_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Pitch_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&ControlDerivative_Yaw_Smooth, CONTROL_DERIVATIVE_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
#ifdef USE_DERIVATIVE_BOOST_PID
  BIQUADFILTER.Settings(&DerivativeBoost_Roll_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&DerivativeBoost_Pitch_Smooth, DERIVATIVE_BOOST_GYRO_CUTOFF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
#endif
  PT1FilterInit(&WindUpRollLPF, INTEGRAL_TERM_RELAX_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
  PT1FilterInit(&WindUpPitchLPF, INTEGRAL_TERM_RELAX_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
  MotorIntegralTermWindUpPoint = 1.0f - (IntegralTermWindUpPercent / 100.0f);
}

void PIDXYZClass::Update(float DeltaTime)
{
  PIDXYZ.CalcedRateTargetRoll = RCControllerToRate(RCController[ROLL], RCRate);
  PIDXYZ.CalcedRateTargetPitch = RCControllerToRate(RCController[PITCH], RCRate);
  PIDXYZ.CalcedRateTargetYaw = RCControllerToRate(RCController[YAW], YawRate);

  PIDXYZ.CalcedRateTargetRollToGCS = PIDXYZ.CalcedRateTargetRoll;
  PIDXYZ.CalcedRateTargetPitchToGCS = PIDXYZ.CalcedRateTargetPitch;
  PIDXYZ.CalcedRateTargetYawToGCS = PIDXYZ.CalcedRateTargetYaw;

  if (GetSafeStateOfHeadingHold())
  {
    CalcedRateTargetYaw = GetHeadingHoldValue(DeltaTime);
  }
  else
  {
    UpdateStateOfHeadingHold();
  }

  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    PIDXYZ.CalcedRateTargetRoll = PIDXYZ.PIDLevelRoll(DeltaTime);
    PIDXYZ.CalcedRateTargetPitch = PIDXYZ.PIDLevelPitch(DeltaTime);
  }

  if (GetFrameStateOfMultirotor())
  {
    AntiWindUpScaler = Constrain_Float((1.0f - GetMotorMixRange()) / MotorIntegralTermWindUpPoint, 0.0f, 1.0f);
    PIDXYZ.PIDApplyMulticopterRateControllerRoll(DeltaTime);
    PIDXYZ.PIDApplyMulticopterRateControllerPitch(DeltaTime);
    PIDXYZ.PIDApplyMulticopterRateControllerYaw(DeltaTime);
  }
  else if (GetFrameStateOfAirPlane())
  {
    PIDXYZ.GetNewControllerForPlaneWithTurn();
    PIDXYZ.PIDApplyFixedWingRateControllerRoll(DeltaTime);
    PIDXYZ.PIDApplyFixedWingRateControllerPitch(DeltaTime);
    PIDXYZ.PIDApplyFixedWingRateControllerYaw(DeltaTime);
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

float PIDXYZClass::DerivativeTermProcessRoll(float GyroDiffInput)
{
  float NewDTermCalced;

  if (PIDXYZ.Get_LPF_Derivative_Value > 0)
  {
    NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Roll_Smooth, GyroDiffInput);
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
    NewDTermCalced = BIQUADFILTER.ApplyAndGet(&Derivative_Pitch_Smooth, GyroDiffInput);
  }
  else
  {
    NewDTermCalced = GyroDiffInput;
  }
  return NewDTermCalced;
}

float PIDXYZClass::ApplyIntegralTermRelaxRoll(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpRollLPF, CurrentPIDSetpoint);
  const float SetPointHPF = fabsf(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermRelaxPitch(float CurrentPIDSetpoint, float IntegralTermErrorRate)
{
  const float SetPointLPF = PT1FilterApply3(&WindUpPitchLPF, CurrentPIDSetpoint);
  const float SetPointHPF = fabsf(CurrentPIDSetpoint - SetPointLPF);
  const float IntegralTermRelaxFactor = MAX(0, 1 - SetPointHPF / 40.0f);
  return IntegralTermErrorRate * IntegralTermRelaxFactor;
}

float PIDXYZClass::ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral)
{
  if ((MixerIsOutputSaturated() && GetFrameStateOfMultirotor()) ||
      (GetFrameStateOfAirPlane() && PIDXYZ.FixedWingIntegralTermLimitActive(Axis)))
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
    const float DerivativeBoostGyroAcceleration = fabsf(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Roll_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = fabsf((ActualRateTagert - PrevRateTagert) / DeltaTime);
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
    const float DerivativeBoostGyroAcceleration = fabsf(BIQUADFILTER.ApplyAndGet(&DerivativeBoost_Pitch_Smooth, DerivativeBoostGyroDelta));
    const float DerivativeBoostRateAcceleration = fabsf((ActualRateTagert - PrevRateTagert) / DeltaTime);
    const float Acceleration = MAX(DerivativeBoostGyroAcceleration, DerivativeBoostRateAcceleration);
    DerivativeBoost = ScaleRangeFloat(Acceleration, 0.0f, DerivativeBoostMaxAceleration, 1.0f, DerivativeBoostFactor);
    DerivativeBoost = PT1FilterApply(&DerivativeBoost_Pitch_LPF, DerivativeBoost, DERIVATIVE_BOOST_CUTOFF, DeltaTime);
    DerivativeBoost = Constrain_Float(DerivativeBoost, 1.0f, DerivativeBoostFactor);
  }

#endif

  return DerivativeBoost;
}

float PIDXYZClass::PIDLevelRoll(float DeltaTime)
{
  float RcControllerAngle = 0;

  if (GetFrameStateOfMultirotor())
  {
    if (IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
    {
      RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MinMaxValueVector));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValueVector));
    }
  }
  else
  {
    RcControllerAngle = RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValueVector));
  }

  const float AngleErrorInDegrees = ConvertDeciDegreesToDegrees((RcControllerAngle + GPS_Angle[ROLL]) - ATTITUDE.AngleOut[ROLL]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].ProportionalVector / 6.56f), -200, 200);

  if (GET_SET[PI_AUTO_LEVEL].IntegralVector > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Roll, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
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
      RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[ATTACK_BANK_MAX].MinMaxValueVector));
    }
    else
    {
      RcControllerAngle = RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValueVector));
    }
  }
  else
  {
    RcControllerAngle = RcControllerToAngleWithMinMax(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValueVector), ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MIN].MinMaxValueVector));
  }

  if (AirPlaneNavigationIsControllingThrottle())
  {
    RcControllerAngle += ScaleRange16Bits(MAX(0, Cruise_Throttle - RCController[THROTTLE]), 0, Cruise_Throttle - MIN_STICKS_PULSE, 0, MinThrottleDownPitchAngle);
  }

  if (GetFrameStateOfAirPlane())
  {
    RcControllerAngle -= ConvertDegreesToDecidegrees(PitchLevelTrim);
  }

  const float AngleErrorInDegrees = ConvertDeciDegreesToDegrees((RcControllerAngle + GPS_Angle[PITCH]) - ATTITUDE.AngleOut[PITCH]);

  float AngleRateTarget = Constrain_Float(AngleErrorInDegrees * (GET_SET[PI_AUTO_LEVEL].ProportionalVector / 6.56f), -200, 200);

  if (GET_SET[PI_AUTO_LEVEL].IntegralVector > 0)
  {
#ifndef __AVR_ATmega2560__
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, DeltaTime);
#else
    AngleRateTarget = PT1FilterApply(&Angle_Smooth_Pitch, AngleRateTarget, GET_SET[PI_AUTO_LEVEL].IntegralVector, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
#endif
  }
  return AngleRateTarget;
}

void PIDXYZClass::PIDApplyMulticopterRateControllerRoll(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetRoll - IMU.Gyroscope.Read[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetRoll - PreviousRateTarget;
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
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_ROLL].FeedForwardVector / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  float GyroDifference = PreviousRateGyro - IMU.Gyroscope.Read[ROLL];

  GyroDifference = PIDXYZ.DerivativeTermProcessRoll(GyroDifference);

  NewDerivativeTerm = GyroDifference * ((GET_SET[PID_ROLL].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostRoll(IMU.Gyroscope.Read[ROLL], PIDXYZ.CalcedRateTargetRoll, PreviousRateTarget, PreviousRateGyro, DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[ROLL] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxRoll(PIDXYZ.CalcedRateTargetRoll, RateError);

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

  PIDXYZ.PIDControllerApply[ROLL] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetRoll;
  PreviousRateGyro = IMU.Gyroscope.Read[ROLL];
}

void PIDXYZClass::PIDApplyMulticopterRateControllerPitch(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetPitch - IMU.Gyroscope.Read[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetPitch - PreviousRateTarget;
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
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_PITCH].FeedForwardVector / 7270.0f * TPA_Parameters.CalcedValue) / DeltaTime);
  }

  float GyroDifference = PreviousRateGyro - IMU.Gyroscope.Read[PITCH];

  GyroDifference = PIDXYZ.DerivativeTermProcessPitch(GyroDifference);

  NewDerivativeTerm = GyroDifference * ((GET_SET[PID_PITCH].DerivativeVector / 1905.0f * TPA_Parameters.CalcedValue) / DeltaTime) * PIDXYZ.ApplyDerivativeBoostPitch(IMU.Gyroscope.Read[PITCH], PreviousRateGyro, PIDXYZ.CalcedRateTargetPitch, PreviousRateTarget, DeltaTime);

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[PITCH] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);

  float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxPitch(PIDXYZ.CalcedRateTargetPitch, RateError);

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

  PIDXYZ.PIDControllerApply[PITCH] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetPitch;
  PreviousRateGyro = IMU.Gyroscope.Read[PITCH];
}

void PIDXYZClass::PIDApplyMulticopterRateControllerYaw(float DeltaTime)
{
  static float PreviousRateTarget;
  static float PreviousRateGyro;

  const float RateError = PIDXYZ.CalcedRateTargetYaw - IMU.Gyroscope.Read[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[YAW].ProportionalVector, RateError);

  const float RateTargetDelta = PIDXYZ.CalcedRateTargetYaw - PreviousRateTarget;
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
    NewControlDerivativeTerm = RateTargetDeltaFiltered * ((GET_SET[PID_YAW].FeedForwardVector / 7270.0f * 1.0f) / DeltaTime);
  }

  if (GET_SET[PID_YAW].DerivativeVector == 0)
  {
    NewDerivativeTerm = 0;
  }
  else
  {
    float GyroDifference = PreviousRateGyro - IMU.Gyroscope.Read[YAW];

    //GyroDifference = PIDXYZ.DerivativeTermProcessYaw(GyroDifference, DeltaTime);

    NewDerivativeTerm = GyroDifference * ((GET_SET[PID_YAW].DerivativeVector / 1905.0f * 1.0f) / DeltaTime) * 1;
  }

  const float NewOutput = NewProportionalTerm + NewDerivativeTerm + ErrorGyroIntegral[YAW] + NewControlDerivativeTerm;
  const float NewOutputLimited = Constrain_Float(NewOutput, -MAX_YAW_PID_SUM_LIMIT, +MAX_YAW_PID_SUM_LIMIT);

  //float IntegralTermErrorRate = PIDXYZ.ApplyIntegralTermRelaxYaw(PIDXYZ.CalcedRateTargetYaw, RateError);

  float IntegralTermErrorRate = RateError;

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

  PIDXYZ.PIDControllerApply[YAW] = NewOutputLimited;

  PreviousRateTarget = PIDXYZ.CalcedRateTargetYaw;
  PreviousRateGyro = IMU.Gyroscope.Read[YAW];
}

void PIDXYZClass::PIDApplyFixedWingRateControllerRoll(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetRoll - IMU.Gyroscope.Read[ROLL];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_ROLL].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetRoll * (GET_SET[PID_ROLL].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[ROLL] += RateError * (GET_SET[PID_ROLL].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[ROLL] = PIDXYZ.ApplyIntegralTermLimiting(ROLL, ErrorGyroIntegral[ROLL]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[ROLL] = Constrain_Float(ErrorGyroIntegral[ROLL], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDXYZ.PIDControllerApply[ROLL] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[ROLL], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

void PIDXYZClass::PIDApplyFixedWingRateControllerPitch(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetPitch - IMU.Gyroscope.Read[PITCH];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_PITCH].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetPitch * (GET_SET[PID_PITCH].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[PITCH] += RateError * (GET_SET[PID_PITCH].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[PITCH] = PIDXYZ.ApplyIntegralTermLimiting(PITCH, ErrorGyroIntegral[PITCH]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[PITCH] = Constrain_Float(ErrorGyroIntegral[PITCH], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDXYZ.PIDControllerApply[PITCH] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[PITCH], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

void PIDXYZClass::PIDApplyFixedWingRateControllerYaw(float DeltaTime)
{
  const float RateError = PIDXYZ.CalcedRateTargetYaw - IMU.Gyroscope.Read[YAW];
  const float NewProportionalTerm = PIDXYZ.ProportionalTermProcess(GET_SET[PID_YAW].ProportionalVector, RateError);
  const float NewFeedForwardTerm = PIDXYZ.CalcedRateTargetYaw * (GET_SET[PID_YAW].FeedForwardVector / 31.0f * TPA_Parameters.CalcedValue);

  ErrorGyroIntegral[YAW] += RateError * (GET_SET[PID_YAW].IntegralVector / 4.0f * TPA_Parameters.CalcedValue) * DeltaTime;

  ErrorGyroIntegral[YAW] = PIDXYZ.ApplyIntegralTermLimiting(YAW, ErrorGyroIntegral[YAW]);

  if (FixedWingIntegralTermThrowLimit != 0)
  {
    ErrorGyroIntegral[YAW] = Constrain_Float(ErrorGyroIntegral[YAW], -FixedWingIntegralTermThrowLimit, FixedWingIntegralTermThrowLimit);
  }

  PIDXYZ.PIDControllerApply[YAW] = Constrain_Float(NewProportionalTerm + NewFeedForwardTerm + ErrorGyroIntegral[YAW], -MAX_PID_SUM_LIMIT, +MAX_PID_SUM_LIMIT);
}

bool PIDXYZClass::FixedWingIntegralTermLimitActive(uint8_t Axis)
{
  float StickPosition = (float)Constrain_16Bits(DECODE.GetRxChannelOutput(Axis) - MIDDLE_STICKS_PULSE, -500, 500) / 500.0f;
  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
  {
    return false;
  }
  return fabsf(StickPosition) > FixedWingIntegralTermLimitOnStickPosition;
}

static void TransformVectorEarthFrameToBodyFrame(Struct_Vector3x3 *Vector)
{
  Vector->Pitch = -Vector->Pitch;
  QuaternionRotateVector(Vector, Vector, &Orientation);
}

void PIDXYZClass::GetNewControllerForPlaneWithTurn()
{
  Struct_Vector3x3 TurnControllerRates;
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
      float BankAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RCController[ROLL], ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValueVector)));
      float FinalBankAngleTarget = Constrain_Float(BankAngleTarget, -ConvertToRadians(60), ConvertToRadians(60));
      float PitchAngleTarget = ConvertDeciDegreesToRadians(RcControllerToAngle(RCController[PITCH], ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValueVector)));
      float TurnRatePitchAdjustmentFactor = Fast_Cosine(fabsf(PitchAngleTarget));
      CoordinatedTurnRateEarthFrame = ConvertToDegrees(980.665f * Fast_Tangent(-FinalBankAngleTarget) / AirSpeedForCoordinatedTurn * TurnRatePitchAdjustmentFactor);
      TurnControllerRates.Yaw = CoordinatedTurnRateEarthFrame;
    }
    else
    {
      return;
    }
  }

  //CONVERTE DE EARTH-FRAME PARA BODY-FRAME
  TransformVectorEarthFrameToBodyFrame(&TurnControllerRates);

  //LIMITA O VALOR MINIMO E MAXIMO DE SAÍDA A PARTIR DOS VALOR DE RATE DEFINIDO PELO USUARIO NO GCS
  PIDXYZ.CalcedRateTargetRoll = Constrain_16Bits(PIDXYZ.CalcedRateTargetRoll + TurnControllerRates.Roll, -ConvertDegreesToDecidegrees(RCRate), ConvertDegreesToDecidegrees(RCRate));
  PIDXYZ.CalcedRateTargetPitch = Constrain_16Bits(PIDXYZ.CalcedRateTargetPitch + TurnControllerRates.Pitch * CoordinatedPitchGain, -ConvertDegreesToDecidegrees(RCRate), ConvertDegreesToDecidegrees(RCRate));
  PIDXYZ.CalcedRateTargetYaw = Constrain_16Bits(PIDXYZ.CalcedRateTargetYaw + TurnControllerRates.Yaw * CoordinatedYawGain, -ConvertDegreesToDecidegrees(YawRate), ConvertDegreesToDecidegrees(YawRate));
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