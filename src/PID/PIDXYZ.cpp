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
#include "DYNAMICPID.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/AVRLOWER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Yaw/HEADINGHOLD.h"
#include "RadioControl/CURVESRC.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/PT1.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "RadioControl/RCSTATES.h"
#include "MotorsControl/THRCLIPPING.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PIDXYZClass PIDXYZ;

PT1_Filter_Struct DerivativeRollFilter;
PT1_Filter_Struct DerivativePitchFilter;

//STABILIZE PARA MULTIROTORES
#define STAB_COPTER_PITCH_ANGLE_MAX 30 //GRAUS
#define STAB_COPTER_ROLL_ANGLE_MAX 30  //GRAUS

//SPORT PARA MULTIROTORES
#define SPORT_PITCH_ANGLE_MAX 40 //GRAUS
#define SPORT_ROLL_ANGLE_MAX 40  //GRAUS

//STABILIZE PARA AEROS
#define STAB_PLANE_PITCH_ANGLE_MAX 35 //GRAUS
#define STAB_PLANE_ROLL_ANGLE_MAX 35  //GRAUS

//RATE MAXIMO DE SAÍDA DO PID YAW PARA AEROS E ASA-FIXA
#define YAW_RATE_MAX_FOR_PLANE 36 //GRAUS

//DPS MAXIMO NO GYRO (PITCH E ROLL)
#define MAX_GYRO_SATURATION 640

//MIGRAR ESSE PARAMETRO PARA A LISTA COMPLETA DE PARAMETROS
int16_t ReferenceAirSpeed = 1000; //VALOR DE 36KM/H CASO NÃO TENHA UM TUBO DE PITOT INSTALADO

void PIDXYZClass::DerivativeLPF_Update()
{
  Get_LPF_Derivative_Value = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
}

void PIDXYZClass::Update(int32_t DeltaTimeUs)
{
  CalcedRateTargetRoll = RCControllerToRate(RCController[ROLL], RCRate);
  CalcedRateTargetPitch = RCControllerToRate(RCController[PITCH], RCRate);
  if (GetSafeStateOfHeadingHold())
  {
    CalcedRateTargetYaw = GetHeadingHoldValue(DeltaTimeUs);
  }
  else
  {
    CalcedRateTargetYaw = RCControllerToRate(RCController[YAW], RCRate);
    UpdateStateOfHeadingHold();
  }
  Controll_Roll(CalcedRateTargetRoll, DeltaTimeUs);
  Controll_Pitch(CalcedRateTargetPitch, DeltaTimeUs);
  Controll_Yaw(CalcedRateTargetYaw, DeltaTimeUs);
  Reset_Integral_Accumulators();
}

int16_t PIDXYZClass::AngleTarget(int16_t RcControllerInput, uint8_t AttitudeAngle, int16_t MaxInclination)
{
  return Constrain_16Bits(RcControllerInput + GPS_Angle[AttitudeAngle], -ConvertDegreesToDecidegrees(MaxInclination), ConvertDegreesToDecidegrees(MaxInclination)) - ATTITUDE.AngleOut[AttitudeAngle];
}

int32_t PIDXYZClass::ProportionalTermProcess(int16_t RcRateError, uint8_t kP)
{
  int32_t NewPTermCalced = Multiplication32Bits(RcRateError, kP);
  return NewPTermCalced;
}

int32_t PIDXYZClass::DerivativeTermProcessRoll(int16_t ActualGyro, int16_t LastGyro, int16_t DynamicDerivative, int32_t DeltaTimeUs)
{
  int32_t NewDTermCalced = ActualGyro - LastGyro;

  if (PIDXYZ.Get_LPF_Derivative_Value > 0)
  {
#ifndef __AVR_ATmega2560__
    NewDTermCalced = PT1FilterApply(&DerivativeRollFilter, Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5, Get_LPF_Derivative_Value, DeltaTimeUs * 1e-6f);
#else
    NewDTermCalced = PT1FilterApply(&DerivativeRollFilter, Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5, Get_LPF_Derivative_Value, 1.0f / 1000);
#endif
  }
  else
  {
    NewDTermCalced = Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5;
  }

  return NewDTermCalced;
}

int32_t PIDXYZClass::DerivativeTermProcessPitch(int16_t ActualGyro, int16_t LastGyro, int16_t DynamicDerivative, int32_t DeltaTimeUs)
{
  int32_t NewDTermCalced = ActualGyro - LastGyro;

  if (PIDXYZ.Get_LPF_Derivative_Value > 0)
  {
#ifndef __AVR_ATmega2560__
    NewDTermCalced = PT1FilterApply(&DerivativePitchFilter, Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5, Get_LPF_Derivative_Value, DeltaTimeUs * 1e-6f);
#else
    NewDTermCalced = PT1FilterApply(&DerivativePitchFilter, Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5, Get_LPF_Derivative_Value, 1.0f / 1000);
#endif
  }
  else
  {
    NewDTermCalced = Multiplication32Bits(NewDTermCalced, DynamicDerivative) >> 5;
  }

  return NewDTermCalced;
}

int16_t PIDXYZClass::ApplyIntegralTermLimiting(int16_t IntegratorTerminate)
{
  static int16_t IntegratorTerminateLimit = 0;
  if (MixerIsOutputSaturated())
  {
    IntegratorTerminate = Constrain_16Bits(IntegratorTerminate, -IntegratorTerminateLimit, IntegratorTerminateLimit);
  }
  else
  {
    IntegratorTerminateLimit = ABS(IntegratorTerminate);
  }
  return IntegratorTerminate;
}

bool PIDXYZClass::GetGyroSaturation(uint8_t GyroAxis, int16_t SaturationValue)
{
  return (ABS(IMU.GyroscopeRead[GyroAxis]) > SaturationValue);
}

void PIDXYZClass::Controll_Roll(int16_t RateTargetInput, int32_t DeltaTimeUs)
{
  int16_t RadioControlToPID;
  int16_t ProportionalTerminate = 0;
  int16_t DerivativeTerminate;
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t IntegratorTerminate = 0;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t LastValueOfGyro = 0;
  RadioControlToPID = RateTargetInput << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[ROLL]) * DeltaTimeUs) >> 12);
  ProportionalTerminate = ProportionalTermProcess(RadioControlToPID, PID[PIDROLL].ProportionalVector) >> 6;
  IntegralGyroError[ROLL] = Constrain_16Bits(IntegralGyroError[ROLL] + PIDError, -16000, +16000);

  if (GetGyroSaturation(ROLL, MAX_GYRO_SATURATION))
  {
    IntegralGyroError[ROLL] = 0;
  }

  IntegratorTerminate = (IntegralGyroError[ROLL] >> 7) * PID[PIDROLL].IntegratorVector >> 6;
  IntegratorTerminate = ApplyIntegralTermLimiting(IntegratorTerminate);

  if (Do_Stabilize_Mode)
  {
    if (GetFrameStateOfMultirotor())
    {
      if (IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
      {
        MaxMinAngle = AngleTarget(CalcedRateTargetRoll, ROLL, SPORT_ROLL_ANGLE_MAX);
      }
      else
      {
        MaxMinAngle = AngleTarget(CalcedRateTargetRoll, ROLL, STAB_COPTER_ROLL_ANGLE_MAX);
      }
    }
    else
    {
      MaxMinAngle = AngleTarget(CalcedRateTargetRoll, ROLL, STAB_PLANE_ROLL_ANGLE_MAX);
    }
    IntegralAccError[ROLL] = Constrain_16Bits(IntegralAccError[ROLL] + ((int16_t)(((int32_t)MaxMinAngle * DeltaTimeUs) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_X = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_X, +Limit_Proportional_X);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[ROLL], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel;
    ProportionalTerminate = ProportionalTerminateLevel;
  }

  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[ROLL], DynamicProportionalVector[ROLL]) >> 6;
  DerivativeTerminate = DerivativeTermProcessRoll(IMU.GyroscopeRead[ROLL], LastValueOfGyro, DynamicDerivativeVector[ROLL], DeltaTimeUs);
  LastValueOfGyro = IMU.GyroscopeRead[ROLL];
  PIDControllerApply[ROLL] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PIDXYZClass::Controll_Pitch(int16_t RateTargetInput, int32_t DeltaTimeUs)
{
  int16_t RadioControlToPID;
  int16_t ProportionalTerminate = 0;
  int16_t DerivativeTerminate;
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t IntegratorTerminate = 0;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t LastValueOfGyro = 0;
  RadioControlToPID = RateTargetInput << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[PITCH]) * DeltaTimeUs) >> 12);
  ProportionalTerminate = ProportionalTermProcess(RadioControlToPID, PID[PIDPITCH].ProportionalVector) >> 6;
  IntegralGyroError[PITCH] = Constrain_16Bits(IntegralGyroError[PITCH] + PIDError, -16000, +16000);

  if (GetGyroSaturation(PITCH, MAX_GYRO_SATURATION))
  {
    IntegralGyroError[PITCH] = 0;
  }

  IntegratorTerminate = (IntegralGyroError[PITCH] >> 7) * PID[PIDPITCH].IntegratorVector >> 6;
  IntegratorTerminate = ApplyIntegralTermLimiting(IntegratorTerminate);

  if (Do_Stabilize_Mode)
  {
    if (GetFrameStateOfMultirotor())
    {
      if (IS_FLIGHT_MODE_ACTIVE(ATTACK_MODE))
      {
        MaxMinAngle = AngleTarget(CalcedRateTargetPitch, PITCH, SPORT_PITCH_ANGLE_MAX);
      }
      else
      {
        MaxMinAngle = AngleTarget(CalcedRateTargetPitch, PITCH, STAB_COPTER_PITCH_ANGLE_MAX);
      }
    }
    else
    {
      MaxMinAngle = AngleTarget(CalcedRateTargetPitch, PITCH, STAB_PLANE_PITCH_ANGLE_MAX);
    }
    IntegralAccError[PITCH] = Constrain_16Bits(IntegralAccError[PITCH] + ((int16_t)(((int32_t)MaxMinAngle * DeltaTimeUs) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_Y = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_Y, +Limit_Proportional_Y);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[PITCH], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel;
    ProportionalTerminate = ProportionalTerminateLevel;
  }

  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[PITCH], DynamicProportionalVector[PITCH]) >> 6;
  DerivativeTerminate = DerivativeTermProcessPitch(IMU.GyroscopeRead[PITCH], LastValueOfGyro, DynamicDerivativeVector[PITCH], DeltaTimeUs);
  LastValueOfGyro = IMU.GyroscopeRead[PITCH];
  PIDControllerApply[PITCH] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PIDXYZClass::Controll_Yaw(int16_t RateTargetInput, int32_t DeltaTimeUs)
{
  static uint8_t IntegralGyroMax = 250;
  int16_t RadioControlToPID;
  int16_t PIDError;
  int16_t DeltaYawSmallFilter;
  int16_t ProportionalTerminate = 0;
  int16_t IntegratorTerminate = 0;
  int16_t DerivativeTerminate = 0;
  static int16_t LastGyroYawValue = 0;
  static int16_t DeltaYawSmallFilterStored = 0;
  if (GetFrameStateOfAirPlane())
  {
    IntegralGyroMax = 200;
  }
  else
  {
    IntegralGyroMax = 250;
  }
  RadioControlToPID = Multiplication32Bits(RateTargetInput, (2 * YawRate + 30)) >> 5;
  if (GetFrameStateOfAirPlane())
  {
    PIDError = TurnControllerForAirPlane(RadioControlToPID);
  }
  else
  {
    PIDError = RadioControlToPID - IMU.GyroscopeRead[YAW];
  }
  if (!IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE) && GetFrameStateOfAirPlane()) //MODO MANUAL DESATIVADO E PERFIL DE AERO?SIM...
  {
    DeltaYawSmallFilter = IMU.GyroscopeRead[YAW] - LastGyroYawValue;
    DeltaYawSmallFilterStored = (DeltaYawSmallFilterStored >> 1) + (DeltaYawSmallFilter >> 1);
    LastGyroYawValue = IMU.GyroscopeRead[YAW];
    DerivativeTerminate = Multiplication32Bits(DeltaYawSmallFilterStored, PID[PIDYAW].DerivativeVector) >> 6;
    DerivativeTerminate = Constrain_16Bits(DerivativeTerminate, -150, 150);
  }
  IntegralGyroError_Yaw += Multiplication32Bits((int16_t)(((int32_t)PIDError * DeltaTimeUs) >> 12), PID[PIDYAW].IntegratorVector);
  if (GetFrameStateOfAirPlane())
  {
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, -(((int32_t)IntegralGyroMax) << 13), (((int32_t)IntegralGyroMax) << 13));
  }
  else
  {
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, -268435454, 268435454);
  }
  if (ABS(RadioControlToPID) > 50)
  {
    IntegralGyroError_Yaw = 0;
  }
  ProportionalTerminate = Multiplication32Bits(PIDError, PID[PIDYAW].ProportionalVector) >> 6;
  int16_t Limit_Proportional_Z = 300 - PID[PIDYAW].DerivativeVector;
  ProportionalTerminate = Constrain_16Bits(ProportionalTerminate, -Limit_Proportional_Z, +Limit_Proportional_Z);
  if (GetFrameStateOfAirPlane())
  {
    IntegratorTerminate = constrain((int16_t)(IntegralGyroError_Yaw >> 13), -IntegralGyroMax, +IntegralGyroMax);
  }
  else
  {
    IntegratorTerminate = (IntegralGyroError_Yaw >> 13);
  }
  if (GetFrameStateOfAirPlane())
  {
    PIDControllerApply[YAW] = Constrain_16Bits(ProportionalTerminate + IntegratorTerminate - DerivativeTerminate, -YAW_RATE_MAX_FOR_PLANE * 10, +YAW_RATE_MAX_FOR_PLANE * 10);
  }
  else
  {
    PIDControllerApply[YAW] = ProportionalTerminate + IntegratorTerminate;
  }
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
  if (GetActualThrottleStatus(THROTTLE_LOW))
  {
    IntegralAccError[ROLL] = 0;
    IntegralAccError[PITCH] = 0;
    IntegralGyroError[ROLL] = 0;
    IntegralGyroError[PITCH] = 0;
    IntegralGyroError_Yaw = 0;
  }
}