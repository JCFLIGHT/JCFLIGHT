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
#include "PAA/FLIPMODE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/AVRLOWER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Yaw/YAWMANIPULATION.h"
#include "RadioControl/CURVESRC.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/PT1.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

static PT1_Filter_Struct DerivativeRollFilter;
static PT1_Filter_Struct DerivativePitchFilter;

//STABILIZE PARA MULTIROTORES
#define STAB_PITCH_ANGLE_MAX 45 //GRAUS
#define STAB_ROLL_ANGLE_MAX 45  //GRAUS

//STABILIZE PARA AEROS
#define STAB_PLANE_PITCH_ANGLE_MAX 35 //GRAUS
#define STAB_PLANE_ROLL_ANGLE_MAX 35  //GRAUS

//RATE MAXIMO DE SAÍDA DO PID YAW PARA AEROS E ASA-FIXA
#define YAW_RATE_MAX_FOR_PLANE 36 //GRAUS

//SPORT PARA MULTIROTORES
#define SPORT_PITCH_ANGLE_MAX 55 //GRAUS
#define SPORT_ROLL_ANGLE_MAX 55  //GRAUS

#define GYRO_SATURATION_LIMIT 1800 //DPS

//MIGRAR ESSE PARAMETRO PARA A LISTA COMPLETA DE PARAMETROS
int16_t ReferenceAirSpeed = 1000; //VALOR DE 36KM/H CASO NÃO TENHA UM TUBO DE PITOT INSTALADO

int16_t IntegralAccError[2] = {0, 0};
int16_t IntegralGyroError[2] = {0, 0};
int16_t Get_LPF_Derivative_Value = 0;
int16_t CalcedRateTargetRoll = 0;
int16_t CalcedRateTargetPitch = 0;
int16_t CalcedRateTargetYaw = 0;
int32_t IntegralGyroError_Yaw = 0;

void PID_DerivativeLPF_Update()
{
  Get_LPF_Derivative_Value = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
}

void PID_Update()
{
  CalcedRateTargetRoll = RCControllerToRate(RCController[ROLL], RCRate);
  CalcedRateTargetPitch = RCControllerToRate(RCController[PITCH], RCRate);
  if (GetSafeStateOfHeadingHold())
  {
    CalcedRateTargetYaw = GetHeadingHoldValue();
  }
  else
  {
    CalcedRateTargetYaw = RCControllerToRate(RCController[YAW], RCRate);
    UpdateStateOfHeadingHold();
  }
  CalcedRateTargetRoll = Constrain_16Bits(CalcedRateTargetRoll, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
  CalcedRateTargetPitch = Constrain_16Bits(CalcedRateTargetPitch, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
  CalcedRateTargetYaw = Constrain_16Bits(CalcedRateTargetYaw, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
  PID_Controll_Roll(CalcedRateTargetRoll);
  PID_Controll_Pitch(CalcedRateTargetPitch);
  PID_Controll_Yaw(CalcedRateTargetYaw);
}

void PID_Controll_Roll(int16_t RateTargetInput)
{
  int16_t ProportionalTerminate = 0;
  int16_t DerivativeTerminate;
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t IntegratorTerminate = 0;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t LastValueOfGyro = 0;
  int16_t RadioControlToPID;
  RadioControlToPID = RateTargetInput << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[ROLL]) * Loop_Integral_Time) >> 12);
  IntegralGyroError[ROLL] = Constrain_16Bits(IntegralGyroError[ROLL] + PIDError, -16000, +16000);
  if (ABS_16BITS(IMU.GyroscopeRead[ROLL]) > 640)
  {
    IntegralGyroError[ROLL] = 0;
  }
  IntegratorTerminate = (IntegralGyroError[ROLL] >> 7) * PID[ROLL].IntegratorVector >> 6;
  ProportionalTerminate = Multiplication32Bits(RadioControlToPID, PID[ROLL].ProportionalVector) >> 6;
  if (Do_Stabilize_Mode)
  {
    if (GetFrameStateOfMultirotor())
    {
      if (!SetFlightModes[ATACK_MODE])
      {
        if (!ApplyFlipRoll)
        {
          MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[ROLL], -STAB_ROLL_ANGLE_MAX * 10, +STAB_ROLL_ANGLE_MAX * 10) - ATTITUDE.AngleOut[ROLL];
        }
        else
        {
          MaxMinAngle = FlipAngleValue;
        }
      }
      else
      {
        MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[ROLL], -SPORT_ROLL_ANGLE_MAX * 10, +SPORT_ROLL_ANGLE_MAX * 10) - ATTITUDE.AngleOut[ROLL];
      }
    }
    else
    {
      MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[ROLL], -STAB_PLANE_ROLL_ANGLE_MAX * 10, +STAB_PLANE_ROLL_ANGLE_MAX * 10) - ATTITUDE.AngleOut[ROLL];
    }
    IntegralAccError[ROLL] = Constrain_16Bits(IntegralAccError[ROLL] + ((int16_t)(((int32_t)MaxMinAngle * Loop_Integral_Time) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_X = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_X, +Limit_Proportional_X);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[ROLL], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel + ((IntegratorTerminate - IntegratorTerminateLevel) * ValueOfFlipToRoll >> 9);
    ProportionalTerminate = ProportionalTerminateLevel + ((ProportionalTerminate - ProportionalTerminateLevel) * ValueOfFlipToRoll >> 9);
  }
  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[ROLL], DynamicProportionalVector[ROLL]) >> 6;
  DerivativeTerminate = IMU.GyroscopeRead[ROLL] - LastValueOfGyro;
  LastValueOfGyro = IMU.GyroscopeRead[ROLL];
  if (Get_LPF_Derivative_Value > 0)
  {
#ifndef __AVR_ATmega2560__
    DerivativeTerminate = PT1FilterApply(&DerivativeRollFilter, Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[ROLL]) >> 5, Get_LPF_Derivative_Value, Loop_Integral_Time * 1e-6);
#else
    DerivativeTerminate = PT1FilterApply(&DerivativeRollFilter, Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[ROLL]) >> 5, Get_LPF_Derivative_Value, 1.0f / 1000);
#endif
  }
  else
  {
    DerivativeTerminate = Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[ROLL]) >> 5;
  }
  PIDControllerApply[ROLL] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PID_Controll_Pitch(int16_t RateTargetInput)
{
  int16_t ProportionalTerminate = 0;
  int16_t DerivativeTerminate;
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t IntegratorTerminate = 0;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t RadioControlToPID;
  static int16_t LastValueOfGyro = 0;
  RadioControlToPID = RateTargetInput << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[PITCH]) * Loop_Integral_Time) >> 12);
  IntegralGyroError[PITCH] = Constrain_16Bits(IntegralGyroError[PITCH] + PIDError, -16000, +16000);
  if (ABS_16BITS(IMU.GyroscopeRead[PITCH]) > 640)
  {
    IntegralGyroError[PITCH] = 0;
  }
  IntegratorTerminate = (IntegralGyroError[PITCH] >> 7) * PID[PITCH].IntegratorVector >> 6;
  ProportionalTerminate = Multiplication32Bits(RadioControlToPID, PID[PITCH].ProportionalVector) >> 6;
  if (Do_Stabilize_Mode)
  {
    if (GetFrameStateOfMultirotor())
    {
      if (!SetFlightModes[ATACK_MODE])
      {
        if (!ApplyFlipPitch)
        {
          MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[PITCH], -STAB_PITCH_ANGLE_MAX * 10, +STAB_PITCH_ANGLE_MAX * 10) - ATTITUDE.AngleOut[PITCH];
        }
        else
        {
          MaxMinAngle = FlipAngleValue;
        }
      }
      else
      {
        MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[PITCH], -SPORT_PITCH_ANGLE_MAX * 10, +SPORT_PITCH_ANGLE_MAX * 10) - ATTITUDE.AngleOut[PITCH];
      }
    }
    else
    {
      MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[PITCH], -STAB_PLANE_PITCH_ANGLE_MAX * 10, +STAB_PLANE_PITCH_ANGLE_MAX * 10) - ATTITUDE.AngleOut[PITCH];
    }
    IntegralAccError[PITCH] = Constrain_16Bits(IntegralAccError[PITCH] + ((int16_t)(((int32_t)MaxMinAngle * Loop_Integral_Time) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_Y = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_Y, +Limit_Proportional_Y);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[PITCH], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel + ((IntegratorTerminate - IntegratorTerminateLevel) * ValueOfFlipToPitch >> 9);
    ProportionalTerminate = ProportionalTerminateLevel + ((ProportionalTerminate - ProportionalTerminateLevel) * ValueOfFlipToPitch >> 9);
  }
  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[PITCH], DynamicProportionalVector[PITCH]) >> 6;
  DerivativeTerminate = IMU.GyroscopeRead[PITCH] - LastValueOfGyro;
  LastValueOfGyro = IMU.GyroscopeRead[PITCH];
  if (Get_LPF_Derivative_Value > 0)
  {
#ifndef __AVR_ATmega2560__
    DerivativeTerminate = PT1FilterApply(&DerivativePitchFilter, Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[PITCH]) >> 5, Get_LPF_Derivative_Value, Loop_Integral_Time * 1e-6);
#else
    DerivativeTerminate = PT1FilterApply(&DerivativePitchFilter, Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[PITCH]) >> 5, Get_LPF_Derivative_Value, 1.0f / 1000);
#endif
  }
  else
  {
    DerivativeTerminate = Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[PITCH]) >> 5;
  }
  PIDControllerApply[PITCH] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PID_Controll_Yaw(int16_t RateTargetInput)
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
  if (!Do_IOC_Mode && GetFrameStateOfAirPlane()) //MODO MANUAL DESATIVADO E PERFIL DE AERO?SIM...
  {
    DeltaYawSmallFilter = IMU.GyroscopeRead[YAW] - LastGyroYawValue;
    DeltaYawSmallFilterStored = (DeltaYawSmallFilterStored >> 1) + (DeltaYawSmallFilter >> 1);
    LastGyroYawValue = IMU.GyroscopeRead[YAW];
    DerivativeTerminate = Multiplication32Bits(DeltaYawSmallFilterStored, PID[YAW].DerivativeVector) >> 6;
    DerivativeTerminate = Constrain_16Bits(DerivativeTerminate, -150, 150);
  }
  IntegralGyroError_Yaw += Multiplication32Bits((int16_t)(((int32_t)PIDError * Loop_Integral_Time) >> 12), PID[YAW].IntegratorVector);
  if (GetFrameStateOfAirPlane())
  {
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, -(((int32_t)IntegralGyroMax) << 13), (((int32_t)IntegralGyroMax) << 13));
  }
  else
  {
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
  }
  if (ABS_16BITS(RadioControlToPID) > 50)
  {
    IntegralGyroError_Yaw = 0;
  }
  ProportionalTerminate = Multiplication32Bits(PIDError, PID[YAW].ProportionalVector) >> 6;
  int16_t Limit_Proportional_Z = 300 - PID[YAW].DerivativeVector;
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

int16_t TurnControllerForAirPlane(int16_t RadioControlToTurn)
{
  static bool OkToTurnCoordination = false;
  if (!TurnCoordinatorMode)
  {
    return (RadioControlToTurn - IMU.GyroscopeRead[YAW]);
  }
  else
  {
    if (ABS_16BITS(ATTITUDE.AngleOut[ROLL]) > 100) //100 = 10 GRAUS DE INCLINAÇÃO
    {
      if (!OkToTurnCoordination)
      {
        IntegralGyroError_Yaw = 0;
        OkToTurnCoordination = true;
      }
      //SE O PITOT NÃO ESTIVER A BORDO,UTILIZE O VALOR PADRÃO DE 1000CM/S = 36KM/H
      int16_t AirSpeedForCoordinatedTurn = Get_AirSpeed_State() ? AirSpeedCalcedInCM : ReferenceAirSpeed;
      //10KM/H - 216KM/H
      AirSpeedForCoordinatedTurn = Constrain_16Bits(AirSpeedForCoordinatedTurn, 360, 6000);
      SlipAngleForAirPlane *= 980.665f / AirSpeedForCoordinatedTurn;
      return (RadioControlToTurn + SlipAngleForAirPlane);
    }
    else
    {
      if (OkToTurnCoordination)
      {
        IntegralGyroError_Yaw = 0;
        OkToTurnCoordination = false;
      }
      return (RadioControlToTurn - IMU.GyroscopeRead[YAW]);
    }
  }
}

void PID_Reset_Integral_Accumulators()
{
  if (RadioControllOutput[THROTTLE] <= 1100 && GetFrameStateOfMultirotor())
  {
    IntegralAccError[ROLL] = 0;
    IntegralAccError[PITCH] = 0;
    IntegralGyroError[ROLL] = 0;
    IntegralGyroError[PITCH] = 0;
    IntegralGyroError_Yaw = 0;
  }
}