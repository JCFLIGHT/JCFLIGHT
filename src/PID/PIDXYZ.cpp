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
#include "Filters/DERIVATIVELPF.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/AVRLOWER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"
#include "FastSerial/PRINTF.h"
#include "AHRS/AHRS.h"
#include "BAR/BAR.h"

//STABILIZE
#define STAB_PITCH_ANGLE_MAX 45 //GRAUS
#define STAB_ROLL_ANGLE_MAX 45  //GRAUS

//SPORT
#define SPORT_PITCH_ANGLE_MAX 55 //GRAUS
#define SPORT_ROLL_ANGLE_MAX 55  //GRAUS

int16_t IntegralAccError[2] = {0, 0};
int16_t IntegralGyroError[2] = {0, 0};
int16_t Value_LPF_Derivative = 0;
uint16_t PID_Integral_Time = 0;
int32_t IntegralGyroError_Yaw = 0;
uint32_t PID_Guard_Time = 0;

void DerivativeLPF_Initialization()
{
  Value_LPF_Derivative = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
}

void DerivativeLPF_Update()
{
  if (STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR) != Value_LPF_Derivative)
  {
    Value_LPF_Derivative = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
  }
}

void PID_Time()
{
  uint32_t PIDTimer = AVRTIME.SchedulerMicros();
  PID_Integral_Time = PIDTimer - PID_Guard_Time;
  PID_Guard_Time = PIDTimer;
}

void PID_Update()
{
  PID_Controll_Roll();
  PID_Controll_Pitch();
  PID_Controll_Yaw();
}

void PID_Controll_Roll()
{
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t SimpleFilterPID;
  static int16_t ProportionalTerminate = 0;
  static int16_t IntegratorTerminate = 0;
  static int16_t DerivativeTerminate;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t LastValueOfGyro = 0;
  static int16_t SimpleFilterPIDOne;
  static int16_t SimpleFilterPIDTwo;
  static int16_t SimpleFilterPIDThree;
  int16_t RadioControlToPID;
  RadioControlToPID = RCController[ROLL] << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[ROLL]) * PID_Integral_Time) >> 12);
  IntegralGyroError[ROLL] = Constrain_16Bits(IntegralGyroError[ROLL] + PIDError, -16000, +16000);
  if (ABS_16BITS(IMU.GyroscopeRead[ROLL]) > 640)
    IntegralGyroError[ROLL] = 0;
  IntegratorTerminate = (IntegralGyroError[ROLL] >> 7) * PID[ROLL].IntegratorVector >> 6;
  ProportionalTerminate = Multiplication32Bits(RadioControlToPID, PID[ROLL].ProportionalVector) >> 6;
  if (Stabilize_Mode)
  {
    if (!SetFlightModes[ATACK_MODE])
    {
      if (!ApplyFlipRoll)
        MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[ROLL], -STAB_ROLL_ANGLE_MAX * 10, +STAB_ROLL_ANGLE_MAX * 10) - ATTITUDE.AngleOut[ROLL];
      else
        MaxMinAngle = FlipAngleValue;
    }
    else
      MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[ROLL], -SPORT_ROLL_ANGLE_MAX * 10, +SPORT_ROLL_ANGLE_MAX * 10) - ATTITUDE.AngleOut[ROLL];
    IntegralAccError[ROLL] = Constrain_16Bits(IntegralAccError[ROLL] + ((int16_t)(((int32_t)MaxMinAngle * PID_Integral_Time) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_X = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_X, +Limit_Proportional_X);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[ROLL], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel + ((IntegratorTerminate - IntegratorTerminateLevel) * ValueOfFlipToRoll >> 9);
    ProportionalTerminate = ProportionalTerminateLevel + ((ProportionalTerminate - ProportionalTerminateLevel) * ValueOfFlipToRoll >> 9);
  }
  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[ROLL], DynamicProportionalVector[ROLL]) >> 6;
  SimpleFilterPID = IMU.GyroscopeRead[ROLL] - LastValueOfGyro;
  LastValueOfGyro = IMU.GyroscopeRead[ROLL];
  DerivativeTerminate = SimpleFilterPIDOne + SimpleFilterPIDTwo + SimpleFilterPID;
  DerivativeTerminate += SimpleFilterPIDThree;
  SimpleFilterPIDThree = SimpleFilterPIDTwo;
  SimpleFilterPIDTwo = SimpleFilterPIDOne;
  SimpleFilterPIDOne = SimpleFilterPID;
  if (Value_LPF_Derivative > 0)
  {
    DiscreteLPF_To_Derivative_PID(Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[ROLL]) >> 5, Value_LPF_Derivative);
  }
  else
  {
    DerivativeTerminate = Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[ROLL]) >> 5;
  }
  PIDControllerApply[ROLL] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PID_Controll_Pitch()
{
  static int16_t PIDError;
  static int16_t MaxMinAngle;
  static int16_t SimpleFilterPID;
  static int16_t ProportionalTerminate = 0;
  static int16_t IntegratorTerminate = 0;
  static int16_t DerivativeTerminate;
  static int16_t ProportionalTerminateLevel;
  static int16_t IntegratorTerminateLevel;
  static int16_t RadioControlToPID;
  static int16_t LastValueOfGyro = 0;
  static int16_t SimpleFilterPIDOne;
  static int16_t SimpleFilterPIDTwo;
  static int16_t SimpleFilterPIDThree;
  RadioControlToPID = RCController[PITCH] << 1;
  PIDError = (int16_t)(((int32_t)(RadioControlToPID - IMU.GyroscopeRead[PITCH]) * PID_Integral_Time) >> 12);
  IntegralGyroError[PITCH] = Constrain_16Bits(IntegralGyroError[PITCH] + PIDError, -16000, +16000);
  if (ABS_16BITS(IMU.GyroscopeRead[PITCH]) > 640)
    IntegralGyroError[PITCH] = 0;
  IntegratorTerminate = (IntegralGyroError[PITCH] >> 7) * PID[PITCH].IntegratorVector >> 6;
  ProportionalTerminate = Multiplication32Bits(RadioControlToPID, PID[PITCH].ProportionalVector) >> 6;
  if (Stabilize_Mode)
  {
    if (!SetFlightModes[ATACK_MODE])
    {
      if (!ApplyFlipPitch)
        MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[PITCH], -STAB_PITCH_ANGLE_MAX * 10, +STAB_PITCH_ANGLE_MAX * 10) - ATTITUDE.AngleOut[PITCH];
      else
        MaxMinAngle = FlipAngleValue;
    }
    else
      MaxMinAngle = Constrain_16Bits(RadioControlToPID + GPS_Angle[PITCH], -SPORT_PITCH_ANGLE_MAX * 10, +SPORT_PITCH_ANGLE_MAX * 10) - ATTITUDE.AngleOut[PITCH];
    IntegralAccError[PITCH] = Constrain_16Bits(IntegralAccError[PITCH] + ((int16_t)(((int32_t)MaxMinAngle * PID_Integral_Time) >> 12)), -10000, +10000);
    ProportionalTerminateLevel = Multiplication32Bits(MaxMinAngle, PID[PIDAUTOLEVEL].ProportionalVector) >> 7;
    int16_t Limit_Proportional_Y = PID[PIDAUTOLEVEL].DerivativeVector * 5;
    ProportionalTerminateLevel = Constrain_16Bits(ProportionalTerminateLevel, -Limit_Proportional_Y, +Limit_Proportional_Y);
    IntegratorTerminateLevel = Multiplication32Bits(IntegralAccError[PITCH], PID[PIDAUTOLEVEL].IntegratorVector) >> 12;
    IntegratorTerminate = IntegratorTerminateLevel + ((IntegratorTerminate - IntegratorTerminateLevel) * ValueOfFlipToPitch >> 9);
    ProportionalTerminate = ProportionalTerminateLevel + ((ProportionalTerminate - ProportionalTerminateLevel) * ValueOfFlipToPitch >> 9);
  }
  ProportionalTerminate -= Multiplication32Bits(IMU.GyroscopeRead[PITCH], DynamicProportionalVector[PITCH]) >> 6;
  SimpleFilterPID = IMU.GyroscopeRead[PITCH] - LastValueOfGyro;
  LastValueOfGyro = IMU.GyroscopeRead[PITCH];
  DerivativeTerminate = SimpleFilterPIDOne + SimpleFilterPIDTwo + SimpleFilterPID;
  DerivativeTerminate += SimpleFilterPIDThree;
  SimpleFilterPIDThree = SimpleFilterPIDTwo;
  SimpleFilterPIDTwo = SimpleFilterPIDOne;
  SimpleFilterPIDOne = SimpleFilterPID;
  if (Value_LPF_Derivative > 0)
  {
    DiscreteLPF_To_Derivative_PID(Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[PITCH]) >> 5, Value_LPF_Derivative);
  }
  else
  {
    DerivativeTerminate = Multiplication32Bits(DerivativeTerminate, DynamicDerivativeVector[PITCH]) >> 5;
  }
  PIDControllerApply[PITCH] = ProportionalTerminate + IntegratorTerminate - DerivativeTerminate;
}

void PID_Controll_Yaw()
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
  if ((FrameType == 3) ||
      (FrameType == 4) ||
      (FrameType == 5))
    IntegralGyroMax = 200;
  else
    IntegralGyroMax = 250;
  RadioControlToPID = Multiplication32Bits(RCController[YAW], (2 * YawRate + 30)) >> 5;
  PIDError = RadioControlToPID - IMU.GyroscopeRead[YAW];
  if ((FrameType == 3) ||
      (FrameType == 4) ||
      (FrameType == 5))
    PIDError = TurnControllerForAirPlane(RadioControlToPID);
  if (!IOCMODE) //STABILIZE OU ACRO
  {
    DeltaYawSmallFilter = IMU.GyroscopeRead[YAW] - LastGyroYawValue;
    DeltaYawSmallFilterStored = (DeltaYawSmallFilterStored >> 1) + (DeltaYawSmallFilter >> 1);
    LastGyroYawValue = IMU.GyroscopeRead[YAW];
    DerivativeTerminate = Multiplication32Bits(DeltaYawSmallFilterStored, PID[YAW].DerivativeVector) >> 6;
    DerivativeTerminate = Constrain_16Bits(DerivativeTerminate, -150, 150);
  }
  IntegralGyroError_Yaw += Multiplication32Bits((int16_t)(((int32_t)PIDError * PID_Integral_Time) >> 12), PID[YAW].IntegratorVector);
  if ((FrameType == 3) ||
      (FrameType == 4) ||
      (FrameType == 5))
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, -(((int32_t)IntegralGyroMax) << 13), (((int32_t)IntegralGyroMax) << 13));
  else
    IntegralGyroError_Yaw = Constrain_32Bits(IntegralGyroError_Yaw, 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
  if (ABS_16BITS(RadioControlToPID) > 50)
    IntegralGyroError_Yaw = 0;
  ProportionalTerminate = Multiplication32Bits(PIDError, PID[YAW].ProportionalVector) >> 6;
  int16_t Limit_Proportional_Z = 300 - PID[YAW].DerivativeVector;
  ProportionalTerminate = Constrain_16Bits(ProportionalTerminate, -Limit_Proportional_Z, +Limit_Proportional_Z);
  if ((FrameType == 3) ||
      (FrameType == 4) ||
      (FrameType == 5))
    IntegratorTerminate = constrain((int16_t)(IntegralGyroError_Yaw >> 13), -IntegralGyroMax, +IntegralGyroMax);
  else
    IntegratorTerminate = (IntegralGyroError_Yaw >> 13);
  if ((FrameType == 3) ||
      (FrameType == 4) ||
      (FrameType == 5))
    PIDControllerApply[YAW] = Constrain_16Bits(ProportionalTerminate + IntegratorTerminate - DerivativeTerminate, -500, 500);
  else
    PIDControllerApply[YAW] = ProportionalTerminate + IntegratorTerminate;
}

int16_t TurnControllerForAirPlane(int16_t RadioControlToTurn)
{
  static bool OkToTurnCoordination = false;
  if (!TurnCoordinatorMode)
  {
    return (RadioControlToTurn + (SlipAngleForAirPlane >> 1));
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

void PID_Reset_Accumulators()
{
  if (Reset_I || (RadioControllOutput[THROTTLE] <= 1100 && (FrameType < 3 || FrameType == 6 || FrameType == 7)))
  {
    IntegralGyroError[ROLL] = 0;
    IntegralGyroError[PITCH] = 0;
    IntegralAccError[ROLL] = 0;
    IntegralAccError[PITCH] = 0;
    IntegralGyroError_Yaw = 0;
  }
}