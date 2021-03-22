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

#include "AIRSPEED.h"
#include "Math/MATHSUPPORT.h"
#include "AIRSPEEDANALOG.h"
#include "AIRSPEEDI2C.h"
#include "AIRSPEEDVIRTUAL.h"
#include "AIRSPEEDBACKEND.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Filters/PT1.h"

AirSpeedClass AIRSPEED;

PT1_Filter_Struct Pitot_Smooth;

#define AIR_DENSITY_SEA_LEVEL_15C 1.225f //DENSIDADE DO AR ACIMA DO NIVEL DO MAR COM A TEMPERATURA DE 15 GRAUS °C
#define PITOT_LPF_CUTOFF 350 / 1000.0f   //EM MILLIHZ

float AirSpeedRatio = 1.0f; //AJUSUTAVEL PELO USUARIO

void AirSpeedClass::Initialization()
{
  if (GetFrameStateOfMultirotor() || Get_AirSpeed_Type() == AIR_SPEED_DISABLED)
  {
    return;
  }

  AirSpeed_State.Healthy = true;

  PT1FilterInit(&Pitot_Smooth, PITOT_LPF_CUTOFF, 1000.0f * 1e-6f);
}

bool AirSpeedClass::Calibrate()
{
  if (!AirSpeed_State.Calibration.Initialized)
  {
    AirSpeed_State.Calibration.Start_MS = SCHEDULERTIME.GetMillis();
    AirSpeed_State.Calibration.Initialized = true;
  }

  if (AirSpeed_State.Calibration.Start_MS == 0)
  {
    return true;
  }

  if (SCHEDULERTIME.GetMillis() - AirSpeed_State.Calibration.Start_MS >= 1000 && AirSpeed_State.Calibration.Read_Count > 15)
  {
    if (AirSpeed_State.Calibration.Count > 0)
    {
      AirSpeed_State.OffSetValue = AirSpeed_State.Calibration.Sum / AirSpeed_State.Calibration.Count;
    }
    AirSpeed_State.Calibration.Start_MS = 0;
    return false;
  }
  //DESCARTA AS 5 PRIMEIRAS AMOSTRAS
  if (AirSpeed_State.Calibration.Read_Count > 5)
  {
    AirSpeed_State.Calibration.Sum += AirSpeed_State.RawValue;
    AirSpeed_State.Calibration.Count++;
  }
  AirSpeed_State.Calibration.Read_Count++;
  return false;
}

float AirSpeedClass::Get_Raw_Pressure()
{
  float RetValue = 0;

  switch (Get_AirSpeed_Type())
  {
  case ANALOG_AIR_SPEED:
    RetValue = AirSpeed_Analog_Get_Actual_Value();
    break;

  case DIGITAL_AIR_SPEED:
    RetValue = AirSpeed_I2C_Get_Actual_Value();
    break;

  case VIRTUAL_AIR_SPEED:
    RetValue = AirSpeed_Virtual_Get_Actual_Value();
    break;
  }

  return RetValue;
}

void AirSpeedClass::Get_Bernoulli_IAS_Pressure(float &Pressure)
{
  //https://en.wikipedia.org/wiki/Indicated_airspeed
  Pressure = AirSpeedRatio * Fast_SquareRoot(2.0f * ABS(AirSpeed_State.RawValue - AirSpeed_State.OffSetValue) / AIR_DENSITY_SEA_LEVEL_15C);
  Pressure = PT1FilterApply3(&Pitot_Smooth, Pressure);
}

void AirSpeedClass::Update()
{
  if (!AirSpeed_State.Healthy)
  {
    return;
  }

  AirSpeed_State.RawValue = AIRSPEED.Get_Raw_Pressure();

  if (!AIRSPEED.Calibrate())
  {
    return;
  }

  float Pressure = 0;
  AIRSPEED.Get_Bernoulli_IAS_Pressure(Pressure);

  AIRSPEED.CalcedInCM = Pressure * 100; //EM CM/S
}