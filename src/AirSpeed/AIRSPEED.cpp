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
#include "AIRSPEEDBACKEND.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Filters/PT1.h"
#include "FastSerial/PRINTF.h"

AirSpeedClass AIRSPEED;

PT1_Filter_Struct Pitot_Smooth;

#define AIR_DENSITY_SEA_LEVEL_15C 1.225f //DENSIDADE DO AR ACIMA DO NIVEL DO MAR COM A TEMPERATURA DE 15 GRAUS °C
#define PITOT_LPF_CUTOFF 350 / 1000.0f   //EM MILLIHZ

float AirSpeedRatio = 1.0f; //AJUSUTAVEL PELO USUARIO

void AirSpeedClass::Initialization()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  LOG("Tubo de Pitot em calib.Aguarde...");

  if (Get_AirSpeed_Type() == NONE_AIRSPEED)
  {
    LOG("Calib do Tubo de Pitot falhou!");
    LINE_SPACE;
    return;
  }

  PT1FilterInit(&Pitot_Smooth, PITOT_LPF_CUTOFF, 1000.0f * 1e-6f);

  AIRSPEED.Calibrate();

  LOG("Calib do Tubo de Pitot finalizada!");
  LINE_SPACE;
}

void AirSpeedClass::Calibrate()
{
  //FAZER O USO DA MATRIZ JACOBIANA AQUI FUTURAMENTE PARA A CALIBRAÇÃO DO TUBO DE PITOT
  //POR ENQUANTO UTILIZA A CALIBRAÇÃO POR MÉDIA (É...NÃO TEM UMA BOA PRECISÃO,MAS SERVE)
  for (uint8_t CountSamples = 0; CountSamples < 10; CountSamples++)
  {
    AIRSPEED.CalibrationSum += AIRSPEED.Get_Raw_Value();
    AIRSPEED.CalibrationSumCount++;
    SCHEDULERTIME.Sleep(100);
  }
  AIRSPEED.OffSetValue = AIRSPEED.CalibrationSum / AIRSPEED.CalibrationSumCount;
}

void AirSpeedClass::GetPressure(float &Pressure)
{
  //https://en.wikipedia.org/wiki/Indicated_airspeed
  Pressure = AirSpeedRatio * Fast_SquareRoot(2.0f * ABS(AIRSPEED.RawValue - AIRSPEED.OffSetValue) / AIR_DENSITY_SEA_LEVEL_15C);
  Pressure = PT1FilterApply3(&Pitot_Smooth, Pressure);
}

float AirSpeedClass::Get_Raw_Value()
{
  if (Get_AirSpeed_Type() == ANALOG_AIRSPEED)
  {
    return AirSpeed_Analog_Get_Actual_Value();
  }
  else if (Get_AirSpeed_Type() == I2C_AIRSPEED)
  {
    return AirSpeed_I2C_Get_Actual_Value();
  }
  return 0;
}

void AirSpeedClass::Update()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  if (Get_AirSpeed_Type() == NONE_AIRSPEED)
  {
    return;
  }

  AIRSPEED.RawValue = AIRSPEED.Get_Raw_Value();

  float Pressure = 0;
  AIRSPEED.GetPressure(Pressure);

  AIRSPEED.CalcedInCM = Pressure * 100; //EM CM/S
}
