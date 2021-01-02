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

#include "CURVESRC.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"

#define THROTTLE_LOOKUP_LENGTH 11
//lrint RETORNA UM VALOR ARREDONDADO DE UM NÚMERO (1.5 = 2 / 2.5 = 2)
#define LRint lrint

void CurvesRC_CalculeValue()
{
  int8_t NewValueCalculed;
  uint8_t ThrottleMiddlePoint;
  for (uint8_t IndexOfLookUpThrottle = 0; IndexOfLookUpThrottle < THROTTLE_LOOKUP_LENGTH; IndexOfLookUpThrottle++)
  {
    NewValueCalculed = 10 * IndexOfLookUpThrottle - ThrottleMiddle;
    ThrottleMiddlePoint = ThrottleMiddle;
    if (NewValueCalculed > 0)
      ThrottleMiddlePoint = 100 - ThrottleMiddlePoint;
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = 100 * ThrottleMiddle + NewValueCalculed * ((int32_t)ThrottleExpo * (NewValueCalculed * NewValueCalculed) /
                                                                                                  ((uint16_t)ThrottleMiddlePoint * ThrottleMiddlePoint) +
                                                                                              100 - ThrottleExpo);
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = MotorSpeed + (uint32_t)((uint16_t)(1900 - MotorSpeed)) * CalculeLookUpThrottle[IndexOfLookUpThrottle] / 10000;
  }
}

int16_t RCControllerToRate(int16_t StickData, uint8_t Rate)
{
  const int16_t MaximumRateDPS = Rate * 10;
  return Map_32Bits((int16_t)StickData, -500, 500, -MaximumRateDPS, MaximumRateDPS);
}

int16_t CalcedAttitudeRC(int16_t Data, int16_t RcExpo)
{
  int16_t RCValueDeflection;
  RCValueDeflection = Constrain_16Bits(RadioControllOutput[Data] - 1500, -500, 500);
  float ConvertValueToFloat = RCValueDeflection / 100.0f;
  return LRint((2500.0f + (float)RcExpo * (ConvertValueToFloat * ConvertValueToFloat - 25.0f)) * ConvertValueToFloat / 25.0f);
}

uint16_t CalcedLookupThrottle(uint16_t CalcedDeflection)
{
  if (CalcedDeflection > 999)
    return 1900;

  const uint8_t CalcedLookUpStep = CalcedDeflection / 100;
  return CalculeLookUpThrottle[CalcedLookUpStep] +
         (CalcedDeflection - CalcedLookUpStep * 100) *
             (CalculeLookUpThrottle[CalcedLookUpStep + 1] -
              CalculeLookUpThrottle[CalcedLookUpStep]) /
             100;
}

void CurvesRC_SetValues()
{
  RCRate = 70;
  RollAndPitchRate = 0;
  YawRate = 0;
  ThrottleMiddle = 50;
  ThrottleExpo = 35;
  if ((STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) < 3) ||
      (STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) == 6) ||
      (STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) == 7)) //MULTIROTORES
  {
    RCExpo = 60;
  }
  else //PARA PLANES
  {
    RCExpo = 0;
  }
}