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
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"
#include "PID/TPA.h"
#include "Common/RCDEFINES.h"
#include "PID/RCPID.h"

#define THROTTLE_LOOKUP_LENGTH 11

int16_t ThrottleRCMiddle = 0;
uint16_t CalculeLookUpThrottle[11];

void CurvesRC_SetValues()
{
  ThrottleMiddle = STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR);
  ThrottleExpo = STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR);
  RCRate = STORAGEMANAGER.Read_8Bits(RC_RATE_ADDR);
  RCExpo = STORAGEMANAGER.Read_8Bits(RC_EXPO_ADDR);
  YawRate = STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR);
  AttitudeThrottleMin = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MIN_ADDR);
  AttitudeThrottleMax = STORAGEMANAGER.Read_16Bits(THR_ATTITUDE_MAX_ADDR);
}

void CurvesRC_CalculeValue()
{
  if (ThrottleMiddle == 0)
  {
    return;
  }
  int8_t NewValueCalculed;
  uint8_t ThrottleMiddlePoint;
  ThrottleRCMiddle = AttitudeThrottleMin + (int32_t)(AttitudeThrottleMax - AttitudeThrottleMin) * ThrottleMiddle / 100;
  for (uint8_t IndexOfLookUpThrottle = 0; IndexOfLookUpThrottle < THROTTLE_LOOKUP_LENGTH; IndexOfLookUpThrottle++)
  {
    NewValueCalculed = 10 * IndexOfLookUpThrottle - ThrottleMiddle;
    ThrottleMiddlePoint = ThrottleMiddle;
    if (NewValueCalculed > 0)
    {
      ThrottleMiddlePoint = 100 - ThrottleMiddlePoint;
    }
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = 100 * ThrottleMiddle + NewValueCalculed * ((int32_t)ThrottleExpo * (NewValueCalculed * NewValueCalculed) / ((uint16_t)ThrottleMiddlePoint * ThrottleMiddlePoint) + 100 - ThrottleExpo);
    CalculeLookUpThrottle[IndexOfLookUpThrottle] = AttitudeThrottleMin + (uint32_t)((uint16_t)(AttitudeThrottleMax - AttitudeThrottleMin)) * CalculeLookUpThrottle[IndexOfLookUpThrottle] / 10000;
  }
}

int16_t RCControllerToRate(int16_t StickData, uint8_t Rate)
{
  const int16_t MaximumRateDPS = ConvertDegreesToDecidegrees(Rate);
  return ScaleRange32Bits((int16_t)StickData, -500, 500, -MaximumRateDPS, MaximumRateDPS);
}

int16_t CalcedAttitudeRC(int16_t Data, int16_t RcExpo)
{
  int16_t RCValueDeflection;
  RCValueDeflection = Constrain_16Bits(DECODE.GetRxChannelOutput(Data) - MIDDLE_STICKS_PULSE, -500, 500);
  float ConvertValueToFloat = RCValueDeflection / 100.0f;
  return lrint((2500.0f + (float)RcExpo * (ConvertValueToFloat * ConvertValueToFloat - 25.0f)) * ConvertValueToFloat / 25.0f);
}

float RcControllerToAngle(int16_t RcControllerInput, int16_t MaxInclination)
{
  RcControllerInput = Constrain_Float(RcControllerInput, -500, 500);
  return ScaleRangeFloat((float)RcControllerInput, -500.0f, 500.0f, (float)-MaxInclination, (float)MaxInclination);
}

float RcControllerToAngleWithMinMax(int16_t RcControllerInput, int16_t MinInclination, int16_t MaxInclination)
{
  RcControllerInput = Constrain_Float(RcControllerInput, -500, 500);
  return ScaleRangeFloat((float)RcControllerInput, -500.0f, 500.0f, (float)-MinInclination, (float)MaxInclination);
}

uint16_t CalcedLookupThrottle(uint16_t CalcedDeflection)
{
  if (CalcedDeflection > 999)
  {
    return AttitudeThrottleMax;
  }

  const uint8_t CalcedLookUpStep = CalcedDeflection / 100;
  return CalculeLookUpThrottle[CalcedLookUpStep] + (CalcedDeflection - CalcedLookUpStep * 100) * (CalculeLookUpThrottle[CalcedLookUpStep + 1] - CalculeLookUpThrottle[CalcedLookUpStep]) / 100;
}

int16_t RCLookupThrottleMiddle(void)
{
  return ThrottleRCMiddle;
}