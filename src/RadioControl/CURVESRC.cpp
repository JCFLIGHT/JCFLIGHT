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

#define THROTTLE_LOOKUP_LENGTH 11

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
