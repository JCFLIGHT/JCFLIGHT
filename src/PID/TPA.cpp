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

#include "TPA.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"

TPA_Parameters_Struct TPA_Parameters;

void TPA_Initialization()
{
  if (STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR) < 1000)
  {
    STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, 1000); //AQUI NÃO PODE SER MENOR QUE 1000
  }
  TPA_Parameters.TPABreakPointer = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
  TPA_Parameters.TPAThrottlePercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR);
}

void TPA_Update()
{
  TPA_Parameters.TPABreakPointer = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
  //PARA AEROS E ASA-FIXA,ESSE PARAMETRO EM 90% FUNCIONA BEM
  TPA_Parameters.TPAThrottlePercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR);
}

int16_t GetThrottleIdleValue(void)
{
  if (!TPA_Parameters.ThrottleIdleValue)
  {
    TPA_Parameters.ThrottleIdleValue = MotorSpeed + (((TPA_Parameters.MaxThrottle - MotorSpeed) / 100.0f) * TPA_Parameters.ThrottleIdleFactor);
  }
  return TPA_Parameters.ThrottleIdleValue;
}

uint8_t CalculateFixedWingTPAFactor(int16_t Throttle)
{
  TPA_Update();
  float TPAFactor;
  if (TPA_Parameters.TPAThrottlePercent != 0 && GetThrottleIdleValue() < TPA_Parameters.TPABreakPointer)
  {
    if (Throttle > GetThrottleIdleValue())
    {
      TPAFactor = 0.5f + ((float)(TPA_Parameters.TPABreakPointer - GetThrottleIdleValue()) / (Throttle - GetThrottleIdleValue()) / 2.0f);
      TPAFactor = Constrain_Float(TPAFactor, 0.5f, 2.0f);
    }
    else
    {
      TPAFactor = 2.0f;
    }
    TPAFactor = 1.0f + (TPAFactor - 1.0f) * (TPA_Parameters.TPAThrottlePercent / 100.0f);
  }
  else
  {
    TPAFactor = 1.0f;
  }
  return TPAFactor * 100;
}

uint8_t CalculateMultirotorTPAFactor(int16_t Throttle)
{
  TPA_Update();
  float TPAFactor;
  if (TPA_Parameters.TPAThrottlePercent == 0 || Throttle < TPA_Parameters.TPABreakPointer)
  {
    TPAFactor = 1.0f;
  }
  else if (Throttle < TPA_Parameters.MaxThrottle)
  {
    TPAFactor = (100 - (uint16_t)TPA_Parameters.TPAThrottlePercent * (Throttle - TPA_Parameters.TPABreakPointer) /
                           (float)(TPA_Parameters.MaxThrottle - TPA_Parameters.TPABreakPointer)) /
                100.0f;
  }
  else
  {
    TPAFactor = (100 - TPA_Parameters.TPAThrottlePercent) / 100.0f;
  }
  return TPAFactor * 100;
}
