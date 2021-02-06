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

#include "RCSMOOTH.h"
#include "Math/MATHSUPPORT.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FastSerial/PRINTF.h"
#ifndef __AVR_ATmega2560__
#include "Filters/LPFSERVO.h"
#else
#include "Filters/PT1.h"
#endif
#include "BAR/BAR.h"

//DEBUG
//#define PRINTLN_RC_INTERPOLATION

#ifndef __AVR_ATmega2560__
Struct_LowPassFilter RCSmooth_LPF[4];
#else
PT1_Filter_Struct PT1_RC_Throttle;
PT1_Filter_Struct PT1_RC_Yaw;
PT1_Filter_Struct PT1_RC_Roll;
PT1_Filter_Struct PT1_RC_Pitch;
#endif

int16_t RCControllerUnFiltered[4];
int16_t RCAttitudeFiltered[4];

void RCInterpolationApply()
{
  int16_t RCFilterFrequencyEEPROM = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);

  //GUARDA OS VALORES ANTERIOR
  RCControllerUnFiltered[THROTTLE] = RCController[THROTTLE];
  RCControllerUnFiltered[YAW] = RCController[YAW];
  RCControllerUnFiltered[PITCH] = RCController[PITCH];
  RCControllerUnFiltered[ROLL] = RCController[ROLL];

  if (RCFilterFrequencyEEPROM != 0)
  {
    //APLICA O FILTRO
#ifndef __AVR_ATmega2560__
    RCAttitudeFiltered[THROTTLE] = (int16_t)LowPassFilter(&RCSmooth_LPF[THROTTLE], RCControllerUnFiltered[THROTTLE], RCFilterFrequencyEEPROM, AttitudeThrottleMin);
    RCAttitudeFiltered[YAW] = (int16_t)LowPassFilter(&RCSmooth_LPF[YAW], RCControllerUnFiltered[YAW], RCFilterFrequencyEEPROM, 0);
    RCAttitudeFiltered[PITCH] = (int16_t)LowPassFilter(&RCSmooth_LPF[PITCH], RCControllerUnFiltered[PITCH], RCFilterFrequencyEEPROM, 0);
    RCAttitudeFiltered[ROLL] = (int16_t)LowPassFilter(&RCSmooth_LPF[ROLL], RCControllerUnFiltered[ROLL], RCFilterFrequencyEEPROM, 0);
#else
    RCAttitudeFiltered[THROTTLE] = (int16_t)PT1FilterApply(&PT1_RC_Throttle, RCControllerUnFiltered[THROTTLE], RCFilterFrequencyEEPROM, 1.0f / 1000);
    RCAttitudeFiltered[YAW] = (int16_t)PT1FilterApply(&PT1_RC_Yaw, RCControllerUnFiltered[YAW], RCFilterFrequencyEEPROM, 1.0f / 1000);
    RCAttitudeFiltered[PITCH] = (int16_t)PT1FilterApply(&PT1_RC_Pitch, RCControllerUnFiltered[PITCH], RCFilterFrequencyEEPROM, 1.0f / 1000);
    RCAttitudeFiltered[ROLL] = (int16_t)PT1FilterApply(&PT1_RC_Roll, RCControllerUnFiltered[ROLL], RCFilterFrequencyEEPROM, 1.0f / 1000);
#endif

    //OBTÉM O VALOR FILTRADO
    RCController[THROTTLE] = RCAttitudeFiltered[THROTTLE];
    RCController[YAW] = RCAttitudeFiltered[YAW];
    RCController[PITCH] = RCAttitudeFiltered[PITCH];
    RCController[ROLL] = RCAttitudeFiltered[ROLL];
  }
#if defined(PRINTLN_RC_INTERPOLATION)
  static uint32_t Refresh = 0;
  if (SCHEDULERTIME.GetMillis() - Refresh >= 20)
  {
    FastSerialPrintln(PSTR("NotFiltered:%d Filtered:%d\n"),
                      RCControllerUnFiltered[ROLL],
                      RCController[ROLL]);
    Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}
