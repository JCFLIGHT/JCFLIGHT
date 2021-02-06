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
#include "Filters/LPFSERVO.h"
#include "BAR/BAR.h"

//DEBUG
//#define PRINTLN_RC_INTERPOLATION

Struct_LowPassFilter RCSmooth_LPF[4];
int16_t RCControllerUnFiltered[4];
int16_t DevicesFiltered[4];

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
    //OBTÉM O VALOR FILTRADO
    DevicesFiltered[THROTTLE] = (int16_t)LowPassFilter(&RCSmooth_LPF[THROTTLE], RCControllerUnFiltered[THROTTLE], RCFilterFrequencyEEPROM, MotorSpeed);
    DevicesFiltered[YAW] = (int16_t)LowPassFilter(&RCSmooth_LPF[YAW], RCControllerUnFiltered[YAW], RCFilterFrequencyEEPROM, 0);
    DevicesFiltered[PITCH] = (int16_t)LowPassFilter(&RCSmooth_LPF[PITCH], RCControllerUnFiltered[PITCH], RCFilterFrequencyEEPROM, 0);
    DevicesFiltered[ROLL] = (int16_t)LowPassFilter(&RCSmooth_LPF[ROLL], RCControllerUnFiltered[ROLL], RCFilterFrequencyEEPROM, 0);

    //APLICA O FILTRO
    RCController[THROTTLE] = Constrain_16Bits(DevicesFiltered[THROTTLE], MotorSpeed, AttitudeThrottleMax);
    RCController[YAW] = DevicesFiltered[YAW];
    RCController[PITCH] = DevicesFiltered[PITCH];
    RCController[ROLL] = DevicesFiltered[ROLL];
  }
#if defined(PRINTLN_RC_INTERPOLATION)
  static uint32_t Refresh = 0;
  if (SCHEDULERTIME.GetMillis() - Refresh >= 20)
  {
    FastSerialPrintln(PSTR("NotFiltered:%d Filtered:%d\n"),
                      RCControllerUnFiltered[THROTTLE],
                      RCController[THROTTLE]);
    Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}
