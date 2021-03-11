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
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "Filters/BIQUADFILTER.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "PID/RCPID.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_RC_INTERPOLATION

static BiquadFilter_Struct Smooth_RC_Throttle;
static BiquadFilter_Struct Smooth_RC_Yaw;
static BiquadFilter_Struct Smooth_RC_Roll;
static BiquadFilter_Struct Smooth_RC_Pitch;

int16_t RC_LPF_CutOff;
int16_t RCControllerUnFiltered[4];
int16_t RCAttitudeFiltered[4];

void RCInterpolationInit()
{
  RC_LPF_CutOff = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);
  BIQUADFILTER.Settings(&Smooth_RC_Throttle, RC_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Yaw, RC_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Roll, RC_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_RC_Pitch, RC_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
}

void RCInterpolationApply()
{

  if (RC_LPF_CutOff > 0)
  {
    //GUARDA OS VALORES ANTERIOR
    RCControllerUnFiltered[THROTTLE] = RCController[THROTTLE];
    RCControllerUnFiltered[YAW] = RCController[YAW];
    RCControllerUnFiltered[PITCH] = RCController[PITCH];
    RCControllerUnFiltered[ROLL] = RCController[ROLL];

    //APLICA O FILTRO
    RCAttitudeFiltered[THROTTLE] = BIQUADFILTER.FilterApplyAndGet(&Smooth_RC_Throttle, RCControllerUnFiltered[THROTTLE]);
    RCAttitudeFiltered[YAW] = BIQUADFILTER.FilterApplyAndGet(&Smooth_RC_Yaw, RCControllerUnFiltered[YAW]);
    RCAttitudeFiltered[PITCH] = BIQUADFILTER.FilterApplyAndGet(&Smooth_RC_Pitch, RCControllerUnFiltered[PITCH]);
    RCAttitudeFiltered[ROLL] = BIQUADFILTER.FilterApplyAndGet(&Smooth_RC_Roll, RCControllerUnFiltered[ROLL]);

    //OBTÉM O VALOR FILTRADO
    RCController[THROTTLE] = ((RCAttitudeFiltered[THROTTLE]) >= (AttitudeThrottleMin) ? (RCAttitudeFiltered[THROTTLE]) : (AttitudeThrottleMin));
    RCController[YAW] = RCAttitudeFiltered[YAW];
    RCController[PITCH] = RCAttitudeFiltered[PITCH];
    RCController[ROLL] = RCAttitudeFiltered[ROLL];
  }
  else
  {
    RCController[THROTTLE] = ((RCController[THROTTLE]) >= (AttitudeThrottleMin) ? (RCController[THROTTLE]) : (AttitudeThrottleMin));
  }
#if defined(PRINTLN_RC_INTERPOLATION)
  static uint32_t Refresh = SCHEDULERTIME.GetMillis();
  if (SCHEDULERTIME.GetMillis() - Refresh >= 20)
  {
    PRINTF.SendToConsole(ProgramMemoryString("NotFiltered:%d Filtered:%d\n"),
                         RCControllerUnFiltered[ROLL],
                         RCController[ROLL]);
    Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}
