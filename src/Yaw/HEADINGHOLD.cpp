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

#include "HEADINGHOLD.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "PID/PIDPARAMS.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_HEADING_HOLD

#define HEADING_HOLD_LPF_FREQ 2

PT1_Filter_Struct HeadingHoldRateFilter;

int16_t HeadingHoldTarget;

void UpdateStateOfHeadingHold(void)
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    HeadingHoldRateFilter.State = 0.0f;         //RESETA O FILTRO
    HeadingHoldTarget = Attitude.EulerAngles.Yaw; //OBTÉM UM NOVO VALOR INICIAL PARA HEADING HOLD TARGET
  }
}

bool GetSafeStateOfHeadingHold()
{
  if (!IS_FLIGHT_MODE_ACTIVE(HEADING_HOLD_MODE)) //NÃO APLICA A CORREÇÃO DO YAW SE O MODO NÃO ESTIVER ATIVO
  {
    return false;
  }

  if (AHRS.CheckAnglesInclination(25)) //NÃO APLICA A CORREÇÃO DO YAW SE OS ANGULOS FOREM MAIOR QUE 25 GRAUS
  {
    return false;
  }

  if (GPSParameters.Mode.Flight == DO_NONE) //NÃO APLICA A CORREÇÃO DO YAW SE NENHUM MODO DE VOO USANDO O GPS ESTIVER ATIVO
  {
    return false;
  }

  if (ABS(RCController[YAW]) != 0) //NÃO APLICA A CORREÇÃO DO YAW SE O USUARIO MANIPULAR O STICK YAW DO RADIO
  {
    return false;
  }

  return true; //TUDO ESTÁ OK
}

float GetHeadingHoldValue(float DeltaTime)
{
  float HeadingHoldRate;

  int16_t YawError = Attitude.EulerAngles.Yaw - HeadingHoldTarget; //CALCULA O ERRO / DIFERENÇA

  //CALCULA O VALOR RELATIVO DO ERRO/DIFERENÇA E CONVERTE PARA UM VALOR ACEITAVEL POR HEADING
  if (YawError <= -180)
  {
    YawError += 360;
  }

  if (YawError >= +180)
  {
    YawError -= 360;
  }

  //CALCULA O VALOR DO RATE
  HeadingHoldRate = YawError * GET_SET[P_YAW_RATE].kP / 30.0f;

  //APLICA LIMITES MIN E MAX NO RATE
  HeadingHoldRate = Constrain_Float(HeadingHoldRate, -GET_SET[P_YAW_RATE_LIMIT].MaxValue, GET_SET[P_YAW_RATE_LIMIT].MaxValue);

#ifdef PRINTLN_HEADING_HOLD
  float HeadingHoldRateNF = HeadingHoldRate;
#endif

//REALIZA FILTRAGEM DO RATE COM O PT1
#ifndef __AVR_ATmega2560__
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRateFilter, HeadingHoldRate, HEADING_HOLD_LPF_FREQ, DeltaTime);
#else
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRateFilter, HeadingHoldRate, HEADING_HOLD_LPF_FREQ, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f);
#endif

#ifdef PRINTLN_HEADING_HOLD
  PRINTF.SendToConsole(ProgramMemoryString("HHR:%.2f HHRF:%.2f\n"),
                       HeadingHoldRateNF,
                       HeadingHoldRate);
#endif

  //APLICA O CONTROLE DO YAW
  return HeadingHoldRate;
}