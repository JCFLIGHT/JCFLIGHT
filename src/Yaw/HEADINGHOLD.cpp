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
#include "GPS/GPSSTATES.h"
#include "BitArray/BITARRAY.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_HEADING_HOLD

#define HEADING_HOLD_LPF_FREQ 2

PT1_Filter_Struct HeadingHoldRate_Smooth;

void UpdateStateOfHeadingHold(void)
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    HeadingHoldRate_Smooth.State = 0.0f;                                   //RESETA O FILTRO
    GPS_Resources.Navigation.HeadingHoldTarget = Attitude.EulerAngles.Yaw; //OBTÉM UM NOVO VALOR INICIAL PARA HEADING HOLD TARGET
  }
}

bool GetSafeStateOfHeadingHold(void)
{
  if (!IS_FLIGHT_MODE_ACTIVE(HEADING_HOLD_MODE)) //NÃO APLICA A CORREÇÃO SE O MODO NÃO ESTIVER ATIVO
  {
    return false;
  }

  if (AHRS.CosineTiltAngle() < GPS_Resources.Navigation.HeadingHoldLimit) //NÃO APLICA A CORREÇÃO SE O COSSENO DE Z FOR MAIOR QUE 'N' GRAUS
  {
    return false;
  }

  if (!GPS_Resources.Navigation.AutoPilot.Control.Enabled) //NÃO APLICA A CORREÇÃO SE NENHUM MODO DE VOO POR GPS ESTIVER ATIVO
  {
    return false;
  }

  if (ABS(RCController[YAW]) != 0) //NÃO APLICA A CORREÇÃO SE O USUARIO MANIPULAR O YAW DO RADIO
  {
    return false;
  }

  return true; //TUDO ESTÁ OK
}

float GetHeadingHoldValue(float DeltaTime)
{
  float HeadingHoldRate;

  int16_t YawError = Attitude.EulerAngles.Yaw - GPS_Resources.Navigation.HeadingHoldTarget; //CALCULA O ERRO / DIFERENÇA

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
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRate_Smooth, HeadingHoldRate, HEADING_HOLD_LPF_FREQ, DeltaTime);
#else
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRate_Smooth, HeadingHoldRate, HEADING_HOLD_LPF_FREQ, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
#endif

#ifdef PRINTLN_HEADING_HOLD
  PRINTF.SendToConsole(ProgramMemoryString("HHR:%.2f HHRF:%.2f\n"),
                       HeadingHoldRateNF,
                       HeadingHoldRate);
#endif

  //APLICA O CONTROLE DO YAW
  return HeadingHoldRate;
}