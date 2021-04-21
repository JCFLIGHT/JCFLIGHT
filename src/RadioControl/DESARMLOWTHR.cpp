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

#include "DESARMLOWTHR.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "RadioControl/DECODE.h"
#include "FailSafe/FAILSAFE.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"

//**************************************************************************
//TIMER DE DESLIGAMENTO AUTOMATICO DOS MOTORES POR INATIVADADE DO THROTTLE
//**************************************************************************

#define THIS_LOOP_RATE 50  //HZ
#define AUTO_DISARM_TIME 5 //SEGUNDOS
#define THROTTLE_VALUE_MAX 1100
#define YPR_VALUE_MIN 1450
#define YPR_VALUE_MAX 1550

DesarmLowThrClass DESARMLOWTHROTTLE;

bool Check_Throttle()
{
  if (DECODE.GetRxChannelOutput(THROTTLE) <= THROTTLE_VALUE_MAX)
  {
    return true;
  }
  return false;
}

bool Check_Others_Channels()
{
  if ((DECODE.GetRxChannelOutput(YAW) >= YPR_VALUE_MIN && DECODE.GetRxChannelOutput(YAW) <= YPR_VALUE_MAX) &&
      (DECODE.GetRxChannelOutput(PITCH) >= YPR_VALUE_MIN && DECODE.GetRxChannelOutput(PITCH) <= YPR_VALUE_MAX) &&
      (DECODE.GetRxChannelOutput(ROLL) >= YPR_VALUE_MIN && DECODE.GetRxChannelOutput(PITCH) <= YPR_VALUE_MAX))
  {
    return true;
  }
  return false;
}

void DesarmLowThrClass::Update()
{
  //FAÇA UMA RAPIDA SAÍDA SE O MODO AERO OU ASA-FIXA ESTIVER ATIVADO
  if (GetAirPlaneEnabled())
  {
    return;
  }
  //THROTTLE NO MINIMO,DRONE ARMADO,FAIL-SAFE DESATIVADO?SIM...
  if (Check_Throttle() && Check_Others_Channels() && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&
      !FastSystemFailSafe() && !IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE) && ArmDisarmConfig == 0)
  {
    if (DESARMLOWTHROTTLE.TimerDesarm == (THIS_LOOP_RATE * AUTO_DISARM_TIME))
    {
      DISABLE_STATE(PRIMARY_ARM_DISARM); //DESARMA OS MOTORES
      BEEPER.Play(BEEPER_DISARMING);     //TOCA A MÚSICA INDICANDO O DESARM
    }
    else if (DESARMLOWTHROTTLE.TimerDesarm > 254)
    {
      DESARMLOWTHROTTLE.TimerDesarm = 254;
    }
    else
    {
      DESARMLOWTHROTTLE.TimerDesarm++; //INICIA A CONTAGEM
    }
  }
  else
  {
    DESARMLOWTHROTTLE.TimerDesarm = 0; //RESETA A CONTAGEM
  }
}
