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
#include "Common/VARIABLES.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"

//**************************************************************************
//TIMER DE DESLIGAMENTO AUTOMATICO DOS MOTORES POR INATIVADADE DO THROTTLE
//**************************************************************************

#define THIS_LOOP_RATE 50  //HZ
#define AUTO_DISARM_TIME 5 //SEGUNDOS
#define THROTTLE_VALUE_MAX 1100
#define YPR_VALUE_MIN 1450
#define YPR_VALUE_MAX 1550

uint8_t TimerDesarm;

void Desarm_LowThrottle()
{
  //FAÇA UMA RAPIDA SAÍDA SE O MODO AERO OU ASA-FIXA ESTIVER ATIVADO
  if (GetFrameStateOfAirPlane())
  {
    return;
  }
  //THROTTLE NO MINIMO,DRONE ARMADO,FAIL-SAFE DESATIVADO?SIM...
  if (Check_Throttle() && Check_Others_Channels() && COMMAND_ARM_DISARM &&
      !ImmediatelyFailSafe && !SetFlightModes[WAYPOINT_MODE] && ArmDisarmConfig == 0)
  {
    if (TimerDesarm == (THIS_LOOP_RATE * AUTO_DISARM_TIME))
    {
      COMMAND_ARM_DISARM = false;    //DESARMA OS MOTORES
      BEEPER.Play(BEEPER_DISARMING); //TOCA A MÚSICA INDICANDO O DESARM
    }
    else if (TimerDesarm > 254)
    {
      TimerDesarm = 254;
    }
    else
    {
      TimerDesarm++; //INICIA A CONTAGEM
    }
  }
  else
  {
    TimerDesarm = 0; //RESETA A CONTAGEM
  }
}

bool Check_Throttle()
{
  if (RadioControllOutput[THROTTLE] <= THROTTLE_VALUE_MAX)
  {
    return true;
  }
  return false;
}

bool Check_Others_Channels()
{
  if ((RadioControllOutput[YAW] >= YPR_VALUE_MIN && RadioControllOutput[YAW] <= YPR_VALUE_MAX) &&
      (RadioControllOutput[PITCH] >= YPR_VALUE_MIN && RadioControllOutput[PITCH] <= YPR_VALUE_MAX) &&
      (RadioControllOutput[ROLL] >= YPR_VALUE_MIN && RadioControllOutput[PITCH] <= YPR_VALUE_MAX))
  {
    return true;
  }
  return false;
}
