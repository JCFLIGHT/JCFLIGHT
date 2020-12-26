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

#define THROTTLE_VALUE_MAX 1100
#define YPR_VALUE_MIN 1450
#define YPR_VALUE_MAX 1550

uint8_t TimerDesarm, Counter_One_Hertz;

void Desarm_LowThrottle()
{
  if (GetFrameStateOfAirPlane())
    return; //FAÇA UMA RAPIDA SAÍDA SE O MODO AERO OU ASA-FIXA ESTIVER ATIVADO
  if (ArmDisarmConfig != 0)
    return; //FAÇA UMA RAPIDA SAÍDA SE O COMANDO ARMDISARM FOR PELA CHAVE AUX
  //THROTTLE NO MINIMO,DRONE ARMADO,FAIL-SAFE DESATIVADO?SIM...
  if (Check_Throttle() && Check_Others_Channels() && COMMAND_ARM_DISARM && Fail_Safe_System < 5)
  {
    //ROTINA DE 1HZ
    Counter_One_Hertz++;
    if (Counter_One_Hertz >= 50)
    {
      TimerDesarm++;        //INICIA A CONTAGEM
      if (TimerDesarm == 4) //4 CONTAGENS = 4 SEGUNDOS
      {
        if (COMMAND_ARM_DISARM && !Cancel_Arm_Disarm)
        {
          COMMAND_ARM_DISARM = false;          //DESARMA OS MOTORES
          BEEPER.Play(BEEPER_DISARMING); //TOCA A MÚSICA INDICANDO O DESARM
        }
      }
      else if (TimerDesarm > 254)
        TimerDesarm = 254;
      Counter_One_Hertz = 0;
    }
  }
  else
  {
    TimerDesarm = 0;       //RESETA A CONTAGEM
    Counter_One_Hertz = 0; //RESETA A CONTAGEM
  }
}

bool Check_Throttle()
{
  if (RadioControllOutput[THROTTLE] <= THROTTLE_VALUE_MAX)
    return true;
  return false;
}

bool Check_Others_Channels()
{
  if ((RadioControllOutput[YAW] >= YPR_VALUE_MIN && RadioControllOutput[YAW] <= YPR_VALUE_MAX) &&
      (RadioControllOutput[PITCH] >= YPR_VALUE_MIN && RadioControllOutput[PITCH] <= YPR_VALUE_MAX) &&
      (RadioControllOutput[ROLL] >= YPR_VALUE_MIN && RadioControllOutput[PITCH] <= YPR_VALUE_MAX))
    return true;
  return false;
}
