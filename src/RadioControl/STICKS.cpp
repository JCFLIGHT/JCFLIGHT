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

#include "STICKS.h"
#include "LedRGB/LEDRGB.h"
#include "Common/VARIABLES.h"
#include "FlightModes/AUXFLIGHT.h"
#include "BatteryMonitor/BATTERY.h"
#include "RadioControl/RCCONFIG.h"
#include "STATES.h"
#include "Buzzer/BUZZER.h"
#include "Arming/ARMING.h"

bool PreArm_Delay = false;
uint8_t PreArm_Delay_Count = 0;

void RC_Sticks_Update()
{
  if (!COMMAND_ARM_DISARM)
  {
    if ((ArmDisarmConfig == 0) && (StickStateToArm()))
    {
      if (!BATTERY.LowBattPreventArm)
      {
        if (ArmDelayedState())
        {
          PreArm_Delay = true;
        }
      }
    }
    else
    {
      ResetArmDelayed();
    }
  }
  else
  {
    if ((ArmDisarmConfig == 0) && (StickStateToDisarm()))
    {
      if (DisarmDelayedState())
      {
        COMMAND_ARM_DISARM = false;
        BEEPER.Play(BEEPER_DISARMING);
      }
    }
    else
    {
      ResetDisarmDelayed();
    }
  }
  if (RadioControllOutput[THROTTLE] <= 1100)
  {
    if (!ImmediatelyFailSafe)
    {
      if (ArmDisarmConfig > 0)
      {
        if (ArmDisarmControlAux)
        {
          SetFlightModes[ARM_DISARM_MODE] = true;
        }
        else
        {
          SetFlightModes[ARM_DISARM_MODE] = false;
        }
        if (SetFlightModes[ARM_DISARM_MODE])
        {
          if (!Fail_Safe_Event && !SetFlightModes[RTH_MODE] && !SetFlightModes[GPS_HOLD_MODE])
          {
            if (!COMMAND_ARM_DISARM)
            {
              COMMAND_ARM_DISARM = true;
              IOC_Initial_Compass = ATTITUDE.AngleOut[YAW];
            }
          }
        }
        else
        {
          COMMAND_ARM_DISARM = false;
        }
      }
    }
  }
}

void Pre_Arm(void)
{
  if (ArmDisarmConfig != 0)
  {
    return; //FAÇA UMA RAPIDA SAÍDA SE O ARMDISARM ESTIVE CONFIGURADO PELA CHAVE AUX
  }
  //ROTINA PRE-ARM
  if (PreArm_Delay)
  {
    PreArm_Delay_Count++;
    if (PreArm_Delay_Count > 30)
    {
      if (!PREARM.CheckSafeState()) //CONDIÇÕES INCORRETAS?SIM...NÃO ARMA OS MOTORES
      {
        COMMAND_ARM_DISARM = false;
      }
      else //IMU CALIBRADA?SIM...ARMA OS MOTORES
      {
        if (!COMMAND_ARM_DISARM)
        {
          COMMAND_ARM_DISARM = true;
          IOC_Initial_Compass = ATTITUDE.AngleOut[YAW];
        }
      }
      PreArm_Delay = false;
      PreArm_Delay_Count = 0;
    }
  }
}

void Pre_Arm_Leds(void)
{
  //ROTINA PRE-ARM LED INDICADOR
  if ((PreArm_Delay_Count > 0 && PreArm_Delay_Count <= 20))
  {
    RGB.Function(PREARMINIT);
    BEEPER.Play(BEEPER_ARM);
  }
  if (!PREARM.CheckSafeState()) //SE TIVER ALGUMA CONDIÇÃO INCORRETA,NÃO ARMA
  {
    if ((PreArm_Delay_Count > 20 && PreArm_Delay_Count <= 30))
    {
      RGB.Function(PREARMFAIL);
      if (PreArm_Delay_Count == 21)
      {
        BEEPER.Play(BEEPER_ACTION_FAIL);
      }
      if (PreArm_Delay_Count == 30)
      {
        NotPriorit = false;
      }
    }
  }
  else //CASO CONTRARIO
  {
    if ((PreArm_Delay_Count > 20 && PreArm_Delay_Count <= 30))
    {
      RGB.Function(PREARMSUCESS);
      if (PreArm_Delay_Count == 30)
      {
        NotPriorit = false;
      }
    }
  }
}