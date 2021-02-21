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
#include "RCSTATES.h"
#include "Buzzer/BUZZER.h"
#include "Arming/ARMING.h"
#include "Common/RCDEFINES.h"
#include "RCSTATES.h"

SticksClass STICKS;

void SticksClass::Update()
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    if ((ArmDisarmConfig == 0) && (SticksStateToArm()))
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
    if ((ArmDisarmConfig == 0) && (SticksStateToDisarm()))
    {
      if (DisarmDelayedState())
      {
        DISABLE_STATE(PRIMARY_ARM_DISARM);
        BEEPER.Play(BEEPER_DISARMING);
      }
    }
    else
    {
      ResetDisarmDelayed();
    }
  }
  if (GetActualThrottleStatus(THROTTLE_LOW))
  {
    if (!ImmediatelyFailSafe)
    {
      if (ArmDisarmConfig > 0)
      {
        ENABLE_DISABLE_STATE_WITH_DEPENDENCY(SECONDARY_ARM_DISARM, ArmDisarmControlAux);
        if (IS_STATE_ACTIVE(SECONDARY_ARM_DISARM))
        {
          if (!Fail_Safe_Event && !IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
          {
            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
              ENABLE_STATE(PRIMARY_ARM_DISARM);
              IOC_Initial_Compass = ATTITUDE.AngleOut[YAW];
            }
          }
        }
        else
        {
          DISABLE_STATE(PRIMARY_ARM_DISARM);
        }
      }
    }
  }
}

void SticksClass::Pre_Arm(void)
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
        DISABLE_STATE(PRIMARY_ARM_DISARM);
      }
      else //IMU CALIBRADA?SIM...ARMA OS MOTORES
      {
        ENABLE_STATE(PRIMARY_ARM_DISARM);
        IOC_Initial_Compass = ATTITUDE.AngleOut[YAW];
      }
      PreArm_Delay = false;
      PreArm_Delay_Count = 0;
    }
  }
}

void SticksClass::Pre_Arm_Leds(void)
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