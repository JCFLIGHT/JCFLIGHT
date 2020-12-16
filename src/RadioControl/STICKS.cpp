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
#include "SafetyButton/SAFETYBUTTON.h"

bool ArmDelay = false;
uint8_t Arm_Delay_Count = 0;

void RCSticks_Update()
{
  if (!SAFETYBUTTON.GetSafeStateToOutput())
    return;
  if (!COMMAND_ARM_DISARM)
  {
    if ((ArmDisarmConfig == 0) && (StickStateToArm()) && (!BATTERY.LowBattPreventArm))
    {
      ArmDelay = true;
    }
  }
  else
  {
    if ((ArmDisarmConfig == 0) && (StickStateToDisarm()))
    {
      COMMAND_ARM_DISARM = false;
      BEEPER.BeeperPlay(BEEPER_DISARMING);
    }
  }
  if (RadioControllOutput[THROTTLE] <= 1100)
  {
    if (!ImmediatelyFailSafe)
    {
      if (ArmDisarmConfig > 0)
      {
        if (ArmDisarmControlAux)
          SetFlightModes[ARMDISARM_MODE] = true;
        else
          SetFlightModes[ARMDISARM_MODE] = false;
        if (SetFlightModes[ARMDISARM_MODE])
        {
          if (Fail_Safe_System < 5 && !SetFlightModes[RTH_MODE] && !SetFlightModes[GPSHOLD_MODE])
          {
            if (!COMMAND_ARM_DISARM)
            {
              COMMAND_ARM_DISARM = true;
              IOC_Initial_Compass = ATTITUDE.CalculedHeading;
              HeadingHoldTarget = ATTITUDE.CalculedHeading;
            }
          }
        }
        else
          COMMAND_ARM_DISARM = false;
      }
    }
  }
}

void Pre_Arm(void)
{
  if (ArmDisarmConfig != 0)
    return; //FAÇA UMA RAPIDA SAÍDA SE O ARMDISARM ESTIVE CONFIGURADO PELA CHAVE AUX
  //ROTINA PRE-ARM
  if (ArmDelay)
  {
    Arm_Delay_Count++;
    if (Arm_Delay_Count > 30)
    {
      if (CALIBRATION.AccelerometerCalibration[ROLL] > 2000 ||
          SetFlightModes[ALTITUDEHOLD_MODE] ||
          GPS_Flight_Mode != GPS_MODE_NONE ||
          Fail_Safe_System > 5 ||
          CalibratingGyroscope > 0 ||
          CheckInclinationForCopter())
        COMMAND_ARM_DISARM = false; //CONDIÇÕES INCORRETAS?SIM...NÃO ARMA OS MOTORES
      else
      { //IMU CALIBRADA?SIM...ARMA OS MOTORES
        if (!COMMAND_ARM_DISARM)
        {
          COMMAND_ARM_DISARM = true;
          IOC_Initial_Compass = ATTITUDE.CalculedHeading;
          HeadingHoldTarget = ATTITUDE.CalculedHeading;
        }
      }
      ArmDelay = false;
      Arm_Delay_Count = 0;
    }
  }
}

void Pre_Arm_Leds(void)
{
  //ROTINA PRE-ARM LED INDICADOR
  if ((Arm_Delay_Count > 0 && Arm_Delay_Count <= 20))
  {
    RGB.Function(PREARMINIT);
    BEEPER.BeeperPlay(BEEPER_ARM);
  }
  if (CALIBRATION.AccelerometerCalibration[ROLL] > 2000 ||
      SetFlightModes[ALTITUDEHOLD_MODE] ||
      GPS_Flight_Mode != GPS_MODE_NONE ||
      Fail_Safe_System > 5 ||
      CalibratingGyroscope > 0 ||
      CheckInclinationForCopter())
  { //SE TIVER ALGUMA CONDIÇÃO INCORRETA,NÃO ARMA
    if ((Arm_Delay_Count > 20 && Arm_Delay_Count <= 30))
    {
      RGB.Function(PREARMFAIL);
      if (Arm_Delay_Count == 21)
        BEEPER.BeeperPlay(BEEPER_ACTION_FAIL);
      if (Arm_Delay_Count == 30)
        NotPriorit = false;
    }
  }
  else
  { //CASO CONTRARIO
    if ((Arm_Delay_Count > 20 && Arm_Delay_Count <= 30))
    {
      RGB.Function(PREARMSUCESS);
      if (Arm_Delay_Count == 30)
        NotPriorit = false;
    }
  }
}