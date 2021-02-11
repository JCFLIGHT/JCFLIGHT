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

#include "FAILSAFE.h"
#include "Common/VARIABLES.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Math/MATHSUPPORT.h"
#include "Buzzer/BUZZER.h"

#define THIS_LOOP_RATE 50     //HZ
#define FAILSAFE_DELAY 1      //SEGUNDO
#define FAILSAFE_DELAY2 0.25f //MS
#define STICK_MOTION 50       //50 uS DE DEFLEXÃO
#define RX_RECOVERY_TIME 15   //NO MINIMO 15 SEGUNDOS DE CONSISTENCIA ACIMA DO VALOR CONSIDERADO FAIL-SAFE

bool FailSafeGoodRunBeep = false;
int16_t RxConsistenceCount = 0;
int16_t BuzzerFailSafeRunCount = 0;

bool GetValidFailSafeState(float DelayToDetect)
{
  return (Fail_Safe_System > (THIS_LOOP_RATE * DelayToDetect));
}

void NormalizeFundamentalChnnels()
{
  RadioControllOutput[THROTTLE] = 1500;
  RadioControllOutput[YAW] = 1500;
  RadioControllOutput[PITCH] = 1500;
  RadioControllOutput[ROLL] = 1500;
}

void NormalizeAuxiliariesChnnels()
{
  RadioControllOutput[AUX1] = 1000;
  RadioControllOutput[AUX2] = 1000;
  RadioControllOutput[AUX3] = 1000;
  RadioControllOutput[AUX4] = 1000;
  RadioControllOutput[AUX5] = 1000;
  RadioControllOutput[AUX6] = 1000;
  RadioControllOutput[AUX7] = 1000;
  RadioControllOutput[AUX8] = 1000;
}

void NormalizeFlightModesToFailSafe()
{
  //MODOS DE VOO NECESSARIOS PARA O FAIL-SAFE
  //O ALT-HOLD É CHAMADO ATRAVÉS DE OUTRA FLAG
  SetFlightModes[STABILIZE_MODE] = true;
  SetFlightModes[RTH_MODE] = true;

  //MODOS DE VOO NÃO NECESSARIOS PARA O FAIL-SAFE
  SetFlightModes[ALTITUDE_HOLD_MODE] = false;
  SetFlightModes[GPS_HOLD_MODE] = false;
  SetFlightModes[IOC_MODE] = false;
  SetFlightModes[ATACK_MODE] = false;
  SetFlightModes[FLIP_MODE] = false;
  SetFlightModes[WAYPOINT_MODE] = false;
  SetFlightModes[LAND_MODE] = false;

  //DESATIVA OS MODOS DE VOO NÃO NECESSARIOS NO MODO AIR-PLANE
  SetFlightModes[AUTO_THROTTLE_MODE] = false;
  SetFlightModes[MANUAL_MODE] = false;
  SetFlightModes[CIRCLE_MODE] = false;
  SetFlightModes[LAUNCH_MODE] = false;
  SetFlightModes[TURN_MODE] = false;
  SetFlightModes[CRUISE_MODE] = false;
}

bool FailSafeCheckStickMotion()
{
  uint32_t CalcedRcDelta = 0;
  CalcedRcDelta += ABS_16BITS(RadioControllOutput[ROLL] - 1500);
  CalcedRcDelta += ABS_16BITS(RadioControllOutput[PITCH] - 1500);
  CalcedRcDelta += ABS_16BITS(RadioControllOutput[YAW] - 1500);
  return CalcedRcDelta >= STICK_MOTION;
}

bool GetRxConsistence()
{
  if (RxConsistenceCount >= (THIS_LOOP_RATE * RX_RECOVERY_TIME))
  {
    return true;
  }
  RxConsistenceCount++;
  return false;
}

void ResetRxConsistence()
{
  RxConsistenceCount = 0;
  BuzzerFailSafeRunCount = 0;
  FailSafeGoodRunBeep = true;
}

void ResetFailSafe()
{
  ImmediatelyFailSafe = false;
  Fail_Safe_Event = false;
  if (!RTHControlAux)
  {
    SetFlightModes[RTH_MODE] = false;
  }
  ResetRxConsistence();
}

void AbortFailSafe()
{
  //EVITA COM QUE O STICK MOTION RODE COM O FAIL-SAFE DESATIVADO
  if (!Fail_Safe_Event)
  {
    return;
  }
  //VERIFICA O TEMPO MINIMO PARA CONSIDERAR QUE O PILOTO AUTOMATICO DO FAIL-SAFE ESTÁ PRONTO PARA SER DESLIGADO
  if (!GetRxConsistence())
  {
    return;
  }
  //SE O PILOTO MOVER OS STICKS,O FAIL-SAFE IRÁ SER ABORTADO
  if (!FailSafeCheckStickMotion())
  {
    return;
  }
  ResetFailSafe();
}

void UpdateFailSafeSystem()
{
  Fail_Safe_System++;
  if (RxConsistenceCount == 0 && !Fail_Safe_Event && FailSafeGoodRunBeep)
  {
    if (BuzzerFailSafeRunCount > 100) //2 SEGUNDOS
    {
      BEEPER.Play(BEEPER_FAIL_SAFE_GOOD);
      FailSafeGoodRunBeep = false;
      BuzzerFailSafeRunCount = 0;
    }
    else
    {
      BuzzerFailSafeRunCount++;
    }
  }
}

void FailSafeBuzzerNotification()
{
  if (!Fail_Safe_Event)
  {
    return;
  }
  if (BEEPER.SafeToOthersBeepsCounter > 200 && CalibratingGyroscope == 0)
  {
    if (BuzzerFailSafeRunCount > 150) //3 SEGUNDOS
    {
      BEEPER.Play(BEEPER_FAIL_SAFE);
    }
    else
    {
      BuzzerFailSafeRunCount++;
    }
  }
}

void FailSafeCheck()
{
  //FAIL-SAFE IMEDIATO CASO O USUARIO ESTIVER USANDO O ARM-DISARM POR CANAL AUX
  if (GetValidFailSafeState(FAILSAFE_DELAY2))
  {
    ImmediatelyFailSafe = true;
  }

  //FAIL-SAFE LENTO PARA DIFERENCIAR SE É UMA FALHA OU REALMENTE UMA PERDA DE SINAL
  if (GetValidFailSafeState(FAILSAFE_DELAY))
  {
    Fail_Safe_Event = true;
    NormalizeFlightModesToFailSafe();
    NormalizeFundamentalChnnels();
    NormalizeAuxiliariesChnnels();
  }
  else
  {
    AbortFailSafe();
  }
  UpdateFailSafeSystem();
  FailSafeBuzzerNotification();
}
