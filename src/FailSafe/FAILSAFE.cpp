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
#include "WayPointNavigation/WAYPOINT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Math/MATHSUPPORT.h"
#include "Buzzer/BUZZER.h"
#include "Common/RCDEFINES.h"
#include "RadioControl/DECODE.h"
#include "IMU/IMUCALIBRATE.h"
#include "BitArray/BITARRAY.h"

#define THIS_LOOP_RATE 50                //HZ
#define IMMEDIATELY_FAILSAFE_DELAY 0.25f //MS
#define AUTOPILOT_FAILSAFE_DELAY 1       //SEGUNDO
#define STICK_MOTION 50                  //50 uS DE DEFLEXÃO
#define RX_RECOVERY_TIME_ARMED 15        //NO MINIMO 15 SEGUNDOS DE CONSISTENCIA ACIMA DO VALOR CONSIDERADO FAIL-SAFE EM MODO ARMADO
#define RX_RECOVERY_TIME_DESARMED 2      //NO MINIMO 2 SEGUNDOS DE CONSISTENCIA ACIMA DO VALOR CONSIDERADO FAIL-SAFE EM MODO DESARMADO

bool Fail_Safe_Event;
bool ImmediatelyFailSafe;
bool FailSafeGoodRunBeep = false;
bool FailSafeDesarmedRecovered = false;

int16_t RxConsistenceCount = 0;
int16_t BuzzerFailSafeRunCount = 0;

volatile int16_t Fail_Safe_System_Count;

bool FastSystemFailSafe()
{
  return ImmediatelyFailSafe;
}

bool SystemInFailSafe()
{
  return Fail_Safe_Event;
}

bool GetValidFailSafeState(float DelayToDetect)
{
  return (Fail_Safe_System_Count > (THIS_LOOP_RATE * DelayToDetect));
}

void NormalizeFundamentalChannels()
{
  DECODE.SetRxChannelInput(THROTTLE, MIDDLE_STICKS_PULSE);
  DECODE.SetRxChannelInput(YAW, MIDDLE_STICKS_PULSE);
  DECODE.SetRxChannelInput(PITCH, MIDDLE_STICKS_PULSE);
  DECODE.SetRxChannelInput(ROLL, MIDDLE_STICKS_PULSE);
}

void NormalizeAuxiliariesChnnels()
{
  DECODE.SetRxChannelInput(AUX1, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX2, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX3, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX4, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX5, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX6, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX7, MIN_STICKS_PULSE);
  DECODE.SetRxChannelInput(AUX8, MIN_STICKS_PULSE);
}

void NormalizeFlightModesToFailSafe()
{
  //ATIVA OS MODOS DE VOO NECESSARIOS PARA O FAIL-SAFE
  //O ALT-HOLD É CHAMADO ATRAVÉS DE OUTRA FLAG
  ENABLE_THIS_FLIGHT_MODE(STABILIZE_MODE);
  ENABLE_THIS_FLIGHT_MODE(RTH_MODE);

  //DESATIVA OS MODOS DE VOO NÃO NECESSARIOS PARA O FAIL-SAFE
  DISABLE_THIS_FLIGHT_MODE(ALTITUDE_HOLD_MODE);
  DISABLE_THIS_FLIGHT_MODE(POS_HOLD_MODE);
  DISABLE_THIS_FLIGHT_MODE(SIMPLE_MODE);
  DISABLE_THIS_FLIGHT_MODE(ATTACK_MODE);
  DISABLE_THIS_FLIGHT_MODE(FLIP_MODE);
  DISABLE_THIS_FLIGHT_MODE(WAYPOINT_MODE);
  DISABLE_THIS_FLIGHT_MODE(LAND_MODE);

  //DESATIVA OS MODOS DE VOO NÃO NECESSARIOS NO MODO AIRPLANE
  DISABLE_THIS_FLIGHT_MODE(AUTO_THROTTLE_MODE);
  DISABLE_THIS_FLIGHT_MODE(MANUAL_MODE);
  DISABLE_THIS_FLIGHT_MODE(CIRCLE_MODE);
  DISABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
  DISABLE_THIS_FLIGHT_MODE(TURN_MODE);
  DISABLE_THIS_FLIGHT_MODE(CRUISE_MODE);
}

bool FailSafeCheckStickMotion()
{
  uint32_t CalcedRcDelta = 0;
  CalcedRcDelta += ABS(DECODE.GetRxChannelOutput(ROLL) - MIDDLE_STICKS_PULSE);
  CalcedRcDelta += ABS(DECODE.GetRxChannelOutput(PITCH) - MIDDLE_STICKS_PULSE);
  CalcedRcDelta += ABS(DECODE.GetRxChannelOutput(YAW) - MIDDLE_STICKS_PULSE);
  return CalcedRcDelta >= STICK_MOTION;
}

bool GetRxConsistence(uint8_t RecoveryTime)
{
  if (RxConsistenceCount >= (THIS_LOOP_RATE * RecoveryTime))
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
  FailSafeDesarmedRecovered = false;
}

void ResetFailSafe()
{
  ImmediatelyFailSafe = false;
  Fail_Safe_Event = false;
  if (!RTHControlAux)
  {
    DISABLE_THIS_FLIGHT_MODE(RTH_MODE);
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
  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    //VERIFICA O TEMPO MINIMO PARA CONSIDERAR QUE O PILOTO AUTOMATICO DO FAIL-SAFE ESTÁ PRONTO PARA SER DESLIGADO
    if (!GetRxConsistence(RX_RECOVERY_TIME_ARMED))
    {
      return;
    }
    //SE O PILOTO MOVER OS STICKS,O FAIL-SAFE IRÁ SER ABORTADO
    if (!FailSafeCheckStickMotion())
    {
      return;
    }
  }
  else
  {
    if (FailSafeDesarmedRecovered)
    {
      return;
    }
    if (!GetRxConsistence(RX_RECOVERY_TIME_DESARMED))
    {
      FailSafeDesarmedRecovered = false;
      return;
    }
    else
    {
      FailSafeDesarmedRecovered = true;
    }
  }
  ResetFailSafe();
}

bool RxConsistenceOK()
{
  return RxConsistenceCount == NONE;
}

void UpdateFailSafeSystem()
{
  Fail_Safe_System_Count++;
  if (RxConsistenceOK() && !Fail_Safe_Event && FailSafeGoodRunBeep)
  {
    if (BuzzerFailSafeRunCount >= 100) //2 SEGUNDOS
    {
      BEEPER.Play(BEEPER_FAIL_SAFE_GOOD);
      BuzzerFailSafeRunCount = 0;
      FailSafeGoodRunBeep = false;
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
  if (BEEPER.GetSafeStateToOthersBeeps() && CalibratingGyroscope == NONE)
  {
    if (BuzzerFailSafeRunCount >= 150) //3 SEGUNDOS
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
  //FAIL-SAFE IMEDIATO CASO O USUARIO ESTIVER USANDO O ARM-DISARM POR CANAL AUX E O DESARM-LOW-THR
  if (GetValidFailSafeState(IMMEDIATELY_FAILSAFE_DELAY))
  {
    ImmediatelyFailSafe = true;
  }

  //FAIL-SAFE LENTO PARA DIFERENCIAR SE É UMA FALHA OU REALMENTE UMA PERDA DE SINAL
  if (GetValidFailSafeState(AUTOPILOT_FAILSAFE_DELAY))
  {
    Fail_Safe_Event = true;
    NormalizeFlightModesToFailSafe();
    NormalizeFundamentalChannels();
    NormalizeAuxiliariesChnnels();
  }
  else
  {
    AbortFailSafe();
  }
  UpdateFailSafeSystem();
  FailSafeBuzzerNotification();
}

void FailSafe_Do_RTH_With_Low_Batt(bool FailSafeBatt)
{
  //ENTRA EM MODO RTH OU LAND (SE ESTIVER PROXIMO DO HOME-POINT)
  //SE A BATERIA ESTIVER COM A TENSÃO ABAIXO DE 20% DA CARGA TOTAL
  static bool FailSafeBattDetect = false;
  if (FailSafeBatt)
  {
    //EVITA COM QUE UMA TROCA RAPIDA DE TRUE PARA FALSE OCORRA
    //A FIM DE NÃO INTERFERIR NO FUNCIONAMENTO DESSA FUNÇÃO
    FailSafeBattDetect = true;
  }
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    FailSafeBattDetect = false;
    return;
  }
  if (!FailSafeBattDetect)
  {
    return;
  }
}