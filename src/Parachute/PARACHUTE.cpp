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

#include "PARACHUTE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "Common/RCDEFINES.h"

ParachuteClass PARACHUTE;

#define MOTORS_DISARM_TIME 40 //MS - DESARMA OS MOTORES APÓS DETECTAR QUE O PARACHUTE FOI LANÇADO

void ParachuteClass::Auto_Do_Now(bool ActiveParachute)
{
  if (!ActiveParachute)
  {
    MotorControl[PARACHUTESERVO] = 400; //0 GRAUS
    PARACHUTE.ParachuteReleased = false;
    return;
  }
  PARACHUTE.ParachuteInAuto = true;
  MotorControl[PARACHUTESERVO] = 2400; //180 GRAUS
  if (!PARACHUTE.ParachuteReleased)
  {
    BEEPER.Play(BEEPER_PARACHUTE);
  }
  PARACHUTE.ParachuteReleased = true;
}

void ParachuteClass::Manual_Do_Now()
{
  if (!PARACHUTE.ParachuteReleased)
  {
    PARACHUTE.OverFlowTime += SCHEDULERTIME.GetMillis();
  }
  if (PARACHUTE.ParachuteInAuto)
  {
    return;
  }
  if (!PARACHUTE.ManualDetectTrigger)
  {
    MotorControl[PARACHUTESERVO] = 400; //0 GRAUS
    PARACHUTE.ParachuteReleased = false;
    return;
  }
  MotorControl[PARACHUTESERVO] = 2400; //180 GRAUS
  if (!PARACHUTE.ParachuteReleased)
  {
    BEEPER.Play(BEEPER_PARACHUTE);
  }
  PARACHUTE.ParachuteReleased = true;
}

void ParachuteClass::Manual_Detect_Channel()
{
  switch (ParachuteDetectTrigger)
  {

  case PARACHUTEAUXONELOW:
    PARACHUTE.ManualDetectTrigger = AUX1_LOW;
    break;

  case PARACHUTEAUXONEMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX1_MID;
    break;

  case PARACHUTEAUXONEHIGH:
    PARACHUTE.ManualDetectTrigger = AUX1_HIGH;
    break;

  case PARACHUTEAUXTWOLOW:
    PARACHUTE.ManualDetectTrigger = AUX2_LOW;
    break;

  case PARACHUTEAUXTWOMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX2_MID;
    break;

  case PARACHUTEAUXTWOHIGH:
    PARACHUTE.ManualDetectTrigger = AUX2_HIGH;
    break;

  case PARACHUTEAUXTHREELOW:
    PARACHUTE.ManualDetectTrigger = AUX3_LOW;
    break;

  case PARACHUTEAUXTHREEMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX3_MID;
    break;

  case PARACHUTEAUXTHREEHIGH:
    PARACHUTE.ManualDetectTrigger = AUX3_HIGH;
    break;

  case PARACHUTEAUXFOURLOW:
    PARACHUTE.ManualDetectTrigger = AUX4_LOW;
    break;

  case PARACHUTEAUXFOURMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX4_MID;
    break;

  case PARACHUTEAUXFOURHIGH:
    PARACHUTE.ManualDetectTrigger = AUX4_HIGH;
    break;

  case PARACHUTEAUXFIVELOW:
    PARACHUTE.ManualDetectTrigger = AUX5_LOW;
    break;

  case PARACHUTEAUXFIVEMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX5_MID;
    break;

  case PARACHUTEAUXFIVEHIGH:
    PARACHUTE.ManualDetectTrigger = AUX5_HIGH;
    break;

  case PARACHUTEAUXSIXLOW:
    PARACHUTE.ManualDetectTrigger = AUX6_LOW;
    break;

  case PARACHUTEAUXSIXMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX6_MID;
    break;

  case PARACHUTEAUXSIXHIGH:
    PARACHUTE.ManualDetectTrigger = AUX6_HIGH;
    break;

  case PARACHUTEAUXSEVENLOW:
    PARACHUTE.ManualDetectTrigger = AUX7_LOW;
    break;

  case PARACHUTEAUXSEVENMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX7_MID;
    break;

  case PARACHUTEAUXSEVENHIGH:
    PARACHUTE.ManualDetectTrigger = AUX7_HIGH;
    break;

  case PARACHUTEAUXEIGHTLOW:
    PARACHUTE.ManualDetectTrigger = AUX8_LOW;
    break;

  case PARACHUTEAUXEIGHTMIDDLE:
    PARACHUTE.ManualDetectTrigger = AUX8_MID;
    break;

  case PARACHUTEAUXEIGHTHIGH:
    PARACHUTE.ManualDetectTrigger = AUX8_HIGH;
    break;
  }
}

bool ParachuteClass::GetSafeStateToDisarmMotors()
{
  if (ParachuteDetectTrigger == 0)
  {
    return false;
  }
  if (PARACHUTE.Released())
  {
    return true;
  }
  return false;
}

bool ParachuteClass::Released()
{
  return (PARACHUTE.ParachuteReleased && PARACHUTE.ReleasedOverFlowTime());
}

bool ParachuteClass::ReleasedOverFlowTime()
{
  if (SCHEDULERTIME.GetMillis() - PARACHUTE.OverFlowTime >= MOTORS_DISARM_TIME)
  {
    return true;
  }
  return false;
}
