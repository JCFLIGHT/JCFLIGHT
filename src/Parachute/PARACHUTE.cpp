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
#include "Common/VARIABLES.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Scheduler/SCHEDULERTIME.h"

ParachuteClass PARACHUTE;

#define MOTORS_DISARM_TIME 40 //MS - DESARMA OS MOTORES APÓS DETECTAR QUE O PARACHUTE FOI LANÇADO

void ParachuteClass::Auto_Do_Now(bool ActiveParachute)
{
  if (!ActiveParachute)
  {
    MotorControl[PARACHUTESERVO] = 400; //0 GRAUS
    ParachuteReleased = false;
    return;
  }
  ParachuteInAuto = true;
  MotorControl[PARACHUTESERVO] = 2400; //180 GRAUS
  ParachuteReleased = true;
}

void ParachuteClass::Manual_Do_Now()
{
  if (!ParachuteReleased)
    OverFlowTime += SCHEDULER.GetMillis();
  if (ParachuteInAuto)
    return;
  if (!ManualDetectTrigger)
  {
    MotorControl[PARACHUTESERVO] = 400; //0 GRAUS
    ParachuteReleased = false;
    return;
  }
  MotorControl[PARACHUTESERVO] = 2400; //180 GRAUS
  ParachuteReleased = true;
}

void ParachuteClass::Manual_Detect_Channel()
{
  switch (ParachuteDetectTrigger)
  {

  case PARACHUTEAUXONELOW:
    ManualDetectTrigger = AUX1_LOW;
    break;

  case PARACHUTEAUXONEMIDDLE:
    ManualDetectTrigger = AUX1_MID;
    break;

  case PARACHUTEAUXONEHIGH:
    ManualDetectTrigger = AUX1_HIGH;
    break;

  case PARACHUTEAUXTWOLOW:
    ManualDetectTrigger = AUX2_LOW;
    break;

  case PARACHUTEAUXTWOMIDDLE:
    ManualDetectTrigger = AUX2_MID;
    break;

  case PARACHUTEAUXTWOHIGH:
    ManualDetectTrigger = AUX2_HIGH;
    break;

  case PARACHUTEAUXTHREELOW:
    ManualDetectTrigger = AUX3_LOW;
    break;

  case PARACHUTEAUXTHREEMIDDLE:
    ManualDetectTrigger = AUX3_MID;
    break;

  case PARACHUTEAUXTHREEHIGH:
    ManualDetectTrigger = AUX3_HIGH;
    break;

  case PARACHUTEAUXFOURLOW:
    ManualDetectTrigger = AUX4_LOW;
    break;

  case PARACHUTEAUXFOURMIDDLE:
    ManualDetectTrigger = AUX4_MID;
    break;

  case PARACHUTEAUXFOURHIGH:
    ManualDetectTrigger = AUX4_HIGH;
    break;

  case PARACHUTEAUXFIVELOW:
    ManualDetectTrigger = AUX5_LOW;
    break;

  case PARACHUTEAUXFIVEMIDDLE:
    ManualDetectTrigger = AUX5_MID;
    break;

  case PARACHUTEAUXFIVEHIGH:
    ManualDetectTrigger = AUX5_HIGH;
    break;

  case PARACHUTEAUXSIXLOW:
    ManualDetectTrigger = AUX6_LOW;
    break;

  case PARACHUTEAUXSIXMIDDLE:
    ManualDetectTrigger = AUX6_MID;
    break;

  case PARACHUTEAUXSIXHIGH:
    ManualDetectTrigger = AUX6_HIGH;
    break;

  case PARACHUTEAUXSEVENLOW:
    ManualDetectTrigger = AUX7_LOW;
    break;

  case PARACHUTEAUXSEVENMIDDLE:
    ManualDetectTrigger = AUX7_MID;
    break;

  case PARACHUTEAUXSEVENHIGH:
    ManualDetectTrigger = AUX7_HIGH;
    break;

  case PARACHUTEAUXEIGHTLOW:
    ManualDetectTrigger = AUX8_LOW;
    break;

  case PARACHUTEAUXEIGHTMIDDLE:
    ManualDetectTrigger = AUX8_MID;
    break;

  case PARACHUTEAUXEIGHTHIGH:
    ManualDetectTrigger = AUX8_HIGH;
    break;
  }
}

bool ParachuteClass::GetSafeStateToDisarmMotors()
{
  if (ParachuteDetectTrigger == 0)
    return false;
  if (PARACHUTE.Released())
    return true;
  return false;
}

bool ParachuteClass::Released()
{
  return (ParachuteReleased && ReleasedOverFlowTime());
}

bool ParachuteClass::ReleasedOverFlowTime()
{
  if (SCHEDULER.GetMillis() - OverFlowTime >= MOTORS_DISARM_TIME)
    return true;
  return false;
}
