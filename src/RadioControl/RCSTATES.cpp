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

#include "RCSTATES.h"
#include "RCCONFIG.h"
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/RCDEFINES.h"

#define THIS_LOOP_RATE 50 //STICKS.Update()
#define ARM_TIME_MAX 2    //SEGUNDOS
#define DISARM_TIME_MAX 2 //SEGUNDOS

int16_t ArmDelayedCount = 0;
int16_t DisarmDelayedCount = 0;

bool CheckInclinationForArm(void)
{
    if (GetFrameStateOfAirPlane())
    {
        return false; //PULA A CHECAGEM DE INCLINAÇÃO NO MODO PLANE
    }
    if (GetFrameStateOfMultirotor() && AHRS.CheckAnglesInclination(25))
    {
        return true; //INVALIDA O ARMAMENTO DO SISTEMA SE HOUVER INCLINAÇÃO NOS EIXOS
    }
    return false; //INCLINAÇÃO NÃO DETECTADA
}

bool ArmDelayedState(void)
{
    if (ArmDelayedCount >= (THIS_LOOP_RATE * ARM_TIME_MAX))
    {
        return true;
    }
    else
    {
        ArmDelayedCount++;
    }
    return false;
}

void ResetArmDelayed(void)
{
    ArmDelayedCount = 0;
}

bool DisarmDelayedState(void)
{
    if (DisarmDelayedCount >= (THIS_LOOP_RATE * DISARM_TIME_MAX))
    {
        return true;
    }
    else
    {
        DisarmDelayedCount++;
    }
    return false;
}

void ResetDisarmDelayed(void)
{
    DisarmDelayedCount = 0;
}

bool SticksStateToArm(void)
{
    return (Throttle.Output < MIN_PULSE) && (Yaw.Output > MAX_PULSE) &&
           (Pitch.Output < MIN_PULSE) && (Roll.Output) < MIN_PULSE;
}

bool SticksStateToDisarm(void)
{
    return (Throttle.Output) < MIN_PULSE && (Yaw.Output < MIN_PULSE) &&
           (Pitch.Output < MIN_PULSE) && (Roll.Output > MAX_PULSE);
}

bool SticksInAutoPilotPosition(int16_t AutoPilotValue)
{
    return ABS(RCController[ROLL]) < AutoPilotValue && ABS(RCController[PITCH]) < AutoPilotValue;
}

bool SticksDeflected(int16_t MinDeflectionValue)
{
    return (ABS(RCController[ROLL]) > MinDeflectionValue) || (ABS(RCController[PITCH]) > MinDeflectionValue);
}

bool GetActualThrottleStatus(uint8_t ThrottleStatus)
{
    if (RadioControllOutput[THROTTLE] <= MIN_PULSE && ThrottleStatus == THROTTLE_LOW)
    {
        return true;
    }
    else if (RadioControllOutput[THROTTLE] >= MAX_PULSE && ThrottleStatus == THROTTLE_HIGH)
    {
        return true;
    }
    else if (RadioControllOutput[THROTTLE] >= (MIDDLE_STICKS_PULSE - 100) &&
             RadioControllOutput[THROTTLE] <= (MIDDLE_STICKS_PULSE + 100) &&
             ThrottleStatus == THROTTLE_MIDDLE)
    {
        return true;
    }
    return false;
}