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

#include "STATES.h"
#include "RCCONFIG.h"
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"

#define THIS_LOOP_RATE 50 //Update()
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

bool StickStateToArm(void)
{
    return (Throttle.Output < 1100) &&
           (Yaw.Output > 1900) &&
           (Pitch.Output < 1100) &&
           (Roll.Output) < 1100;
}

bool StickStateToDisarm(void)
{
    return (Throttle.Output) < 1100 &&
           (Yaw.Output < 1100) &&
           (Pitch.Output < 1100) &&
           (Roll.Output > 1900);
}

bool SticksInAutoPilotPosition(int16_t AutoPilotValue)
{
    return ABS(RCController[ROLL]) < AutoPilotValue &&
           ABS(RCController[PITCH]) < AutoPilotValue;
}

bool SticksDeflected(int16_t MinDeflectionValue)
{
    return (ABS(RCController[ROLL]) > MinDeflectionValue) ||
           (ABS(RCController[PITCH]) > MinDeflectionValue);
}