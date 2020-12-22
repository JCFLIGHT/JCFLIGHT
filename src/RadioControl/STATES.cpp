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
#include "Math/AVRMATH.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"

bool CheckInclinationForCopter(void)
{
    if (GetFrameStateOfAirPlane())
        return false; //PULA A CHECAGEM DE INCLINAÇÃO NO MODO PLANE
    if (GetFrameStateOfMultirotor() && CheckAnglesInclination(25))
        return true; //INVALIDA O ARMAMENTO DO SISTEMA SE HOUVER INCLINAÇÃO NOS EIXOS
    return false;    //INCLINAÇÃO NÃO DETECTADA
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
           (Pitch.Output > 1900) &&
           (Roll.Output < 1100);
}

bool SticksInAutoPilotPosition(int16_t AutoPilotValue)
{
    return ABS_16BITS(RCController[ROLL]) < AutoPilotValue && ABS_16BITS(RCController[PITCH]) < AutoPilotValue;
}

bool SticksDeflected(int16_t MinDeflectionValue)
{
    return (ABS_16BITS(RCController[ROLL]) > MinDeflectionValue) || (ABS_16BITS(RCController[PITCH]) > MinDeflectionValue);
}