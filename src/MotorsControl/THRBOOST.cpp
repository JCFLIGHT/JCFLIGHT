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

#include "THRBOOST.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Math/MATHSUPPORT.h"
#include "PID/RCPID.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//DEBUG
//#define PRINTLN_THR_BOOST

int16_t Get_Angle_Boost(int16_t Throttle_Value)
{
    float AHRS_Angles_Cosine = AHRS.GetCosinePitch() * AHRS.GetCosineRoll();
    AHRS_Angles_Cosine = Constrain_Float(AHRS_Angles_Cosine, 0.5f, 1.0f);
    AHRS_Angles_Cosine = Constrain_Float(9000 - MAX(labs(ATTITUDE.AngleOut[ROLL]), labs(ATTITUDE.AngleOut[PITCH])), 0, 3000) / (3000 * AHRS_Angles_Cosine);
    return Constrain_Float((float)(Throttle_Value - AttitudeThrottleMin) * AHRS_Angles_Cosine + AttitudeThrottleMin, AttitudeThrottleMin, MAX_STICKS_PULSE);
}

void Set_Throttle_Out(int16_t Throttle_Out, bool Apply_Angle_Boost)
{
    if (Apply_Angle_Boost)
    {
        RCController[THROTTLE] = Get_Angle_Boost(Throttle_Out);
    }
    else
    {
        RCController[THROTTLE] = Throttle_Out;
    }
}

void ApplyThrottleBoost()
{
    if (!GetFrameStateOfMultirotor()) //NÃO APLICA EM MODO AERO
    {
        return;
    }
    Set_Throttle_Out(RCController[THROTTLE], IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE)); //APLIQUE APENAS NO MODO STABILIZE
#ifdef PRINTLN_THR_BOOST
    PRINTF.SendToConsole(ProgramMemoryString("RCController[THROTTLE]:%d\n"), RCController[THROTTLE]);
#endif
}