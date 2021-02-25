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

#include "THRCLIPPING.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Math/MATHSUPPORT.h"
#include "MotorsControl/MOTORS.h"
#include "PID/RCPID.h"
#include "Common/ENUM.h"

#define THROTTLE_CLIPPING_FACTOR 0.33f

static float MotorMixRange;

void Throttle_Clipping_Update(uint8_t MotorCount, int16_t MixerThrottleCommand)
{
    if (GetFrameStateOfMultirotor())
    {
        int16_t ThrottleRange = 0;
        int16_t ThrottleMin = 0;
        int16_t ThrottleMax = 0;
        int16_t MotorControlMax = 0;
        int16_t MotorControlMin = 0;

        for (uint8_t MotorIndex = 0; MotorIndex < MotorCount; MotorIndex++)
        {
            if (MotorControl[MotorIndex] > MotorControlMax)
            {
                MotorControlMax = MotorControl[MotorIndex];
            }

            if (MotorControl[MotorIndex] < MotorControlMin)
            {
                MotorControlMin = MotorControl[MotorIndex];
            }
        }

        int16_t MotorControlRange = MotorControlMax - MotorControlMin;

        ThrottleMin = AttitudeThrottleMin;
        ThrottleMax = AttitudeThrottleMax;

        ThrottleRange = ThrottleMax - ThrottleMin;

        MotorMixRange = (float)MotorControlRange / (float)ThrottleRange;
        if (MotorMixRange > 1.0f)
        {
            for (uint8_t MotorIndex = 0; MotorIndex < MotorCount; MotorIndex++)
            {
                MotorControl[MotorIndex] /= MotorMixRange;
            }

            ThrottleMin = ThrottleMin + (ThrottleRange / 2) - (ThrottleRange * THROTTLE_CLIPPING_FACTOR / 2);
            ThrottleMax = ThrottleMin + (ThrottleRange / 2) + (ThrottleRange * THROTTLE_CLIPPING_FACTOR / 2);
        }
        else
        {
            ThrottleMin = MIN(ThrottleMin + (MotorControlRange / 2), ThrottleMin + (ThrottleRange / 2) - (ThrottleRange * THROTTLE_CLIPPING_FACTOR / 2));
            ThrottleMax = MAX(ThrottleMax - (MotorControlRange / 2), ThrottleMin + (ThrottleRange / 2) + (ThrottleRange * THROTTLE_CLIPPING_FACTOR / 2));
        }

        for (uint8_t MotorIndex = 0; MotorIndex < MotorCount; MotorIndex++)
        {
            MotorControl[MotorIndex] = MotorControl[MotorIndex] + Constrain_16Bits(MixerThrottleCommand, ThrottleMin, ThrottleMax);
        }
    }
    else if (GetFrameStateOfAirPlane())
    {
        MotorControl[MOTOR1] = Constrain_16Bits(MixerThrottleCommand, AttitudeThrottleMin, AttitudeThrottleMax);
    }
}

bool MixerIsOutputSaturated(void)
{
    return MotorMixRange >= 1.0f;
}