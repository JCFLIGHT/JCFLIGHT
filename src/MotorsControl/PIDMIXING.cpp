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

#include "PIDMIXING.h"
#include "Common/VARIABLES.h"
#include "AirPlane/AIRPLANE.h"
#include "MOTORSCOMPENSATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "MIXTABLE.h"
#include "ProgMem/PROGMEM.h"

uint8_t NumberOfMotors = 4;
int16_t MixerThrottleCommand = 1000;
float ThrottleScale = 1.0f;

void MixingApplyPIDControl()
{

    MixerThrottleCommand = RCController[THROTTLE];
    MixerThrottleCommand = ((MixerThrottleCommand - AttitudeThrottleMin) * ThrottleScale) + AttitudeThrottleMin;
    Motors_Compensation(STORAGEMANAGER.Read_8Bits(MOTCOMP_STATE_ADDR), NumberOfMotors);

    switch (FrameType)
    {

    case QUAD_X:
    {
        //QUAD-X
        MotorControl[MOTOR1] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Quad_X[0].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Quad_X[0].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Quad_X[0].Yaw);
        MotorControl[MOTOR2] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Quad_X[1].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Quad_X[1].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Quad_X[1].Yaw);
        MotorControl[MOTOR3] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Quad_X[2].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Quad_X[2].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Quad_X[2].Yaw);
        MotorControl[MOTOR4] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Quad_X[3].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Quad_X[3].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Quad_X[3].Yaw);
        NumberOfMotors = ProgMemReadByte(&Motors_Count[QUAD_X].FrameMotorsCount);
        return;
    }

    case HEXA_X:
    {
        //HEXA-X
        MotorControl[MOTOR1] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[0].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[0].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[0].Yaw);
        MotorControl[MOTOR2] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[1].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[1].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[1].Yaw);
        MotorControl[MOTOR3] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[2].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[2].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[2].Yaw);
        MotorControl[MOTOR4] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[3].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[3].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[3].Yaw);
        MotorControl[MOTOR5] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[4].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[4].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[4].Yaw);
        MotorControl[MOTOR6] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[5].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[5].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_X[5].Yaw);
        NumberOfMotors = ProgMemReadByte(&Motors_Count[HEXA_X].FrameMotorsCount);
        return;
    }

    case HEXA_I:
    {
        //HEXA+
        MotorControl[MOTOR1] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[0].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[0].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[0].Yaw);
        MotorControl[MOTOR2] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[1].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[1].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[1].Yaw);
        MotorControl[MOTOR3] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[2].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[2].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[2].Yaw);
        MotorControl[MOTOR4] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[3].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[3].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[3].Yaw);
        MotorControl[MOTOR5] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[4].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[4].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[4].Yaw);
        MotorControl[MOTOR6] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[5].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[5].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_Hexa_I[5].Yaw);
        NumberOfMotors = ProgMemReadByte(&Motors_Count[HEXA_I].FrameMotorsCount);
        return;
    }

    case AIRPLANE:
    {
        //AEROMODELO
        AIR_PLANE.Mode_ConventionalPlane_Run();
        MotorControl[MOTOR6] = 1000;
        NumberOfMotors = ProgMemReadByte(&Motors_Count[AIRPLANE].FrameMotorsCount);
        return;
    }

    case FIXED_WING:
    {
        //ASA-FIXA
        AIR_PLANE.Mode_FixedWing_Run();
        MotorControl[MOTOR4] = 1000;
        MotorControl[MOTOR5] = 1000;
        MotorControl[MOTOR6] = 1000;
        NumberOfMotors = ProgMemReadByte(&Motors_Count[FIXED_WING].FrameMotorsCount);
        return;
    }

    case PLANE_VTAIL:
    {
        //AERO TIPO V-TAIL
        AIR_PLANE.Mode_PlaneVTail_Run();
        MotorControl[MOTOR6] = 1000;
        NumberOfMotors = ProgMemReadByte(&Motors_Count[PLANE_VTAIL].FrameMotorsCount);
        return;
    }

    case ZMR250:
    {
        //ZMR250
        MotorControl[MOTOR1] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_ZMR250[0].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_ZMR250[0].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_ZMR250[0].Yaw);
        MotorControl[MOTOR2] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_ZMR250[1].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_ZMR250[1].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_ZMR250[1].Yaw);
        MotorControl[MOTOR3] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_ZMR250[2].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_ZMR250[2].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_ZMR250[2].Yaw);
        MotorControl[MOTOR4] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_ZMR250[3].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_ZMR250[3].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_ZMR250[3].Yaw);
        NumberOfMotors = ProgMemReadByte(&Motors_Count[ZMR250].FrameMotorsCount);
        return;
    }

    case TBS:
    {
        //TEAM BLACK SHEEP
        MotorControl[MOTOR1] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_TBS[0].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_TBS[0].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_TBS[0].Yaw);
        MotorControl[MOTOR2] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_TBS[1].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_TBS[1].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_TBS[1].Yaw);
        MotorControl[MOTOR3] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_TBS[2].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_TBS[2].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_TBS[2].Yaw);
        MotorControl[MOTOR4] = MixerThrottleCommand + PIDControllerApply[ROLL] * ProgMemReadFloat(&Pid_Mixer_TBS[3].Roll) +
                               PIDControllerApply[PITCH] * ProgMemReadFloat(&Pid_Mixer_TBS[3].Pitch) + 1 * PIDControllerApply[YAW] * ProgMemReadFloat(&Pid_Mixer_TBS[3].Yaw);
        NumberOfMotors = ProgMemReadByte(&Motors_Count[TBS].FrameMotorsCount);
        return;
    }
    }
}