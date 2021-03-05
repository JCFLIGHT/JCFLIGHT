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

#include "SERVOAUTOTRIM.h"
#include "PID/PIDXYZ.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AirPlane/AIRPLANE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "BitArray/BITARRAY.h"

#define SERVO_AUTOTRIM_OVERFLOW 2000
#define SAVE_OVERFLOW 2500

bool ServoAutoTrimEnabled;

int16_t ServoActualPulse[MAX_SUPPORTED_SERVOS];
int16_t ServoMiddleBackup[MAX_SUPPORTED_SERVOS];

int32_t ServoMiddleAccum[MAX_SUPPORTED_SERVOS];
int32_t ServoMiddleAccumCount;

uint32_t AutoTrimPreviousTime;
uint32_t SavePreviuosTime;

void ServosSaveAndUpdateMiddlePoint(void)
{
    SAVE_SERVO_MIDDLE(SERVO1_MID_ADDR, AIR_PLANE.ServoMiddle[SERVO1]);
    SAVE_SERVO_MIDDLE(SERVO2_MID_ADDR, AIR_PLANE.ServoMiddle[SERVO2]);
    SAVE_SERVO_MIDDLE(SERVO3_MID_ADDR, AIR_PLANE.ServoMiddle[SERVO3]);
    SAVE_SERVO_MIDDLE(SERVO4_MID_ADDR, AIR_PLANE.ServoMiddle[SERVO4]);
    AIR_PLANE.UpdateServosMiddlePoint();
}

void ServoAutoTrimRun(void)
{
    static ServoAutoTrimState_Enum ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;

    ServoActualPulse[SERVO1] = MotorControl[MOTOR2];
    ServoActualPulse[SERVO2] = MotorControl[MOTOR3];
    ServoActualPulse[SERVO3] = MotorControl[MOTOR4];
    ServoActualPulse[SERVO4] = MotorControl[MOTOR5];

    if (ServoAutoTrimEnabled)
    {
        switch (ServoAutoTrimState)
        {
        case SERVO_AUTOTRIM_IDLE:
            if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                {
                    ServoMiddleBackup[ServoIndex] = AIR_PLANE.ServoMiddle[ServoIndex];
                    ServoMiddleAccum[ServoIndex] = 0;
                }

                AutoTrimPreviousTime = SCHEDULERTIME.GetMillis();
                ServoMiddleAccumCount = 0;
                ServoAutoTrimState = SERVO_AUTOTRIM_COLLECTING;
            }
            else
            {
                break;
            }

        case SERVO_AUTOTRIM_COLLECTING:
            if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                ServoMiddleAccumCount++;

                for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                {
                    ServoMiddleAccum[ServoIndex] += ServoActualPulse[ServoIndex];
                }

                if ((SCHEDULERTIME.GetMillis() - AutoTrimPreviousTime) > SERVO_AUTOTRIM_OVERFLOW)
                {
                    for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
                    {
                        AIR_PLANE.ServoMiddle[ServoIndex] = ServoMiddleAccum[ServoIndex] / ServoMiddleAccumCount;
                    }
                    ServoAutoTrimState = SERVO_AUTOTRIM_SAVE_PENDING;
                    PIDXYZ.Reset_Integral_Accumulators();
                }
            }
            else
            {
                ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
            }
            break;

        case SERVO_AUTOTRIM_SAVE_PENDING:
            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
            {
                if ((SCHEDULERTIME.GetMillis() - SavePreviuosTime) > SAVE_OVERFLOW)
                {
                    ServosSaveAndUpdateMiddlePoint();
                    BEEPER.Play(BEEPER_ACTION_SUCCESS);
                    ServoAutoTrimState = SERVO_AUTOTRIM_DONE;
                }
            }
            else
            {
                SavePreviuosTime = SCHEDULERTIME.GetMillis();
            }
            break;

        case SERVO_AUTOTRIM_DONE:
            break;
        }
    }
    else
    {
        if (ServoAutoTrimState == SERVO_AUTOTRIM_SAVE_PENDING)
        {
            for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
            {
                AIR_PLANE.ServoMiddle[ServoIndex] = ServoMiddleBackup[ServoIndex];
            }
        }
        ServoAutoTrimState = SERVO_AUTOTRIM_IDLE;
    }
}