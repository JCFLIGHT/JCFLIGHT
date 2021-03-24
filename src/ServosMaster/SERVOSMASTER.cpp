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

#include "SERVOSMASTER.h"
#include "AirPlane/AIRPLANE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"
#include "Common/ENUM.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "Filters/BIQUADFILTER.h"
#include "Scheduler/SCHEDULER.h"
#include "MotorsControl/MOTORS.h"
#include "Build/BOARDDEFS.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

ServosMasterClass SERVOSMASTER;

int16_t ServoScaleMin[MAX_SUPPORTED_SERVOS];
int16_t ServoScaleMax[MAX_SUPPORTED_SERVOS];

static BiquadFilter_Struct Smooth_Servo1_Aileron;
static BiquadFilter_Struct Smooth_Servo2_Aileron;
static BiquadFilter_Struct Smooth_Servo_Rudder;
static BiquadFilter_Struct Smooth_Servo_Elevator;

void ServosMasterClass::Initialization(void)
{
  if (SERVOSMASTER.LoadBiquadSettings())
  {
    SERVOSMASTER.UpdateMinAndMax();
    SERVOSMASTER.UpdateMiddlePoint();
    SERVOSMASTER.UpdateDirection();
  }
}

bool ServosMasterClass::LoadBiquadSettings(void)
{
  if (GetFrameStateOfMultirotor())
  {
    return false;
  }
  //ATUALIZA A FREQUENCIA DE CORTE DO LPF DOS SERVOS
  AIR_PLANE.Servo_LPF_CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
  BIQUADFILTER.Settings(&Smooth_Servo1_Aileron, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo2_Aileron, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Rudder, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Elevator, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  return true;
}

void ServosMasterClass::Rate_Update(void)
{
  //OBTÉM O RATE DOS SERVOS
  AIR_PLANE.ServoRate[SERVO1] = GET_SERVO_RATE(SERVO1_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO2] = GET_SERVO_RATE(SERVO2_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO3] = GET_SERVO_RATE(SERVO3_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO4] = GET_SERVO_RATE(SERVO4_RATE_ADDR);
}

void ServosMasterClass::UpdateMinAndMax()
{
  //OBTÉM O PULSO MINIMO DOS SERVOS
  AIR_PLANE.ServoMin[SERVO1] = GET_SERVO_MIN(SERVO1_MIN_ADDR);
  AIR_PLANE.ServoMin[SERVO2] = GET_SERVO_MIN(SERVO2_MIN_ADDR);
  AIR_PLANE.ServoMin[SERVO3] = GET_SERVO_MIN(SERVO3_MIN_ADDR);
  AIR_PLANE.ServoMin[SERVO4] = GET_SERVO_MIN(SERVO4_MIN_ADDR);

  //OBTÉM O PULSO MAXIMO DOS SERVOS
  AIR_PLANE.ServoMax[SERVO1] = GET_SERVO_MAX(SERVO1_MAX_ADDR);
  AIR_PLANE.ServoMax[SERVO2] = GET_SERVO_MAX(SERVO2_MAX_ADDR);
  AIR_PLANE.ServoMax[SERVO3] = GET_SERVO_MAX(SERVO3_MAX_ADDR);
  AIR_PLANE.ServoMax[SERVO4] = GET_SERVO_MAX(SERVO4_MAX_ADDR);
}

void ServosMasterClass::UpdateMiddlePoint(void)
{
  if (GET_SERVO_MIDDLE(SERVO1_MID_ADDR) == NONE)
  {
    SAVE_SERVO_MIDDLE(SERVO1_MID_ADDR, MIDDLE_STICKS_PULSE);
    SAVE_SERVO_MIDDLE(SERVO2_MID_ADDR, MIDDLE_STICKS_PULSE);
    SAVE_SERVO_MIDDLE(SERVO3_MID_ADDR, MIDDLE_STICKS_PULSE);
    SAVE_SERVO_MIDDLE(SERVO4_MID_ADDR, MIDDLE_STICKS_PULSE);
  }
  AIR_PLANE.ServoMiddle[SERVO1] = GET_SERVO_MIDDLE(SERVO1_MID_ADDR);
  AIR_PLANE.ServoMiddle[SERVO2] = GET_SERVO_MIDDLE(SERVO2_MID_ADDR);
  AIR_PLANE.ServoMiddle[SERVO3] = GET_SERVO_MIDDLE(SERVO3_MID_ADDR);
  AIR_PLANE.ServoMiddle[SERVO4] = GET_SERVO_MIDDLE(SERVO4_MID_ADDR);
}

void ServosMasterClass::UpdateDirection(void)
{
  CHECKSUM.UpdateServosReverse();
}

void ServosMasterClass::Rate_Apply()
{
  //CALCULA O RATE PARA OS SERVOS
  AIR_PLANE.ServoToFilter[SERVO1] = (((int32_t)AIR_PLANE.ServoRate[SERVO1] * AIR_PLANE.ServoToFilter[SERVO1]) / 100L); //AJUSTA O RATE DO SERVO 1
  AIR_PLANE.ServoToFilter[SERVO2] = (((int32_t)AIR_PLANE.ServoRate[SERVO2] * AIR_PLANE.ServoToFilter[SERVO2]) / 100L); //AJUSTA O RATE DO SERVO 2
  AIR_PLANE.ServoToFilter[SERVO3] = (((int32_t)AIR_PLANE.ServoRate[SERVO3] * AIR_PLANE.ServoToFilter[SERVO3]) / 100L); //AJUSTA O RATE DO SERVO 3
  AIR_PLANE.ServoToFilter[SERVO4] = (((int32_t)AIR_PLANE.ServoRate[SERVO4] * AIR_PLANE.ServoToFilter[SERVO4]) / 100L); //AJUSTA O RATE DO SERVO 4

  //AJUSTA O PONTO MÉDIO DOS SERVOS
  AIR_PLANE.ServoToFilter[SERVO1] += AIR_PLANE.ServoMiddle[SERVO1];
  AIR_PLANE.ServoToFilter[SERVO2] += AIR_PLANE.ServoMiddle[SERVO2];
  AIR_PLANE.ServoToFilter[SERVO3] += AIR_PLANE.ServoMiddle[SERVO3];
  AIR_PLANE.ServoToFilter[SERVO4] += AIR_PLANE.ServoMiddle[SERVO4];
}

void ServosMasterClass::Update(void)
{
  if (GetFrameStateOfMultirotor() || !SAFETYBUTTON.GetSafeStateToOutput())
  {
    return;
  }

  SERVOSMASTER.Rate_Apply();

  if (AIR_PLANE.Servo_LPF_CutOff == NONE)
  {
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(AIR_PLANE.ServoToFilter[SERVO1], AIR_PLANE.ServoMin[SERVO1], AIR_PLANE.ServoMax[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(AIR_PLANE.ServoToFilter[SERVO2], AIR_PLANE.ServoMin[SERVO2], AIR_PLANE.ServoMax[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(AIR_PLANE.ServoToFilter[SERVO3], AIR_PLANE.ServoMin[SERVO3], AIR_PLANE.ServoMax[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(AIR_PLANE.ServoToFilter[SERVO4], AIR_PLANE.ServoMin[SERVO4], AIR_PLANE.ServoMax[SERVO4]); //SERVO 4
  }
  else
  {
    //APLICA O LOW PASS FILTER NO SINAL DOS SERVOS
    AIR_PLANE.ServosFiltered[SERVO1] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo1_Aileron, AIR_PLANE.ServoToFilter[SERVO1]);
    AIR_PLANE.ServosFiltered[SERVO2] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo2_Aileron, AIR_PLANE.ServoToFilter[SERVO2]);
    AIR_PLANE.ServosFiltered[SERVO3] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo_Rudder, AIR_PLANE.ServoToFilter[SERVO3]);
    AIR_PLANE.ServosFiltered[SERVO4] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo_Elevator, AIR_PLANE.ServoToFilter[SERVO4]);

    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO1], AIR_PLANE.ServoMin[SERVO1], AIR_PLANE.ServoMax[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO2], AIR_PLANE.ServoMin[SERVO2], AIR_PLANE.ServoMax[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO3], AIR_PLANE.ServoMin[SERVO3], AIR_PLANE.ServoMax[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO4], AIR_PLANE.ServoMin[SERVO4], AIR_PLANE.ServoMax[SERVO4]); //SERVO 4
  }
}