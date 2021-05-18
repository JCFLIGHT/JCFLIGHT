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
#include "SERVOSAUTOTRIM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

ServosMasterClass SERVOSMASTER;
Servo_Struct Servo;

#ifndef __AVR_ATmega2560__

typedef struct ServoSpeedLimit
{
  float State;
} ServoSpeedLimit_Stuct;

static BiquadFilter_Struct Smooth_Servo1_Aileron;
static BiquadFilter_Struct Smooth_Servo2_Aileron;
static BiquadFilter_Struct Smooth_Servo_Rudder;
static BiquadFilter_Struct Smooth_Servo_Elevator;

static ServoSpeedLimit_Stuct ServoSpeedLimit[MAX_SUPPORTED_SERVOS];
#endif

void ServosMasterClass::Initialization(void)
{
  if (SERVOSMASTER.LoadBiquadSettings())
  {
    SERVOSMASTER.UpdateMinAndMax();
    SERVOSMASTER.UpdateMiddlePoint();
    SERVOSMASTER.UpdateDirection();
    SERVOSMASTER.Rate_Update();
  }
}

bool ServosMasterClass::LoadBiquadSettings(void)
{
  if (GetMultirotorEnabled())
  {
    return false;
  }
#ifndef __AVR_ATmega2560__
  //ATUALIZA A FREQUENCIA DE CORTE DO LPF DOS SERVOS
  Servo.Filter.CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
  BIQUADFILTER.Settings(&Smooth_Servo1_Aileron, Servo.Filter.CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo2_Aileron, Servo.Filter.CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Rudder, Servo.Filter.CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Elevator, Servo.Filter.CutOff, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US), LPF);
#endif
  return true;
}

void ServosMasterClass::Rate_Update(void)
{
  //OBTÉM O RATE DOS SERVOS
  Servo.Rate.GetAndSet[SERVO1] = GET_SERVO_RATE(SERVO1_RATE_ADDR);
  Servo.Rate.GetAndSet[SERVO2] = GET_SERVO_RATE(SERVO2_RATE_ADDR);
  Servo.Rate.GetAndSet[SERVO3] = GET_SERVO_RATE(SERVO3_RATE_ADDR);
  Servo.Rate.GetAndSet[SERVO4] = GET_SERVO_RATE(SERVO4_RATE_ADDR);
}

void ServosMasterClass::UpdateMinAndMax(void)
{
  //OBTÉM O PULSO MINIMO DOS SERVOS
  Servo.Pulse.Min[SERVO1] = GET_SERVO_MIN(SERVO1_MIN_ADDR);
  Servo.Pulse.Min[SERVO2] = GET_SERVO_MIN(SERVO2_MIN_ADDR);
  Servo.Pulse.Min[SERVO3] = GET_SERVO_MIN(SERVO3_MIN_ADDR);
  Servo.Pulse.Min[SERVO4] = GET_SERVO_MIN(SERVO4_MIN_ADDR);

  //OBTÉM O PULSO MAXIMO DOS SERVOS
  Servo.Pulse.Max[SERVO1] = GET_SERVO_MAX(SERVO1_MAX_ADDR);
  Servo.Pulse.Max[SERVO2] = GET_SERVO_MAX(SERVO2_MAX_ADDR);
  Servo.Pulse.Max[SERVO3] = GET_SERVO_MAX(SERVO3_MAX_ADDR);
  Servo.Pulse.Max[SERVO4] = GET_SERVO_MAX(SERVO4_MAX_ADDR);
}

void ServosMasterClass::UpdateMiddlePoint(void)
{
  Servo.Pulse.Middle[SERVO1] = GET_SERVO_MIDDLE(SERVO1_MID_ADDR);
  Servo.Pulse.Middle[SERVO2] = GET_SERVO_MIDDLE(SERVO2_MID_ADDR);
  Servo.Pulse.Middle[SERVO3] = GET_SERVO_MIDDLE(SERVO3_MID_ADDR);
  Servo.Pulse.Middle[SERVO4] = GET_SERVO_MIDDLE(SERVO4_MID_ADDR);
}

void ServosMasterClass::UpdateDirection(void)
{
  CHECKSUM.UpdateServosReverse();
}

#ifndef __AVR_ATmega2560__

float ServoSpeedLimitApply(ServoSpeedLimit_Stuct *ServoLimitPointer, float Input, float RateLimit, float DeltaTime)
{
  if (RateLimit > 0)
  {
    const float RateLimitPerSample = RateLimit * DeltaTime;
    ServoLimitPointer->State = Constrain_Float(Input, ServoLimitPointer->State - RateLimitPerSample, ServoLimitPointer->State + RateLimitPerSample);
  }
  else
  {
    ServoLimitPointer->State = Input;
  }
  return ServoLimitPointer->State;
}

#endif

void ServosMasterClass::Rate_Apply(void)
{
#ifndef __AVR_ATmega2560__

/*
 0 = NENHUM LIMITE
 1 = 10 SEGUNDOS
 10 = 10 SEGUNDOS
 100 = 1 SEGUNDO

 POR EXEMPLO:
 EM 100,O SERVO IRÁ LEVAR 1 SEGUNDO PARA IR DE UM PONTO A OUTRO,POR EXEMPLO DE 0° A 180°
*/

#define DEFAULT_SERVO_SPEED 0
#define DELTA_TIME_VIRTUAL 1000 * 1e-6f //APENAS PARA TESTE

  int16_t ServoSignalLimited[MAX_SUPPORTED_SERVOS];

  for (uint8_t ServoIndex = SERVO1; ServoIndex < MAX_SUPPORTED_SERVOS; ServoIndex++)
  {
    ServoSignalLimited[ServoIndex] = (int16_t)ServoSpeedLimitApply(&ServoSpeedLimit[ServoIndex],
                                                                   Servo.Signal.UnFiltered[ServoIndex],
                                                                   DEFAULT_SERVO_SPEED * 10,
                                                                   DELTA_TIME_VIRTUAL);

    Servo.Signal.UnFiltered[ServoIndex] = ServoSignalLimited[ServoIndex];
  }

#endif

  //CALCULA O RATE PARA OS SERVOS
  Servo.Signal.UnFiltered[SERVO1] = (((int32_t)Servo.Rate.GetAndSet[SERVO1] * Servo.Signal.UnFiltered[SERVO1]) / 100L); //AJUSTA O RATE DO SERVO 1
  Servo.Signal.UnFiltered[SERVO2] = (((int32_t)Servo.Rate.GetAndSet[SERVO2] * Servo.Signal.UnFiltered[SERVO2]) / 100L); //AJUSTA O RATE DO SERVO 2
  Servo.Signal.UnFiltered[SERVO3] = (((int32_t)Servo.Rate.GetAndSet[SERVO3] * Servo.Signal.UnFiltered[SERVO3]) / 100L); //AJUSTA O RATE DO SERVO 3
  Servo.Signal.UnFiltered[SERVO4] = (((int32_t)Servo.Rate.GetAndSet[SERVO4] * Servo.Signal.UnFiltered[SERVO4]) / 100L); //AJUSTA O RATE DO SERVO 4

  //AJUSTA O PONTO MÉDIO DOS SERVOS
  Servo.Signal.UnFiltered[SERVO1] += Servo.Pulse.Middle[SERVO1];
  Servo.Signal.UnFiltered[SERVO2] += Servo.Pulse.Middle[SERVO2];
  Servo.Signal.UnFiltered[SERVO3] += Servo.Pulse.Middle[SERVO3];
  Servo.Signal.UnFiltered[SERVO4] += Servo.Pulse.Middle[SERVO4];
}

void ServosMasterClass::Update(void)
{
  if (GetMultirotorEnabled() || !SAFETYBUTTON.GetSafeStateToOutput())
  {
    return;
  }

  AIRPLANE.Update_Conventional_AirPlane();
  AIRPLANE.Update_FixedWing();
  AIRPLANE.Update_AirPlaneVTail();

  SERVOSMASTER.Rate_Apply();

#ifndef __AVR_ATmega2560__
  if (Servo.Filter.CutOff == NONE)
#endif
  {
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(Servo.Signal.UnFiltered[SERVO1], Servo.Pulse.Min[SERVO1], Servo.Pulse.Max[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(Servo.Signal.UnFiltered[SERVO2], Servo.Pulse.Min[SERVO2], Servo.Pulse.Max[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(Servo.Signal.UnFiltered[SERVO3], Servo.Pulse.Min[SERVO3], Servo.Pulse.Max[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(Servo.Signal.UnFiltered[SERVO4], Servo.Pulse.Min[SERVO4], Servo.Pulse.Max[SERVO4]); //SERVO 4
  }
#ifndef __AVR_ATmega2560__
  else
  {
    //APLICA O LOW PASS FILTER NO SINAL DOS SERVOS
    Servo.Signal.Filtered[SERVO1] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo1_Aileron, Servo.Signal.UnFiltered[SERVO1]);
    Servo.Signal.Filtered[SERVO2] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo2_Aileron, Servo.Signal.UnFiltered[SERVO2]);
    Servo.Signal.Filtered[SERVO3] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo_Rudder, Servo.Signal.UnFiltered[SERVO3]);
    Servo.Signal.Filtered[SERVO4] = BIQUADFILTER.ApplyAndGet(&Smooth_Servo_Elevator, Servo.Signal.UnFiltered[SERVO4]);

    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(Servo.Signal.Filtered[SERVO1], Servo.Pulse.Min[SERVO1], Servo.Pulse.Max[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(Servo.Signal.Filtered[SERVO2], Servo.Pulse.Min[SERVO2], Servo.Pulse.Max[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(Servo.Signal.Filtered[SERVO3], Servo.Pulse.Min[SERVO3], Servo.Pulse.Max[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(Servo.Signal.Filtered[SERVO4], Servo.Pulse.Min[SERVO4], Servo.Pulse.Max[SERVO4]); //SERVO 4
  }
#endif
  ServoAutoTrimRun();
}