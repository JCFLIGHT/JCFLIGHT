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

#include "AIRPLANE.h"
#include "Filters/BIQUADFILTER.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "Math/MATHSUPPORT.h"
#include "ServosMaster/SERVORATE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "Common/RCDEFINES.h"
#include "PID/RCPID.h"
#include "PID/PIDXYZ.h"
#include "MotorsControl/MOTORS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AirPlaneClass AIR_PLANE;

static BiquadFilter_Struct Smooth_Servo1_Aileron;
static BiquadFilter_Struct Smooth_Servo2_Aileron;
static BiquadFilter_Struct Smooth_Servo_Rudder;
static BiquadFilter_Struct Smooth_Servo_Elevator;

void AirPlaneClass::LoadBiquadLPFSettings()
{
  BIQUADFILTER.Settings(&Smooth_Servo1_Aileron, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo2_Aileron, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Rudder, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
  BIQUADFILTER.Settings(&Smooth_Servo_Elevator, AIR_PLANE.Servo_LPF_CutOff, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), LPF);
}

void AirPlaneClass::UpdateServosMinAndMax()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

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

  //ATUALIZA A FREQUENCIA DE CORTE DO LPF DOS SERVOS
  AIR_PLANE.Servo_LPF_CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);

  //INICIALIZA O FILTRO LPF DO SERVOS
  AIR_PLANE.LoadBiquadLPFSettings();
}

void AirPlaneClass::UpdateServosMiddlePoint(void)
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

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

void AirPlaneClass::UpdateServosDirection(void)
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  CHECKSUM.UpdateServosReverse();
}

void AirPlaneClass::Mode_ConventionalPlane_Run()
{
  if (!GetActualFrameState(AIRPLANE))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = RCController[YAW] * AIR_PLANE.ServoDirection[SERVO3];   //RUDDER   (LEME)
    AIR_PLANE.ServoToFilter[SERVO4] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = PIDXYZ.PIDControllerApply[YAW] * AIR_PLANE.ServoDirection[SERVO3];   //RUDDER   (LEME)
    AIR_PLANE.ServoToFilter[SERVO4] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
}

void AirPlaneClass::Mode_FixedWing_Run()
{
  if (!GetActualFrameState(FIXED_WING))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = (RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) + (RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO1]); //AILERON (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = (RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) - (RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = (PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) + (PIDXYZ.PIDControllerApply[PITCH] * ServoDirection[SERVO1]); //AILERON (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = (PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) - (PIDXYZ.PIDControllerApply[PITCH] * ServoDirection[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
}

void AirPlaneClass::Mode_PlaneVTail_Run()
{
  if (!GetActualFrameState(PLANE_VTAIL))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO1];                      //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO2];                      //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = (RCController[ROLL] + RCController[YAW]) * AIR_PLANE.ServoDirection[SERVO3]; //V-TAIL   (CAUDA)
    AIR_PLANE.ServoToFilter[SERVO4] = (RCController[ROLL] - RCController[YAW]) * AIR_PLANE.ServoDirection[SERVO4]; //V-TAIL   (CAUDA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO1];                                   //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO2];                                   //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = (PIDXYZ.PIDControllerApply[ROLL] + PIDXYZ.PIDControllerApply[YAW]) * AIR_PLANE.ServoDirection[SERVO3]; //V-TAIL   (CAUDA)
    AIR_PLANE.ServoToFilter[SERVO4] = (PIDXYZ.PIDControllerApply[ROLL] - PIDXYZ.PIDControllerApply[YAW]) * AIR_PLANE.ServoDirection[SERVO4]; //V-TAIL   (CAUDA)
  }
}

void AirPlaneClass::Servo_Rate_Adjust_And_Apply_LPF()
{
  if (GetFrameStateOfMultirotor() || !SAFETYBUTTON.GetSafeStateToOutput())
  {
    return;
  }

  Servo_Rate_Apply();

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
    AIR_PLANE.ServosFiltered[SERVO1] = BIQUADFILTER.FilterApplyAndGet(&Smooth_Servo1_Aileron, AIR_PLANE.ServoToFilter[SERVO1]);
    AIR_PLANE.ServosFiltered[SERVO2] = BIQUADFILTER.FilterApplyAndGet(&Smooth_Servo2_Aileron, AIR_PLANE.ServoToFilter[SERVO2]);
    AIR_PLANE.ServosFiltered[SERVO3] = BIQUADFILTER.FilterApplyAndGet(&Smooth_Servo_Rudder, AIR_PLANE.ServoToFilter[SERVO3]);
    AIR_PLANE.ServosFiltered[SERVO4] = BIQUADFILTER.FilterApplyAndGet(&Smooth_Servo_Elevator, AIR_PLANE.ServoToFilter[SERVO4]);

    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO1], AIR_PLANE.ServoMin[SERVO1], AIR_PLANE.ServoMax[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO2], AIR_PLANE.ServoMin[SERVO2], AIR_PLANE.ServoMax[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO3], AIR_PLANE.ServoMin[SERVO3], AIR_PLANE.ServoMax[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(AIR_PLANE.ServosFiltered[SERVO4], AIR_PLANE.ServoMin[SERVO4], AIR_PLANE.ServoMax[SERVO4]); //SERVO 4
  }
}
