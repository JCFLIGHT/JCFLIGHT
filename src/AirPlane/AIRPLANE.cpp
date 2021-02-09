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
#include "Common/VARIABLES.h"
#ifndef __AVR_ATmega2560__
#include "Filters/LPFSERVO.h"
#else
#include "Filters/PT1.h"
#endif
#include "Math/MATHSUPPORT.h"
#include "SERVORATE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AirPlaneClass AIR_PLANE;

#ifndef __AVR_ATmega2560__
Struct_LowPassFilter ServosSmooth_LPF[MAX_SUPPORTED_SERVOS];
#else
PT1_Filter_Struct PT1_Servo1_Aileron;
PT1_Filter_Struct PT1_Servo2_Aileron;
PT1_Filter_Struct PT1_Servo_Rudder;
PT1_Filter_Struct PT1_Servo_Elevator;
#endif

#define LPF_SETPOINT 1500 //PONTO MEDIO DOS SERVOS PARA O FILTRO LPF

void AirPlaneClass::UpdateServosMinAndMax()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  //OBTÉM O PULSO MINIMO DOS SERVOS
  ServoMin[SERVO1] = GET_SERVO_MIN(SERVO1_MIN_ADDR);
  ServoMin[SERVO2] = GET_SERVO_MIN(SERVO2_MIN_ADDR);
  ServoMin[SERVO3] = GET_SERVO_MIN(SERVO3_MIN_ADDR);
  ServoMin[SERVO4] = GET_SERVO_MIN(SERVO4_MIN_ADDR);

  //OBTÉM O PULSO MAXIMO DOS SERVOS
  ServoMax[SERVO1] = GET_SERVO_MAX(SERVO1_MAX_ADDR);
  ServoMax[SERVO2] = GET_SERVO_MAX(SERVO2_MAX_ADDR);
  ServoMax[SERVO3] = GET_SERVO_MAX(SERVO3_MAX_ADDR);
  ServoMax[SERVO4] = GET_SERVO_MAX(SERVO4_MAX_ADDR);

  //ATUALIZA A FREQUENCIA DE CORTE DO LPF DOS SERVOS
  Servo_LPF_CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
}

void AirPlaneClass::UpdateServosMiddlePoint(void)
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  if (GET_SERVO_MIDDLE(SERVO1_MID_ADDR) == 0)
  {
    SAVE_SERVO_MIDDLE(SERVO1_MID_ADDR, 1500);
    SAVE_SERVO_MIDDLE(SERVO2_MID_ADDR, 1500);
    SAVE_SERVO_MIDDLE(SERVO3_MID_ADDR, 1500);
    SAVE_SERVO_MIDDLE(SERVO4_MID_ADDR, 1500);
  }
  ServoMiddle[SERVO1] = GET_SERVO_MIDDLE(SERVO1_MID_ADDR);
  ServoMiddle[SERVO2] = GET_SERVO_MIDDLE(SERVO2_MID_ADDR);
  ServoMiddle[SERVO3] = GET_SERVO_MIDDLE(SERVO3_MID_ADDR);
  ServoMiddle[SERVO4] = GET_SERVO_MIDDLE(SERVO4_MID_ADDR);
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
  if (FrameType != AIRPLANE)
  {
    return;
  }

  if (!COMMAND_ARM_DISARM)
  {
    MotorControl[MOTOR1] = 1000;
  }
  else
  {
    MotorControl[MOTOR1] = RCController[THROTTLE];
  }

  if (SetFlightModes[MANUAL_MODE]) //MODO MANUAL
  {
    ServoToFilter[SERVO1] = RCController[ROLL] * ServoDirection[0];  //WING     (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = RCController[ROLL] * ServoDirection[1];  //WING     (SERVO 2 DA ASA)
    ServoToFilter[SERVO3] = RCController[YAW] * ServoDirection[2];   //RUDDER   (LEME)
    ServoToFilter[SERVO4] = RCController[PITCH] * ServoDirection[3]; //ELEVATOR (PROFUNDOR)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    ServoToFilter[SERVO1] = PIDControllerApply[ROLL] * ServoDirection[0];  //WING     (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = PIDControllerApply[ROLL] * ServoDirection[1];  //WING     (SERVO 2 DA ASA)
    ServoToFilter[SERVO3] = PIDControllerApply[YAW] * ServoDirection[2];   //RUDDER   (LEME)
    ServoToFilter[SERVO4] = PIDControllerApply[PITCH] * ServoDirection[3]; //ELEVATOR (PROFUNDOR)
  }
}

void AirPlaneClass::Mode_FixedWing_Run()
{
  if (FrameType != FIXED_WING)
  {
    return;
  }

  if (!COMMAND_ARM_DISARM)
  {
    MotorControl[MOTOR1] = 1000;
  }
  else
  {
    MotorControl[MOTOR1] = RCController[THROTTLE];
  }

  if (SetFlightModes[MANUAL_MODE]) //MODO MANUAL
  {
    ServoToFilter[SERVO1] = (RCController[ROLL] * ServoDirection[0]) + (RCController[PITCH] * ServoDirection[0]); //WING (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = (RCController[ROLL] * ServoDirection[0]) - (RCController[PITCH] * ServoDirection[1]); //WING (SERVO 2 DA ASA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    ServoToFilter[SERVO1] = (PIDControllerApply[ROLL] * ServoDirection[0]) + (PIDControllerApply[PITCH] * ServoDirection[0]); //WING (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = (PIDControllerApply[ROLL] * ServoDirection[0]) - (PIDControllerApply[PITCH] * ServoDirection[1]); //WING (SERVO 2 DA ASA)
  }
}

void AirPlaneClass::Mode_PlaneVTail_Run()
{
  if (FrameType != PLANE_VTAIL)
  {
    return;
  }

  if (!COMMAND_ARM_DISARM)
  {
    MotorControl[MOTOR1] = 1000;
  }
  else
  {
    MotorControl[MOTOR1] = RCController[THROTTLE];
  }

  if (SetFlightModes[MANUAL_MODE]) //MODO MANUAL
  {
    ServoToFilter[SERVO1] = RCController[PITCH] * ServoDirection[0];                      //WING     (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = RCController[PITCH] * ServoDirection[1];                      //WING     (SERVO 2 DA ASA)
    ServoToFilter[SERVO3] = (RCController[ROLL] + RCController[YAW]) * ServoDirection[2]; //V-TAIL   (CAUDA)
    ServoToFilter[SERVO4] = (RCController[ROLL] - RCController[YAW]) * ServoDirection[3]; //V-TAIL   (CAUDA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    ServoToFilter[SERVO1] = PIDControllerApply[PITCH] * ServoDirection[0];                            //WING     (SERVO 1 DA ASA)
    ServoToFilter[SERVO2] = PIDControllerApply[PITCH] * ServoDirection[1];                            //WING     (SERVO 2 DA ASA)
    ServoToFilter[SERVO3] = (PIDControllerApply[ROLL] + PIDControllerApply[YAW]) * ServoDirection[2]; //V-TAIL   (CAUDA)
    ServoToFilter[SERVO4] = (PIDControllerApply[ROLL] - PIDControllerApply[YAW]) * ServoDirection[3]; //V-TAIL   (CAUDA)
  }
}

void AirPlaneClass::Servo_Rate_Adjust_And_Apply_LPF()
{
  if (GetFrameStateOfMultirotor() || !SAFETYBUTTON.GetSafeStateToOutput())
  {
    return;
  }

  Servo_Rate_Apply();

  if (Servo_LPF_CutOff == 0)
  {
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(ServoToFilter[SERVO1], ServoMin[SERVO1], ServoMax[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(ServoToFilter[SERVO2], ServoMin[SERVO2], ServoMax[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(ServoToFilter[SERVO3], ServoMin[SERVO3], ServoMax[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(ServoToFilter[SERVO4], ServoMin[SERVO4], ServoMax[SERVO4]); //SERVO 4
  }
  else
  {
//APLICA O LOW PASS FILTER NO SINAL DOS SERVOS
#ifndef __AVR_ATmega2560__
    ServosFiltered[SERVO1] = (int16_t)LowPassFilter(&ServosSmooth_LPF[SERVO1], ServoToFilter[SERVO1], Servo_LPF_CutOff, LPF_SETPOINT);
    ServosFiltered[SERVO2] = (int16_t)LowPassFilter(&ServosSmooth_LPF[SERVO2], ServoToFilter[SERVO2], Servo_LPF_CutOff, LPF_SETPOINT);
    ServosFiltered[SERVO3] = (int16_t)LowPassFilter(&ServosSmooth_LPF[SERVO3], ServoToFilter[SERVO3], Servo_LPF_CutOff, LPF_SETPOINT);
    ServosFiltered[SERVO4] = (int16_t)LowPassFilter(&ServosSmooth_LPF[SERVO4], ServoToFilter[SERVO4], Servo_LPF_CutOff, LPF_SETPOINT);
#else
    ServosFiltered[SERVO1] = (int16_t)PT1FilterApply(&PT1_Servo1_Aileron, ServoToFilter[SERVO1], Servo_LPF_CutOff, 1.0f / 1000);
    ServosFiltered[SERVO2] = (int16_t)PT1FilterApply(&PT1_Servo2_Aileron, ServoToFilter[SERVO2], Servo_LPF_CutOff, 1.0f / 1000);
    ServosFiltered[SERVO3] = (int16_t)PT1FilterApply(&PT1_Servo_Rudder, ServoToFilter[SERVO3], Servo_LPF_CutOff, 1.0f / 1000);
    ServosFiltered[SERVO4] = (int16_t)PT1FilterApply(&PT1_Servo_Elevator, ServoToFilter[SERVO4], Servo_LPF_CutOff, 1.0f / 1000);
#endif

    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(ServosFiltered[SERVO1], ServoMin[SERVO1], ServoMax[SERVO1]); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(ServosFiltered[SERVO2], ServoMin[SERVO2], ServoMax[SERVO2]); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(ServosFiltered[SERVO3], ServoMin[SERVO3], ServoMax[SERVO3]); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(ServosFiltered[SERVO4], ServoMin[SERVO4], ServoMax[SERVO4]); //SERVO 4
  }
}
