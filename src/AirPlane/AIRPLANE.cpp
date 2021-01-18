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
#include "Filters/LPFSERVO.h"
#include "Math/MATHSUPPORT.h"
#include "SERVOMANUALTRIM.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "SafetyButton/SAFETYBUTTON.h"

Struct_LowPassFilter LPFDevice[4]; //INSTANCIA PARA APLICAR O FILTRO LPF NOS SERVOS

#define PULSE_MIN 500                                                                     //PULSO MINIMO PARA OS SERVOS
#define PULSE_MIDDLE 1500                                                                 //PULSO MEDIO PARA OS SERVOS
#define PULSE_MAX 2200                                                                    //PULSO MAXIMO PARA OS SERVOS
#define LPF_SETPOINT 1500                                                                 //PONTO MEDIO DOS SERVOS PARA O FILTRO
#define GET_SERVO_DIRECTION(Address) ((STORAGEMANAGER.Read_8Bits(Address) == 1) ? -1 : 1) //OBTÉM A DIREÇÃO DOS SERVOS

int8_t ServoDirection[4] = {1, 1, 1, 1}; //CONTROLE DE DIREÇÃO DOS SERVOS
int8_t ServoRate[4];                     //AJUSTE DE RATE DOS SERVOS DE -127 - +127
int16_t ServoToFilter[4];                //SINAL DO SERVO PARA APLICAR NO FILTRO
int16_t DeviceFiltered[4];               //SAIDA FILTRADADA DOS PULSOS PWM DOS SERVOS

void UpdateServosDirection(void)
{
  ServoDirection[SERVO1] = GET_SERVO_DIRECTION(SERVO1_DIRECTION_ADDR);
  ServoDirection[SERVO2] = GET_SERVO_DIRECTION(SERVO2_DIRECTION_ADDR);
  ServoDirection[SERVO3] = GET_SERVO_DIRECTION(SERVO3_DIRECTION_ADDR);
  ServoDirection[SERVO4] = GET_SERVO_DIRECTION(SERVO4_DIRECTION_ADDR);
}

void AirPlane_Mode_ConventionalPlane_Run()
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
  if (!OkToTrimServo)
  {
    if (Do_IOC_Mode) //MODO MANUAL
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
}

void AirPlane_Mode_FixedWing_Run()
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
  if (!OkToTrimServo)
  {
    if (Do_IOC_Mode) //MODO MANUAL
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
}

void AirPlane_Mode_PlaneVTail_Run()
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
  if (!OkToTrimServo)
  {
    if (Do_IOC_Mode) //MODO MANUAL
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
}

void Servo_Rate_Adjust()
{
  if (GetFrameStateOfMultirotor() || !SAFETYBUTTON.GetSafeStateToOutput())
  {
    return;
  }
  //CALCULA O RATE PARA OS SERVOS
  ServoToFilter[SERVO1] = (((int32_t)ServoRate[SERVO1] * ServoToFilter[SERVO1]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 1
  ServoToFilter[SERVO2] = (((int32_t)ServoRate[SERVO2] * ServoToFilter[SERVO2]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 2
  ServoToFilter[SERVO3] = (((int32_t)ServoRate[SERVO3] * ServoToFilter[SERVO3]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 3
  ServoToFilter[SERVO4] = (((int32_t)ServoRate[SERVO4] * ServoToFilter[SERVO4]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 4

  int16_t Servo_LPF_CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);

  if (Servo_LPF_CutOff == 0)
  {
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(ServoToFilter[SERVO1], PULSE_MIN, PULSE_MAX); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(ServoToFilter[SERVO2], PULSE_MIN, PULSE_MAX); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(ServoToFilter[SERVO3], PULSE_MIN, PULSE_MAX); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(ServoToFilter[SERVO4], PULSE_MIN, PULSE_MAX); //SERVO 4
  }
  else
  {
    //APLICA O LOW PASS FILTER NO SINAL DOS SERVOS
    DeviceFiltered[SERVO1] = (int16_t)LowPassFilter(&LPFDevice[SERVO1], ServoToFilter[SERVO1], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO2] = (int16_t)LowPassFilter(&LPFDevice[SERVO2], ServoToFilter[SERVO2], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO3] = (int16_t)LowPassFilter(&LPFDevice[SERVO3], ServoToFilter[SERVO3], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO4] = (int16_t)LowPassFilter(&LPFDevice[SERVO4], ServoToFilter[SERVO4], Servo_LPF_CutOff, LPF_SETPOINT);
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR2] = Constrain_16Bits(DeviceFiltered[SERVO1], PULSE_MIN, PULSE_MAX); //SERVO 1
    MotorControl[MOTOR3] = Constrain_16Bits(DeviceFiltered[SERVO2], PULSE_MIN, PULSE_MAX); //SERVO 2
    MotorControl[MOTOR4] = Constrain_16Bits(DeviceFiltered[SERVO3], PULSE_MIN, PULSE_MAX); //SERVO 3
    MotorControl[MOTOR5] = Constrain_16Bits(DeviceFiltered[SERVO4], PULSE_MIN, PULSE_MAX); //SERVO 4
  }
}
