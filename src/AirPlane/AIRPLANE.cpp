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
#include "Math/AVRMATH.h"
#include "SERVOTRIM.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "FastSerial/PRINTF.h"

#define PULSE_MIN 500     //PULSO MINIMO PARA OS SERVOS
#define PULSE_MIDDLE 1500 //PULSO MEDIO PARA OS SERVOS
#define PULSE_MAX 2200    //PULSO MAXIMO PARA OS SERVOS
#define LPF_SETPOINT 1500 //PONTO MEDIO DOS SERVOS PARA O FILTRO

//DEBUG
//#define PRINTLN_SERVO_SIGNAL

Struct_LowPassFilter LPFDevice[4]; //INSTANCIA PARA APLICAR O FILTRO LPF NOS SERVOS

int8_t FlapperonsDirection[2] = {-1, 1}; //CONTROLE DOS 2 SERVOS DA ASA,O SEGUNDO SERVO TEM O MOVIMENTO AO CONTRARIO DO PRIMEIRO
int8_t ServoRate[4];                     //AJUSTE DE RATE DOS SERVOS DE -127 - +127
int16_t DeviceFiltered[4];               //SAIDA FILTRADADA DOS PULSOS PWM DOS SERVOS

void AirPlane_Mode_ConventionalPlane_Run()
{
  if (FrameType != 3)
    return;
  if (!COMMAND_ARM_DISARM)
    MotorControl[MOTOR4] = 1000;
  else
    MotorControl[MOTOR4] = RCController[THROTTLE];
  if (!OkToTrimServo)
  {
    if (IOCMODE) //MANUAL
    {
      MotorControl[MOTOR3] = RCController[PITCH] * FlapperonsDirection[0]; //WING     (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = RCController[PITCH] * FlapperonsDirection[1]; //WING     (SERVO 2 DA ASA)
      MotorControl[MOTOR1] = RCController[YAW];                            //RUDDER   (LEME)
      MotorControl[MOTOR6] = RCController[ROLL];                           //ELEVATOR (PROFUNDOR)
    }
    else //STABILIZE OU ACRO
    {
      //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
      MotorControl[MOTOR3] = PIDControllerApply[PITCH] * FlapperonsDirection[0]; //WING     (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = PIDControllerApply[PITCH] * FlapperonsDirection[1]; //WING     (SERVO 2 DA ASA)
      MotorControl[MOTOR1] = PIDControllerApply[YAW];                            //RUDDER   (LEME)
      MotorControl[MOTOR6] = PIDControllerApply[ROLL];                           //ELEVATOR (PROFUNDOR)
    }
  }
}

void AirPlane_Mode_FixedWing_Run()
{
  if (FrameType != 4)
    return;
  if (!COMMAND_ARM_DISARM)
    MotorControl[MOTOR4] = 1000;
  else
    MotorControl[MOTOR4] = RCController[THROTTLE];
  if (!OkToTrimServo)
  {
    if (IOCMODE) //MANUAL
    {
      MotorControl[MOTOR3] = (RCController[PITCH] * FlapperonsDirection[0]) + (RCController[ROLL] * FlapperonsDirection[1]); //WING (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = (RCController[PITCH] * FlapperonsDirection[0]) - (RCController[ROLL] * FlapperonsDirection[1]); //WING (SERVO 2 DA ASA)
    }
    else //STABILIZE OU ACRO
    {
      //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
      MotorControl[MOTOR3] = (PIDControllerApply[PITCH] * FlapperonsDirection[0]) + (PIDControllerApply[ROLL] * FlapperonsDirection[1]); //WING (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = (PIDControllerApply[PITCH] * FlapperonsDirection[0]) - (PIDControllerApply[ROLL] * FlapperonsDirection[1]); //WING (SERVO 2 DA ASA)
    }
  }
}

void AirPlane_Mode_PlaneVTail_Run()
{
  if (FrameType != 5)
    return;
  if (!COMMAND_ARM_DISARM)
    MotorControl[MOTOR4] = 1000;
  else
    MotorControl[MOTOR4] = RCController[THROTTLE];
  if (!OkToTrimServo)
  {
    if (IOCMODE) //MANUAL
    {
      MotorControl[MOTOR3] = RCController[ROLL] * FlapperonsDirection[0]; //WING     (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = RCController[ROLL] * FlapperonsDirection[1]; //WING     (SERVO 2 DA ASA)
      MotorControl[MOTOR1] = RCController[PITCH] + RCController[YAW];     //V-TAIL   (CAUDA)
      MotorControl[MOTOR6] = RCController[PITCH] - RCController[YAW];     //V-TAIL   (CAUDA)
    }
    else //STABILIZE OU ACRO
    {
      //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
      MotorControl[MOTOR3] = PIDControllerApply[ROLL] * FlapperonsDirection[0];   //WING     (SERVO 1 DA ASA)
      MotorControl[MOTOR2] = PIDControllerApply[ROLL] * FlapperonsDirection[1];   //WING     (SERVO 2 DA ASA)
      MotorControl[MOTOR1] = PIDControllerApply[PITCH] + PIDControllerApply[YAW]; //V-TAIL   (CAUDA)
      MotorControl[MOTOR6] = PIDControllerApply[PITCH] - PIDControllerApply[YAW]; //V-TAIL   (CAUDA)
    }
  }
}

void Servo_Rate_Adjust()
{
  if (FrameType < 3 || FrameType == 6 || FrameType == 7)
    return;
  //CASO OS SERVOS INICIEM COM O PULSO MINIMO É POR QUE O SERVO TRIM NÃO FOI FEITO,OS VALORES GUARDADOS NA EEPROM SERÃO 0 (ZERO),ZERO É IGUAL A -127 NO TRIM
  //RATE PARA OS SERVOS
  //SERVO 1
  MotorControl[MOTOR3] = (((int32_t)ServoRate[SERVO1] * MotorControl[MOTOR3]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 1
  //SERVO 2
  MotorControl[MOTOR2] = (((int32_t)ServoRate[SERVO2] * MotorControl[MOTOR2]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 2
  //SERVO 3
  MotorControl[MOTOR1] = (((int32_t)ServoRate[SERVO3] * MotorControl[MOTOR1]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 3
  //SERVO 4
  MotorControl[MOTOR6] = (((int32_t)ServoRate[SERVO4] * MotorControl[MOTOR6]) / 100) + PULSE_MIDDLE; //AJUSTA O RATE DO SERVO 4

  int16_t Servo_LPF_CutOff = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);

  if (Servo_LPF_CutOff == 0)
  {
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR3] = Constrain_16Bits(MotorControl[MOTOR3], PULSE_MIN, PULSE_MAX); //SERVO 1
    MotorControl[MOTOR2] = Constrain_16Bits(MotorControl[MOTOR2], PULSE_MIN, PULSE_MAX); //SERVO 2
    MotorControl[MOTOR1] = Constrain_16Bits(MotorControl[MOTOR1], PULSE_MIN, PULSE_MAX); //SERVO 3
    MotorControl[MOTOR6] = Constrain_16Bits(MotorControl[MOTOR6], PULSE_MIN, PULSE_MAX); //SERVO 4
  }
  else
  {
    //APLICA O LOW PASS FILTER NO SINAL DOS SERVOS
    DeviceFiltered[SERVO1] = (int16_t)LowPassFilter(&LPFDevice[SERVO1], MotorControl[MOTOR3], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO2] = (int16_t)LowPassFilter(&LPFDevice[SERVO2], MotorControl[MOTOR2], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO3] = (int16_t)LowPassFilter(&LPFDevice[SERVO3], MotorControl[MOTOR1], Servo_LPF_CutOff, LPF_SETPOINT);
    DeviceFiltered[SERVO4] = (int16_t)LowPassFilter(&LPFDevice[SERVO4], MotorControl[MOTOR6], Servo_LPF_CutOff, LPF_SETPOINT);
    //PULSO MINIMO E MAXIMO PARA OS SERVOS
    MotorControl[MOTOR3] = Constrain_16Bits(DeviceFiltered[SERVO1], PULSE_MIN, PULSE_MAX); //SERVO 1
    MotorControl[MOTOR2] = Constrain_16Bits(DeviceFiltered[SERVO2], PULSE_MIN, PULSE_MAX); //SERVO 2
    MotorControl[MOTOR1] = Constrain_16Bits(DeviceFiltered[SERVO3], PULSE_MIN, PULSE_MAX); //SERVO 3
    MotorControl[MOTOR6] = Constrain_16Bits(DeviceFiltered[SERVO4], PULSE_MIN, PULSE_MAX); //SERVO 4
  }

#if defined(PRINTLN_SERVO_SIGNAL)
  FastSerialPrintln(PSTR("Servo1:%d COMMAND_ARM_DISARM:%d AirPlaneMotor:%d RCController[YAW]:%d\n"),
                    MotorControl[MOTOR3],
                    COMMAND_ARM_DISARM,
                    AirPlaneMotor,
                    RCController[YAW]);
#endif
}
