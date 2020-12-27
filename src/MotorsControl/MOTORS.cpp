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

#include "MOTORS.h"
#include "Common/VARIABLES.h"
#include "EscCalibration/CALIBESC.h"
#include "Common/STRUCTS.h"
#include "AirPlane/AIRPLANE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"
#include "PIDMIXING.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "FrameStatus/FRAMESTATUS.h"

//#define PWM_PINS_IN_ORDER //JCFLIGHT PCB

void ConfigureRegisters()
{
#ifdef __AVR_ATmega2560__

  //CONFIGURAÇÃO DAS PORTAS DE SAÍDA
  DDRE |= (1 << DDD4); //DEFINE A PORTA DIGITAL 2 COMO SAIDA
  DDRE |= (1 << DDD5); //DEFINE A PORTA DIGITAL 3 COMO SAIDA
  DDRE |= (1 << DDD3); //DEFINE A PORTA DIGITAL 5 COMO SAIDA
  DDRH |= (1 << DDD3); //DEFINE A PORTA DIGITAL 6 COMO SAIDA
  DDRH |= (1 << DDD4); //DEFINE A PORTA DIGITAL 7 COMO SAIDA
  DDRH |= (1 << DDD5); //DEFINE A PORTA DIGITAL 8 COMO SAIDA
  DDRL |= (1 << DDD3); //DEFINE A PORTA DIGITAL 46 COMO SAIDA
  DDRL |= (1 << DDD4); //DEFINE A PORTA DIGITAL 45 COMO SAIDA
  DDRA |= (1 << DDD0); //DEFINE A PORTA DIGITAL 22 COMO SAIDA

  //CONFIGURA O TIMER1 CANAIS A E B E TAMBÉM O TIMER2 CANAL A PARA O CONTROLE DO LED RGB
  TCCR1A |= _BV(COM1A1); //CONECTA O PINO 12 AO TIMER1 CANAL A
  TCCR1A |= _BV(COM1B1); //CONECTA O PINO 11 AO TIMER1 CANAL B
  TCCR2A |= _BV(COM2A1); //CONECTA O PINO 10 AO TIMER2 CANAL A

  //CONFIGURA O TIMER2 CANAL B PARA O CONTROLE DO BUZZER
  TCCR2A |= _BV(COM2B1); //CONECTA O PINO 9 AO TIMER2 CANAL B

  //CONFIGURA O TIMER 3 PARA OPERAR INICIALMENTE EM 490Hz
  TCCR3A |= (1 << WGM31);
  TCCR3A &= ~(1 << WGM30);
  TCCR3B |= (1 << WGM33);
  TCCR3B &= ~(1 << CS31);
  if (GetFrameStateOfAirPlane())
  {
    ICR3 |= 40000; //50Hz
  }
  else
  {
    ICR3 |= 16383; //490Hz
  }

  //CONFIGURA O TIMER 3
  TCCR3A |= _BV(COM3A1); //CONECTA O PINO 5 AO TIMER1 CANAL A
  TCCR3A |= _BV(COM3B1); //CONECTA O PINO 2 AO TIMER3 CANAL B
  TCCR3A |= _BV(COM3C1); //CONECTA O PINO 3 AO TIMER1 CANAL C

  //CONFIGURA O TIMER 4 PARA OPERAR INICIALMENTE EM 490Hz
  TCCR4A |= (1 << WGM41);
  TCCR4A &= ~(1 << WGM40);
  TCCR4B |= (1 << WGM43);
  TCCR4B &= ~(1 << CS41);
  if (GetFrameStateOfAirPlane())
  {
    ICR4 |= 40000; //50Hz
  }
  else
  {
    ICR4 |= 16383; //490Hz
  }

  //CONFIGURA O TIMER 4
  TCCR4A |= _BV(COM4A1); //CONECTA O PINO 6 AO TIMER4 CANAL A
  TCCR4A |= _BV(COM4B1); //CONECTA O PINO 7 AO TIMER4 CANAL B
  TCCR4A |= _BV(COM4C1); //CONECTA O PINO 8 AO TIMER4 CANAL C

  //CONFIGURA O TIMER 5 PARA OPERAR EM 50Hz
  TCCR5A |= (1 << WGM51);
  TCCR5A &= ~(1 << WGM50);
  TCCR5B |= (1 << WGM53);
  TCCR5B &= ~(1 << CS41);
  ICR5 |= 40000; //50Hz

  //CONFIGURA O TIMER 5
  TCCR5A |= _BV(COM4A1); //CONECTA O PINO 46 AO TIMER4 CANAL A
  TCCR5A |= _BV(COM4B1); //CONECTA O PINO 45 AO TIMER4 CANAL B
  //TCCR5A |= _BV(COM4C1); //CONECTA O PINO 44 AO TIMER4 CANAL C

#elif defined __arm__

#endif

  if (!ESC.Run_Calibrate && !SAFETYBUTTON.SafeButtonEnabled())
  {
    PulseInAllMotors(1000);
    AVRTIME.SchedulerSleep(300);
  }
}

void PID_MixMotors()
{
  if (!SAFETYBUTTON.GetSafeStateToOutput())
    return;
  MixingApplyPIDControl();
  if (GetFrameStateOfMultirotor())
  {
    int16_t SuportMotor = MotorControl[MOTOR4];
    for (uint8_t i = 1; i < NumberOfMotors; i++)
      if (MotorControl[i] > SuportMotor)
        SuportMotor = MotorControl[i];
    for (uint8_t i = 0; i < NumberOfMotors; i++)
    {
      if (SuportMotor > 1900)
        MotorControl[i] -= SuportMotor - 1900;
      MotorControl[i] = Constrain_16Bits(MotorControl[i], MotorSpeed, 1900);
      if (RadioControllOutput[THROTTLE] < 1100)
        MotorControl[i] = MotorSpeed;
      if (!COMMAND_ARM_DISARM)
        MotorControl[i] = 1000;
    }
  }
}

void PulseInAllMotors(int16_t Pulse)
{
  MotorControl[MOTOR4] = Pulse;
  MotorControl[MOTOR3] = Pulse;
  MotorControl[MOTOR2] = Pulse;
  MotorControl[MOTOR1] = Pulse;
  MotorControl[MOTOR6] = Pulse;
  MotorControl[MOTOR5] = Pulse;
  ApplyPWMInAllComponents();
}

void ApplyPWMInAllComponents()
{

#ifdef __AVR_ATmega2560__

#ifndef PWM_PINS_IN_ORDER

  OCR3C = MotorControl[MOTOR4] << 3; //PINO DIGITAL 3 (MOTOR 4 NO FRAME)
  OCR3A = MotorControl[MOTOR3] << 3; //PINO DIGITAL 5 (MOTOR 3 NO FRAME)
  OCR4A = MotorControl[MOTOR2] << 3; //PINO DIGITAL 6 (MOTOR 2 NO FRAME)
  OCR3B = MotorControl[MOTOR1] << 3; //PINO DIGITAL 2 (MOTOR 1 NO FRAME)
  OCR4B = MotorControl[MOTOR6] << 3; //PINO DIGITAL 7 (MOTOR 6 NO FRAME)
  OCR4C = MotorControl[MOTOR5] << 3; //PINO DIGITAL 8 (MOTOR 5 NO FRAME)

#else

  OCR3B = MotorControl[MOTOR1] << 3; //PINO DIGITAL 2 (MOTOR 1 NO FRAME)
  OCR3C = MotorControl[MOTOR2] << 3; //PINO DIGITAL 3 (MOTOR 2 NO FRAME)
  OCR3A = MotorControl[MOTOR3] << 3; //PINO DIGITAL 5 (MOTOR 3 NO FRAME)
  OCR4A = MotorControl[MOTOR4] << 3; //PINO DIGITAL 6 (MOTOR 4 NO FRAME)
  OCR4B = MotorControl[MOTOR5] << 3; //PINO DIGITAL 7 (MOTOR 5 NO FRAME)
  OCR4C = MotorControl[MOTOR6] << 3; //PINO DIGITAL 8 (MOTOR 6 NO FRAME)

#endif

  OCR5A = MotorControl[GIMBAL] << 3;         //PINO DIGITAL 46
  OCR5B = MotorControl[PARACHUTESERVO] << 3; //PINO DIGITAL 45

#elif defined __arm__

#endif
}
