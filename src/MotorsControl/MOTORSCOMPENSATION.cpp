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

#include "MOTORSCOMPENSATION.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "BatteryMonitor/BATTERY.h"
#include "BatteryMonitor/BATTLEVELS.h"

#ifdef __AVR_ATmega2560__

const int8_t TableSelectDrop3SLipo[] __attribute__((__progmem__)) = {0, 3, 5, 8, 11, 14, 17, 19, 22, 25, 28, 31, 34, 38, 41, 44, 47, 51,
                                                                     54, 58, 61, 65, 68, 72, 76, 79, 83, 87, 91, 95, 99, 104, 108, 112,
                                                                     117, 121, 126};

const int8_t TableSelectDrop4SLipo[] __attribute__((__progmem__)) = {0, 2, 4, 6, 8, 10, 12, 14, 17, 19, 21, 23, 25, 28, 30, 32, 34, 37,
                                                                     39, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 78, 81,
                                                                     84, 87, 90, 93, 96, 99, 103, 106, 109, 112, 116, 119, 122, 126};

const int8_t TableSelectDrop6SLipo[] __attribute__((__progmem__)) = {0, 1, 3, 4, 5, 7, 8, 9, 11, 12, 14, 15, 17, 18, 19, 21, 22, 24, 25, 27,
                                                                     28, 30, 31, 33, 34, 36, 38, 39, 41, 42, 44, 46, 47, 49, 51, 52, 54, 56,
                                                                     58, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 79, 81, 83, 85, 87, 89,
                                                                     91, 93, 95, 97, 99, 101, 104, 106, 108, 110, 112, 114, 117, 119, 121, 123, 126};

#elif defined __arm__ || defined ESP32

const int8_t TableSelectDrop3SLipo[] = {0, 3, 5, 8, 11, 14, 17, 19, 22, 25, 28, 31, 34, 38, 41, 44, 47, 51,
                                        54, 58, 61, 65, 68, 72, 76, 79, 83, 87, 91, 95, 99, 104, 108, 112,
                                        117, 121, 126};

const int8_t TableSelectDrop4SLipo[] = {0, 2, 4, 6, 8, 10, 12, 14, 17, 19, 21, 23, 25, 28, 30, 32, 34, 37,
                                        39, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76, 78, 81,
                                        84, 87, 90, 93, 96, 99, 103, 106, 109, 112, 116, 119, 122, 126};

const int8_t TableSelectDrop6SLipo[] = {0, 1, 3, 4, 5, 7, 8, 9, 11, 12, 14, 15, 17, 18, 19, 21, 22, 24, 25, 27,
                                        28, 30, 31, 33, 34, 36, 38, 39, 41, 42, 44, 46, 47, 49, 51, 52, 54, 56,
                                        58, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 79, 81, 83, 85, 87, 89,
                                        91, 93, 95, 97, 99, 101, 104, 106, 108, 110, 112, 114, 117, 119, 121, 123, 126};

#endif

void Motors_Compensation(uint8_t State, uint8_t _NumbOfMotors)
{
  if (!State || _NumbOfMotors == 0)
  {
    return;
  }
  uint8_t VoltageDropCalculate = 0;
  if (BATTERY.Voltage > BATT_3S_SAFE_LOW_VOLTAGE && BATTERY.Voltage < BATT_3S_SAFE_HIGH_VOLTAGE)
  {
    //BATERIA LIPO 3S
    VoltageDropCalculate = Constrain_U8Bits(126 - Constrain_U8Bits(BATTERY.Voltage * 10, 93, 126), 0, 36);
    MotorControl[MOTOR1] += (((int32_t)(MotorControl[MOTOR1] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR2] += (((int32_t)(MotorControl[MOTOR2] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR3] += (((int32_t)(MotorControl[MOTOR3] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR4] += (((int32_t)(MotorControl[MOTOR4] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
    if (_NumbOfMotors > 4)
    {
      MotorControl[MOTOR5] += (((int32_t)(MotorControl[MOTOR5] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
      MotorControl[MOTOR6] += (((int32_t)(MotorControl[MOTOR6] - 1000) * (int32_t)TableSelectDrop3SLipo[VoltageDropCalculate])) / 500;
    }
  }
  else if (BATTERY.Voltage > BATT_4S_SAFE_LOW_VOLTAGE && BATTERY.Voltage < BATT_4S_SAFE_HIGH_VOLTAGE)
  {
    //BATERIA LIPO 4S
    VoltageDropCalculate = Constrain_U8Bits(168 - Constrain_U8Bits(BATTERY.Voltage * 10, 124, 168), 0, 48);
    MotorControl[MOTOR1] += (((int32_t)(MotorControl[MOTOR1] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR2] += (((int32_t)(MotorControl[MOTOR2] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR3] += (((int32_t)(MotorControl[MOTOR3] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR4] += (((int32_t)(MotorControl[MOTOR4] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
    if (_NumbOfMotors > 4)
    {
      MotorControl[MOTOR5] += (((int32_t)(MotorControl[MOTOR5] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
      MotorControl[MOTOR6] += (((int32_t)(MotorControl[MOTOR6] - 1000) * (int32_t)TableSelectDrop4SLipo[VoltageDropCalculate])) / 500;
    }
  }
  else if (BATTERY.Voltage > BATT_6S_SAFE_LOW_VOLTAGE && BATTERY.Voltage < BATT_6S_SAFE_HIGH_VOLTAGE)
  {
    //BATERIA LIPO 6S
    VoltageDropCalculate = Constrain_U16Bits(252 - Constrain_U16Bits(BATTERY.Voltage * 10, 186, 252), 0, 78);
    MotorControl[MOTOR1] += (((int32_t)(MotorControl[MOTOR1] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR2] += (((int32_t)(MotorControl[MOTOR2] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR3] += (((int32_t)(MotorControl[MOTOR3] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
    MotorControl[MOTOR4] += (((int32_t)(MotorControl[MOTOR4] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
    if (_NumbOfMotors > 4)
    {
      MotorControl[MOTOR5] += (((int32_t)(MotorControl[MOTOR5] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
      MotorControl[MOTOR6] += (((int32_t)(MotorControl[MOTOR6] - 1000) * (int32_t)TableSelectDrop6SLipo[VoltageDropCalculate])) / 500;
    }
  }
}