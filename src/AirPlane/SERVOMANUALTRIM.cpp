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

#include "SERVOMANUALTRIM.h"
#include "AIRPLANE.h"
#include "RadioControl/RCCONFIG.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "BAR/BAR.h"
#include "FastSerial/PRINTF.h"

//#define PRINTLN_SERVORATE

bool OkToTrimServo = false;
bool OkToSaveTrim = false;
int16_t Trim_Servo[4];

void Manual_Trim_Servo_Initializate()
{
  //CARREGA A TRIMAGEM SALVA NA EEPROM
  Trim_Servo[SERVO1] = STORAGEMANAGER.Read_8Bits(SERVO1_TRIM_ADDR);
  Trim_Servo[SERVO2] = STORAGEMANAGER.Read_8Bits(SERVO2_TRIM_ADDR);
  Trim_Servo[SERVO3] = STORAGEMANAGER.Read_8Bits(SERVO3_TRIM_ADDR);
  Trim_Servo[SERVO4] = STORAGEMANAGER.Read_8Bits(SERVO4_TRIM_ADDR);
  //FAZ A CONVERSÃO DA TRIMAGEM SALVA NA EEPROM
  ServoRate[SERVO1] = Map_16Bits(Trim_Servo[0], 0, 255, -127, 127);
  ServoRate[SERVO2] = Map_16Bits(Trim_Servo[1], 0, 255, -127, 127);
  ServoRate[SERVO3] = Map_16Bits(Trim_Servo[2], 0, 255, -127, 127);
  ServoRate[SERVO4] = Map_16Bits(Trim_Servo[3], 0, 255, -127, 127);
}

void Manual_Trim_Servo_Update()
{
  if (GetFrameStateOfMultirotor())
    return;
#if defined(PRINTLN_SERVORATE)
  FastSerialPrintln(PSTR("Trim1:%d Trim2:%d Trim3:%d Trim4:%d Rate1:%d Rate2:%d Rate3:%d Rate4:%d OkToTrimServo:%d\n"),
                    Trim_Servo[SERVO1],
                    Trim_Servo[SERVO2],
                    Trim_Servo[SERVO3],
                    Trim_Servo[SERVO4],
                    ServoRate[SERVO1],
                    ServoRate[SERVO2],
                    ServoRate[SERVO3],
                    ServoRate[SERVO4],
                    OkToTrimServo);
#endif
  if (OkToTrimServo)
  {
    OkToSaveTrim = true;
    RCCONFIG.CancelDeadZone = true;
    //TRIMANDO OS SERVOS (PROCESSO)
    ServoRate[SERVO1] = Map_32Bits(Throttle.Output, 1000, 2000, -127, 127);
    ServoRate[SERVO2] = Map_32Bits(Yaw.Output, 1000, 2000, -127, 127);
    ServoRate[SERVO3] = Map_32Bits(Roll.Output, 1000, 2000, -127, 127);
    ServoRate[SERVO4] = Map_32Bits(Pitch.Output, 1000, 2000, -127, 127);
    Trim_Servo[SERVO1] = Map_16Bits(ServoRate[SERVO1], -127, 127, 0, 255);
    Trim_Servo[SERVO2] = Map_16Bits(ServoRate[SERVO2], -127, 127, 0, 255);
    Trim_Servo[SERVO3] = Map_16Bits(ServoRate[SERVO3], -127, 127, 0, 255);
    Trim_Servo[SERVO4] = Map_16Bits(ServoRate[SERVO4], -127, 127, 0, 255);
  }
  else
  {
    if (OkToSaveTrim)
    {
      //GUARDA A TRIMAGEM NA EEPROM
      STORAGEMANAGER.Write_8Bits(SERVO1_TRIM_ADDR, Trim_Servo[SERVO1]);
      STORAGEMANAGER.Write_8Bits(SERVO2_TRIM_ADDR, Trim_Servo[SERVO2]);
      STORAGEMANAGER.Write_8Bits(SERVO3_TRIM_ADDR, Trim_Servo[SERVO3]);
      STORAGEMANAGER.Write_8Bits(SERVO4_TRIM_ADDR, Trim_Servo[SERVO4]);
      RCCONFIG.CancelDeadZone = false;
      OkToSaveTrim = false;
    }
    else
    {
      ServoRate[SERVO1] = Map_16Bits(Trim_Servo[SERVO1], 0, 255, -127, 127);
      ServoRate[SERVO2] = Map_16Bits(Trim_Servo[SERVO2], 0, 255, -127, 127);
      ServoRate[SERVO3] = Map_16Bits(Trim_Servo[SERVO3], 0, 255, -127, 127);
      ServoRate[SERVO4] = Map_16Bits(Trim_Servo[SERVO4], 0, 255, -127, 127);
    }
  }
}