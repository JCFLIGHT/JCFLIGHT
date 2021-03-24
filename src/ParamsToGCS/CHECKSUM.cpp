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

#include "CHECKSUM.h"
#include "I2C/I2C.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "AirPlane/AIRPLANE.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "RadioControl/RCCONFIG.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

CheckSumClass CHECKSUM;

uint8_t CheckSumClass::GetDevicesActived()
{
    const bool Compass_Detect = I2C.CompassFound;
    const bool Parachute_Detect = ParachuteDetectTrigger > 0 ? true : false;
    const bool Matek_Lidar_OptFlowDetect = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR) == 1 ? true : false;
    const bool Pitot_Detect = Get_AirSpeed_Enabled();
    uint8_t CheckDevices = Compass_Detect |
                           Parachute_Detect << 1 |
                           Matek_Lidar_OptFlowDetect << 2 |
                           Pitot_Detect << 3;
    return CheckDevices;
}

void CheckSumClass::UpdateServosReverse()
{
    uint8_t ServosReverse = STORAGEMANAGER.Read_8Bits(SERVOS_REVERSE_ADDR);

    //ASA
    Servo.Direction.GetAndSet[SERVO1] = GET_SERVO_DIRECTION(ServosReverse & 1);

    //ASA
    Servo.Direction.GetAndSet[SERVO2] = GET_SERVO_DIRECTION(ServosReverse & 2);

    //LEME
    Servo.Direction.GetAndSet[SERVO3] = GET_SERVO_DIRECTION(ServosReverse & 4);

    //PROFUNDOR
    Servo.Direction.GetAndSet[SERVO4] = GET_SERVO_DIRECTION(ServosReverse & 8);
}

void CheckSumClass::UpdateChannelsReverse()
{
    uint8_t ChannelsReverse = STORAGEMANAGER.Read_8Bits(CH_REVERSE_ADDR);
    CHECKSUM.GetFailSafeValue = STORAGEMANAGER.Read_16Bits(FAILSAFE_VAL_ADDR);

    //THROTTLE
    if ((ChannelsReverse & 1) > 0)
    {
        Throttle.Set_Reverse(true);
    }
    else
    {
        Throttle.Set_Reverse(false);
    }

    //YAW
    if ((ChannelsReverse & 2) > 0)
    {
        Yaw.Set_Reverse(true);
    }
    else
    {
        Yaw.Set_Reverse(false);
    }

    //PITCH
    if ((ChannelsReverse & 4) > 0)
    {
        Pitch.Set_Reverse(true);
    }
    else
    {
        Pitch.Set_Reverse(false);
    }

    //ROLL
    if ((ChannelsReverse & 8) > 0)
    {
        Roll.Set_Reverse(true);
    }
    else
    {
        Roll.Set_Reverse(false);
    }
}