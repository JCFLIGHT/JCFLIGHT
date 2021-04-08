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

#include "GPSSTATES.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPS/GPSUBLOX.h"
#include "BitArray/BITARRAY.h"
#include "Common/ENUM.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

bool Get_State_Armed_With_GPS(void)
{
    if (GPS_NumberOfSatellites >= 5 && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && Home_Point)
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Good_Condition(void)
{
    if (GPS_NumberOfSatellites >= 5)
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Bad_Condition(void)
{
    if (GPS_NumberOfSatellites < 5)
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Eight_Or_Plus_Satellites(void)
{
    if (GPS_NumberOfSatellites >= 8)
    {
        return true;
    }
    return false;
}

bool Get_GPS_Type(uint8_t GPS_Type)
{
    static uint8_t Get_GPS_Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_1_ADDR);
    if ((Get_GPS_Type == GPS_UBLOX && GPS_Type == GPS_UBLOX) ||
        (Get_GPS_Type == GPS_DJI_NAZA && GPS_Type == GPS_DJI_NAZA))
    {
        return true;
    }
    return false;
}

bool Get_GPS_Heading_Is_Valid(void)
{
    return GPS_NumberOfSatellites >= 6 && GPS_Ground_Speed >= 300;
}

bool Get_GPS_Flight_Modes_And_Navigation_In_Use(void)
{
    return (GPS_Flight_Mode != GPS_MODE_NONE) && (GPS_Navigation_Mode != DO_NONE);
}

bool Get_GPS_Only_Flight_Modes_In_Use(void)
{
    return (GPS_Flight_Mode != GPS_MODE_NONE);
}