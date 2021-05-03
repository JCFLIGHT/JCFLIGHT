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
#include "GPSNavigation/NAVIGATION.h"

bool Get_State_Armed_With_GPS(void)
{
    if (GPS_Resources.Navigation.Misc.Get.Satellites >= 5 && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Good_Condition(void)
{
    if (GPS_Resources.Navigation.Misc.Get.Satellites >= 5)
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Bad_Condition(void)
{
    if (GPS_Resources.Navigation.Misc.Get.Satellites < 5)
    {
        return true;
    }
    return false;
}

bool Get_GPS_In_Eight_Or_Plus_Satellites(void)
{
    if (GPS_Resources.Navigation.Misc.Get.Satellites >= 8)
    {
        return true;
    }
    return false;
}

bool Get_GPS_Type(uint8_t GPS_Type)
{
    static uint8_t _GPS_Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_1_ADDR);
    if (_GPS_Type == GPS_Type)
    {
        return true;
    }
    return false;
}

bool Get_GPS_Heading_Is_Valid(void)
{
    return GPS_Resources.Navigation.Misc.Get.Satellites >= 6 && GPS_Resources.Navigation.Misc.Get.GroundSpeed >= 300;
}

bool Get_GPS_Used_To_Navigation(void)
{
    return (GPS_Resources.Mode.Navigation != DO_NONE);
}

bool Get_GPS_Used_To_Land(void)
{
    return GPS_Resources.Mode.Navigation == DO_LAND_DESCENT ||
           GPS_Resources.Mode.Navigation == DO_LAND_DETECTED ||
           GPS_Resources.Mode.Navigation == DO_LANDED;
}