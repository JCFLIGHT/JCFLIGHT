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

#include "AIRSPEED.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/AVRMATH.h"
#include "AIRSPEEDANALOG.h"
#include "AIRSPEEDI2C.h"
#include "AIRSPEEDBACKEND.h"
#include "BAR/BAR.h"

#define AIRSPEED_FACTOR 11.96f

float AirSpeedPressureRead;
float AirSpeedAdjustOffSet;

uint16_t AirSpeedCalculedPressure = 0;

void AirSpeed_Initialization()
{
  if ((STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) < 3) ||
      (STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) == 6) ||
      (STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR) == 7))
    return;
  if (Get_AirSpeed_Type() == NONE_AIRSPEED)
  {
    return;
  }
  else if (Get_AirSpeed_Type() == ANALOG_AIRSPEED)
  {
    AirSpeedAdjustOffSet = AirSpeed_Analog_Get_Calibration();
  }
  else if (Get_AirSpeed_Type() == I2C_AIRSPEED)
  {
    AirSpeedAdjustOffSet = AirSpeed_I2C_Get_Calibration();
  }
}

void AirSpeed_Update()
{
  if (FrameType < 3 || FrameType == 6 || FrameType == 7)
    return;
  if (Get_AirSpeed_Type() == NONE_AIRSPEED)
  {
    return;
  }
  else if (Get_AirSpeed_Type() == ANALOG_AIRSPEED)
  {
    AirSpeedPressureRead = AirSpeed_Analog_Get_Actual_Value();
  }
  else if (Get_AirSpeed_Type() == I2C_AIRSPEED)
  {
    AirSpeedPressureRead = AirSpeed_I2C_Get_Actual_Value();
  }
  if ((AirSpeedPressureRead - AirSpeedAdjustOffSet) < 0)
  {
    AirSpeedCalculedPressure = 0;
  }
  else
  {
    AirSpeedCalculedPressure = SquareRootU16Bits((float)(AirSpeedPressureRead - AirSpeedAdjustOffSet) * AIRSPEED_FACTOR) * 3.6f;
  }
}