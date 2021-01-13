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

#include "AIRSPEEDBACKEND.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

void Set_AirSpeed_Type(uint8_t AirSpeedType)
{
  if (Get_AirSpeed_Type() != AirSpeedType)
  {
    STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, AirSpeedType);
  }
}

uint8_t Get_AirSpeed_Type(void)
{
  return STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR);
}

bool Get_AirSpeed_State(void)
{
  return STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR) > 0;
}