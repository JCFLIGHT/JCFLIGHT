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

#include "SPEED.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

SPEEDCLASS SPEEDMOTORS;

void SPEEDCLASS::LoadEEPROM()
{
    if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == VERY_LOW_SPEED)
    {
        MotorSpeed = 1000;
    }
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == LOW_SPEED)
    {
        MotorSpeed = 1050;
    }
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == MEDIUM_SPEED)
    {
        MotorSpeed = 1100;
    }
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == HIGH_SPEED)
    {
        MotorSpeed = 1150;
    }
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == VERY_HIGH_SPEED)
    {
        MotorSpeed = 1200;
    }
}