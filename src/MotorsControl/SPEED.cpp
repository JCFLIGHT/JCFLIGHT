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

int ValueConverterMotorSpeed(int Input, int InMin, int InMax, int OutMin, int OutMax)
{
    int ValueA = ((int)OutMax - (int)OutMin) * ((int)Input - (int)InMin);
    int ValueB = (int)InMax - (int)InMin;
    return ((ValueA / ValueB) + OutMin);
}

void SPEEDCLASS::LoadEEPROM()
{
    MotorSpeed = ValueConverterMotorSpeed(STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR), 0, 100, 900, 1500);
}