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

#include "HALFREERAM.h"
#include "HAL_AVR/AVRFREERAM.h"
#include "HAL_ESP32/ESP32FREERAM.h"
#include "HAL_STM32/STM32FREERAM.h"

uint16_t _MemoryRAM_Check_()
{
    return _MemoryRAM_Check();
}

uint8_t _GetPercentageRAMUsed_()
{
    return _GetPercentageRAMUsed();
}