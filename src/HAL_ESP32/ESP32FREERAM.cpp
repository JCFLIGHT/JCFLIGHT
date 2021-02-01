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

#include "ESP32FREERAM.h"
#include "Math/MATHSUPPORT.h"

#ifdef ESP32

bool MemRamChecked = false;
uint32_t Free;
uint32_t SizeOfRamMemory;

uint16_t _MemoryRAM_Check()
{
  if (MemRamChecked)
  {
    return SizeOfRamMemory; //EVITA REALIZAR UM NOVO CALCULO DE RAM A CADA CICLO DE MAQUINA
  }
  MemRamChecked = true;
  Free = 327680 - ESP.getFreeHeap();
  SizeOfRamMemory = ESP.getFreeHeap();
  return ESP.getFreeHeap(); //RETORNA O VALOR DA MEMORIA RAM LIVRE NO ESP32
}

uint8_t _GetPercentageRAMUsed()
{
  return Free / 327680 * 100;
}

#endif