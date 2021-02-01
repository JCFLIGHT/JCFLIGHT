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

#include "STM32FREERAM.h"

#ifdef __arm__

bool MemRamChecked = false;
float Free;

uint16_t _MemoryRAM_Check()
{
    if (MemRamChecked)
    {
        return 0; //EVITA REALIZAR UM NOVO CALCULO DE RAM A CADA CICLO DE MAQUINA
    }
    MemRamChecked = true;
    //Free = 131072 - (STACKPTR - HEAPPTR);
    return 0; //RETORNA O VALOR DA MEMORIA RAM LIVRE NO STM32
}

uint8_t _GetPercentageRAMUsed()
{
    return Free / 131072 * 100;
}

#endif