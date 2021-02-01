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

#include "AVRFREERAM.h"

#ifdef __AVR_ATmega2560__

bool MemRamChecked = false;
uint8_t *HEAPPTR;
uint8_t *STACKPTR;
float Free;

//STACK POINTER INICIA NO TOPO DA MEMORIA RAM E DESCE ATÉ O FINAL
//HEAP POINTER INICIA JUNTO COM VARIAVEIS STATICAS,STRUTURAS,ETC E VAI ATÉ O FINAL
//SP É A LARGURA DO HEAP POINTER
uint16_t _MemoryRAM_Check()
{
    if (MemRamChecked)
    {
        return STACKPTR - HEAPPTR; //EVITA REALIZAR UM NOVO CALCULO DE RAM A CADA CICLO DE MAQUINA
    }
    STACKPTR = (uint8_t *)malloc(4); //USA STACKPTR TEMPORIAMENTE
    HEAPPTR = STACKPTR;              //SALVA O VALOR DO STACKPTR NA VARIAVEL HEAPPTR
    free(STACKPTR);                  //SETA STACK EM ZERO
    STACKPTR = (uint8_t *)(SP);      //SALVA O VALOR PARA STACKPTR
    MemRamChecked = true;
    Free = 8192 - (STACKPTR - HEAPPTR);
    return STACKPTR - HEAPPTR; //RETORNA O VALOR DA MEMORIA RAM LIVRE NO MEGA2560
}

uint8_t _GetPercentageRAMUsed()
{
    return Free / 8192 * 100;
}

#endif