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

#include "UART2MODE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

#ifdef __AVR_ATmega2560__
void UART2Mode_Initialization()
{
    DDRA |= (1 << DDD0); //DEFINE A PORTA DIGITAL 22 COMO SAIDA
    if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) != 1)
    {
        PORTA |= 1 << 0; //ATIVA OS TRASISTORES DE CORTE E O DE BY-PASS
    }
    else
    {
        PORTA &= ~(1 << 0); //PREPARA OS TRANSISTORES PARA O MODO SBUS
    }
}

#elif __arm__
void UART2Mode_Initialization()
{
}
#endif