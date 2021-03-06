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
#include "Common/ENUM.h"
#include "PID/RCPID.h"

#ifdef __AVR_ATmega2560__

void UART2Mode_Initialization(void)
{
    DDRA |= (1 << DDD0); //DEFINE A PORTA DIGITAL 22 COMO SAIDA
    if (RC_Resources.ReceiverTypeEnabled != SBUS_RECEIVER)
    {
        PORTA |= 1 << 0; //ATIVA OS TRASISTORES DE CORTE E O DE BY-PASS
    }
    else
    {
        PORTA &= ~(1 << 0); //PREPARA OS TRANSISTORES PARA O MODO SBUS
    }
}

#elif defined __arm__ || defined ESP32

void UART2Mode_Initialization(void)
{
}

#endif