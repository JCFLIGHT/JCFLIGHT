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

#include "SERVOSPWR.h"
#include "FrameStatus/FRAMESTATUS.h"

void ServosPWR(void)
{
    DDRA |= (1 << DDD1); //DEFINE A PORTA DIGITAL 23 COMO SAIDA
    DDRA |= (1 << DDD2); //DEFINE A PORTA DIGITAL 24 COMO SAIDA
    if (GetFrameStateOfAirPlane())
    {
        PORTA |= 1 << 1;    //ATIVA A ALIMENTAÇÃO DOS SERVOS
        PORTA &= ~(1 << 2); //ATIVA A ALIMENTAÇÃO DOS SERVOS
    }
    else
    {
        PORTA &= ~(1 << 1); //DESATIVA A ALIMENTAÇÃO DOS SERVOS
        PORTA |= 1 << 2;    //DESATIVA A ALIMENTAÇÃO DOS SERVOS
    }
}