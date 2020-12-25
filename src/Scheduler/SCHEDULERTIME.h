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

#ifndef SCHEDULERTIME_h
#define SCHEDULERTIME_h
#include "Arduino.h"
#ifdef __AVR_ATmega2560__
#include <avr/interrupt.h>
#endif
class AVRTIMECLASS
{
public:
  void SchedulerInit(void);
  void SchedulerSleep(uint16_t MillisSeconds);
  void SchedulerMicroSecondsSleep(uint16_t MicroSeconds);
  uint32_t SchedulerMillis(void);
  uint32_t SchedulerMicros(void);
};
extern AVRTIMECLASS AVRTIME;
#endif
