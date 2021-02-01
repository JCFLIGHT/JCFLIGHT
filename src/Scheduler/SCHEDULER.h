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

#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "Arduino.h"
#include "Common/STRUCTS.h"
#define SCHEDULER_SET_FREQUENCY(Frequecy, Unidad) (1000000 / (Frequecy))
extern uint16_t Loop_Integral_Time;
bool Scheduler(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime);
void Update_Loop_Time();
#endif
