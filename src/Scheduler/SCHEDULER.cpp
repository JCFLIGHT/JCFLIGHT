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

#include "SCHEDULER.h"
#include "SCHEDULERTIME.h"

uint16_t Loop_Integral_Time = 0;

bool SchedulerTimer(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime)
{
  uint32_t StoredTime = SCHEDULER.GetMicros();
  uint32_t ActualTime = StoredTime - SchedulerPointer->StoredTime;
  if (ActualTime >= RefreshTime)
  {
    SchedulerPointer->ActualTime = ActualTime;
    SchedulerPointer->StoredTime = StoredTime;
    return true;
  }
  return false;
}

void Update_Loop_Time()
{
  static uint32_t Loop_Guard_Time = 0;
  uint32_t Actual_Loop_TIme = SCHEDULER.GetMicros();
  Loop_Integral_Time = Actual_Loop_TIme - Loop_Guard_Time;
  Loop_Guard_Time = Actual_Loop_TIme;
}
