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

#include "TASKSYSTEM.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "ProgMem/PROGMEM.h"

void TaskSystem_Class::Initialization(const TaskSystem_Class::Task *Tasks, uint8_t Number_Of_Tasks)
{
  _Tasks = Tasks;
  _Number_Of_Tasks = Number_Of_Tasks;
  _Last_Run = new uint16_t[_Number_Of_Tasks];
  memset(_Last_Run, 0, sizeof(_Last_Run[0]) * _Number_Of_Tasks);
  _Tick_Counter = 0;
}

void TaskSystem_Class::UpdateTick(void)
{
  _Tick_Counter++;
}

void TaskSystem_Class::RunProcess(uint16_t Time_Available)
{
  uint32_t Run_Started_MicrosSec = AVRTIME.SchedulerMicros();
  uint32_t Time_Now = Run_Started_MicrosSec;
  for (uint8_t i = 0; i < _Number_Of_Tasks; i++)
  {
    uint16_t Delta_Time = _Tick_Counter - _Last_Run[i];
    uint16_t Interval_Ticks = ProgMemReadWord(&_Tasks[i].Interval_Ticks);
    if (Delta_Time >= Interval_Ticks)
    {
      _Task_Time_Allowed = ProgMemReadWord(&_Tasks[i].Maximum_Time);
      if (_Task_Time_Allowed <= Time_Available)
      {
        _Task_Time_Started = Time_Now;
        TaskSystem_FN func = (TaskSystem_FN)PGM_Read_Pointer(&_Tasks[i].Function);
        func();
        _Last_Run[i] = _Tick_Counter;
        Time_Now = AVRTIME.SchedulerMicros();
        uint32_t Time_Taken = Time_Now - _Task_Time_Started;
        if (Time_Taken >= Time_Available)
        {
          goto update_Spare_Ticks;
        }
        Time_Available -= Time_Taken;
      }
    }
  }
  _Spare_Micros += Time_Available;
update_Spare_Ticks:
  _Spare_Ticks++;
  if (_Spare_Ticks == 32)
  {
    _Spare_Ticks /= 2;
    _Spare_Micros /= 2;
  }
}

uint8_t TaskSystem_Class::Calced_CPU_Load_Average(uint32_t Tick_Time_Usec) const
{
  if (_Spare_Ticks == 0)
  {
    return 0.0f;
  }
  uint32_t used_time = Tick_Time_Usec - (_Spare_Micros / _Spare_Ticks);
  return (used_time / (float)Tick_Time_Usec) * 100;
}