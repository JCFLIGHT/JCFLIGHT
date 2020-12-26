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

#ifndef TASKSYSTEM_H
#define TASKSYSTEM_H
#include "Arduino.h"
class TaskSystem_Class
{
public:
  typedef void (*TaskSystem_FN)(void);
  struct Task
  {
    TaskSystem_FN Function;
    uint16_t Interval_Ticks;
    uint16_t Maximum_Time;
  };
  void Initialization(const Task *Tasks, uint8_t Number_Of_Tasks);
  void UpdateTick(void);
  void RunProcess(uint16_t Time_Available);

private:
  const struct Task *_Tasks;
  uint8_t _Number_Of_Tasks;
  uint16_t _Tick_Counter;
  uint16_t *_Last_Run;
  uint32_t _Task_Time_Allowed;
  uint32_t _Task_Time_Started;
  uint32_t _Spare_Micros;
  uint8_t _Spare_Ticks;
};
extern TaskSystem_Class TaskSystem;
#endif
