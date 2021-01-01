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

#ifndef TASKSYSTEM_H_
#define TASKSYSTEM_H_
#include "Arduino.h"
enum TaskPriority_Enum
{
  TASK_PRIORITY_LOW = 1,
  TASK_PRIORITY_MEDIUM = 3,
  TASK_PRIORITY_MEDIUM_HIGH = 4,
  TASK_PRIORITY_HIGH = 5,
  TASK_PRIORITY_REALTIME = 6
};

typedef enum
{
  TASK_SLOW_LOOP = 0,
  TASK_MEDIUM_LOOP,
  TASK_FAST_MEDIUM_LOOP,
  TASK_FAST_LOOP,
  TASK_INTEGRAL_LOOP,
  //NÃO MOVA DE LUGAR O RESTANTE DOS ENUM ABAIXO
  TASK_COUNT,
  TASK_NONE = TASK_COUNT,
  TASK_SELF
} Tasks_ID_Enum;

typedef struct
{
  const char *TaskName;
  void (*TaskFunction)();
  int32_t DesiredPeriod;
  const uint8_t StaticPriority;
  uint16_t DynamicPriority;
  uint16_t TaskAgeCycles;
  uint32_t LastExecuted;
  int32_t TaskLatestDeltaTime;
} Task_Recurses_Struct;

void TaskSystemInitialization(void);
void TaskSystemRun(void);

#endif
