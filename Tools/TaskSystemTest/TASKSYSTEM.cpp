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

static Task_Resources_Struct *TaskQueueArray[TASK_COUNT + 1];
static Task_Resources_Struct *CurrentTask = NULL;

static int16_t TaskQueuePosition = 0;
static int16_t TaskQueueSize = 0;

uint16_t SystemLoadPercent = 0;

static uint32_t TotalWaitingTasks;
static uint32_t WaitingTasksSamples;

void TaskQueueClear(void)
{
  memset(TaskQueueArray, 0, sizeof(TaskQueueArray));
  TaskQueuePosition = 0;
  TaskQueueSize = 0;
}

static bool TaskQueueContains(Task_Resources_Struct *TaskPointer)
{
  for (int TaskQueueSizeCount = 0; TaskQueueSizeCount < TaskQueueSize; ++TaskQueueSizeCount)
  {
    if (TaskQueueArray[TaskQueueSizeCount] == TaskPointer)
    {
      return true;
    }
  }
  return false;
}

bool TaskQueueAdd(Task_Resources_Struct *TaskPointer)
{
  if ((TaskQueueSize >= TASK_COUNT) || TaskQueueContains(TaskPointer))
  {
    return false;
  }
  for (int TaskQueueSizeCount = 0; TaskQueueSizeCount <= TaskQueueSize; ++TaskQueueSizeCount)
  {
    if (TaskQueueArray[TaskQueueSizeCount] == NULL || TaskQueueArray[TaskQueueSizeCount]->StaticPriority < TaskPointer->StaticPriority)
    {
      memmove(&TaskQueueArray[TaskQueueSizeCount + 1], &TaskQueueArray[TaskQueueSizeCount], sizeof(TaskPointer) * (TaskQueueSize - TaskQueueSizeCount));
      TaskQueueArray[TaskQueueSizeCount] = TaskPointer;
      ++TaskQueueSize;
      return true;
    }
  }
  return false;
}

static Task_Resources_Struct *TaskQueueNext(void)
{
  return TaskQueueArray[++TaskQueuePosition];
}

static bool TaskQueueRemove(Task_Resources_Struct *TaskPointer)
{
  for (int TaskQueueSizeCount = 0; TaskQueueSizeCount < TaskQueueSize; ++TaskQueueSizeCount)
  {
    if (TaskQueueArray[TaskQueueSizeCount] == TaskPointer)
    {
      memmove(&TaskQueueArray[TaskQueueSizeCount], &TaskQueueArray[TaskQueueSizeCount + 1], sizeof(TaskPointer) * (TaskQueueSize - TaskQueueSizeCount));
      --TaskQueueSize;
      return true;
    }
  }
  return false;
}

static inline Task_Resources_Struct *QueueFirst(void)
{
  TaskQueuePosition = 0;
  return TaskQueueArray[0];
}

void SetTaskEnabled(Tasks_ID_Enum TaskID, bool Enabled)
{
  Task_Resources_Struct *TaskPointer = &Task_Resources[TaskID];
  if (Enabled && TaskPointer->TaskFunction)
  {
    TaskQueueAdd(TaskPointer);
  }
  else
  {
    TaskQueueRemove(TaskPointer);
  }
}

void TaskSystemRun(void)
{
  const uint32_t ActualCurrentTime = micros();
  uint32_t TimeToNextRealTimeTask = 0xffffffffUL;
  for (const Task_Resources_Struct *TaskPointer = QueueFirst(); TaskPointer != NULL && TaskPointer->StaticPriority >= TASK_PRIORITY_REALTIME; TaskPointer = TaskQueueNext())
  {
    const uint32_t NextExecuteTask = TaskPointer->LastExecuted + TaskPointer->DesiredPeriod;
    if ((int32_t)(ActualCurrentTime - NextExecuteTask) >= 0)
    {
      TimeToNextRealTimeTask = 0;
    }
    else
    {
      const uint32_t NewTimeInterval = NextExecuteTask - ActualCurrentTime;
      TimeToNextRealTimeTask = min(TimeToNextRealTimeTask, NewTimeInterval);
    }
  }
  const bool OutsideRealtimeGuardInterval = (TimeToNextRealTimeTask > 0);
  Task_Resources_Struct *SelectedTask = NULL;
  uint16_t SelectedTaskDynamicPriority = 0;
  uint16_t WaitingTasks = 0;
  for (Task_Resources_Struct *TaskPointer = QueueFirst(); TaskPointer != NULL; TaskPointer = TaskQueueNext())
  {
    TaskPointer->TaskAgeCycles = ((int32_t)(ActualCurrentTime - TaskPointer->LastExecuted)) / TaskPointer->DesiredPeriod;
    if (TaskPointer->TaskAgeCycles > 0)
    {
      TaskPointer->DynamicPriority = 1 + TaskPointer->StaticPriority * TaskPointer->TaskAgeCycles;
      WaitingTasks++;
    }
    if (TaskPointer->DynamicPriority > SelectedTaskDynamicPriority)
    {
      const bool TaskCanBeChosenForScheduling = (OutsideRealtimeGuardInterval) ||
                                                (TaskPointer->TaskAgeCycles > 1) ||
                                                (TaskPointer->StaticPriority == TASK_PRIORITY_REALTIME);
      if (TaskCanBeChosenForScheduling)
      {
        SelectedTaskDynamicPriority = TaskPointer->DynamicPriority;
        SelectedTask = TaskPointer;
      }
    }
  }
  WaitingTasksSamples++;
  TotalWaitingTasks += WaitingTasks;
  CurrentTask = SelectedTask;
  if (SelectedTask)
  {
    SelectedTask->TaskLatestDeltaTime = (int32_t)(ActualCurrentTime - SelectedTask->LastExecuted);
    SelectedTask->LastExecuted = ActualCurrentTime;
    SelectedTask->DynamicPriority = 0;
    SelectedTask->TaskFunction();
  }
}

void SystemLoad()
{
  if (WaitingTasksSamples > 0)
  {
    SystemLoadPercent = 100 * TotalWaitingTasks / WaitingTasksSamples;
    WaitingTasksSamples = 0;
    TotalWaitingTasks = 0;
  }
  Serial.print("Carga Da CPU:");
  Serial.println(SystemLoadPercent);
}

uint32_t GetTaskDeltaTime(Tasks_ID_Enum TaskId)
{
  if (TaskId < TASK_COUNT)
  {
    return Task_Resources[TaskId].TaskLatestDeltaTime;
  }
  return 0;
}
