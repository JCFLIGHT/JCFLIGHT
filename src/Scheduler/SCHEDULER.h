#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "Arduino.h"
#define SCHEDULER_PERIOD_HZ(Frequecy, Unidad) (1000000 / (Frequecy))
typedef struct
{
  uint32_t ActualTime;
  uint32_t StoredTime;
} Scheduler_Struct;
extern uint16_t Loop_Integral_Time;
bool SchedulerTimer(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime);
void Update_Loop_Time();
#endif
