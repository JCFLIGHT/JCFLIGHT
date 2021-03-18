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

#include "MACHINEINIT.h"
#include "Scheduler/SCHEDULERTIME.h"

MachineInitTime_Struct MachineInitTime_StructNow;

void SetInitialTimeToInitTheMachine(MachineInitTime_Struct *MachineInitTime_StructPointer)
{
    MachineInitTime_StructPointer->PreviousTime = SCHEDULERTIME.GetMillis();
}

void CalculeTheFinalTimeToInitTheMachine(MachineInitTime_Struct *MachineInitTime_StructPointer)
{
    MachineInitTime_StructPointer->ActualTime = SCHEDULERTIME.GetMillis();
    MachineInitTime_StructPointer->TotalTime = MachineInitTime_StructPointer->ActualTime - MachineInitTime_StructPointer->PreviousTime;
}

uint32_t GetTheFinalTimeToInitTheMachine(MachineInitTime_Struct *MachineInitTime_StructPointer)
{
    return MachineInitTime_StructPointer->TotalTime / 1000;
}