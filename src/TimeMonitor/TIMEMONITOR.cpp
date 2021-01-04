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

#include "TIMEMONITOR.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FunctionsLoop/LOOPS.h"
#include "FunctionsName/FUNCTIONSNAME.h"
#include "FastSerial/PRINTF.h"

AVRTimeMonitor AVRTIMEMONITOR;

bool SafeToPrint = false;
uint16_t FunctionNumberCount = 0;

void AVRTimeMonitor::MeasuringStartTime(uint8_t FunctionNumber)
{
  UpdateFunctionName(FunctionNumber);
  if (FunctionNumber == FunctionNumberCount)
  {
    FunctionNumberCount++;
    SafeToPrint = true;
  }
  if (FunctionNumberCount >= SIZE_LOOPS)
  {
    FunctionNumberCount++;
    if (FunctionNumberCount >= SIZE_LOOPS + 500)
    {
#ifdef __AVR_ATmega2560__
      FastSerialPrintln(PSTR("\n"));
#elif defined ESP32

#elif defined __arm__

#endif
      FunctionNumberCount = 0;
    }
  }
  StartTime = SCHEDULERTIME.GetMicros();
  EndTime = 0;
}

void AVRTimeMonitor::MeasuringFinishTime()
{
  if (EndTime == 0)
    EndTime = SCHEDULERTIME.GetMicros();
  AVRTotalTime = EndTime - StartTime;
  if (SafeToPrint)
  {
#ifdef __AVR_ATmega2560__
    FastSerialPrintln(PSTR("%-20s\tTempo Gasto:%.5f ms\n"),
                      GetFunctionName,
                      AVRTotalTime / 1000);
#elif defined ESP32

#elif defined __arm__

#endif
    SafeToPrint = false;
  }
}