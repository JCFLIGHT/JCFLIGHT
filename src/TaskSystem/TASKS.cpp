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

#include "TASKS.h"
#include "Scheduler/SCHEDULER.h"
#include "FunctionsLoop/LOOPS.h"
#include "Build/BOARDDEFS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

Task_Resources_Struct Task_Resources[TASK_COUNT] = {

    [TASK_SLOW_LOOP] = {
        .TaskName = "SLOW_LOOP",
        .TaskFunction = Slow_Loop,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(10, "Hz"),
        .StaticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_MEDIUM_LOOP] = {
        .TaskName = "MEDIUM_LOOP",
        .TaskFunction = Medium_Loop,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(50, "Hz"),
        .StaticPriority = TASK_PRIORITY_HIGH,
    },

#ifndef __AVR_ATmega2560__

    [TASK_FAST_MEDIUM_LOOP] = {
        .TaskName = "FAST_MEDIUM_LOOP",
        .TaskFunction = Fast_Medium_Loop,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(100, "Hz"),
        .StaticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_FAST_LOOP] = {
        .TaskName = "FAST_LOOP",
        .TaskFunction = Fast_Loop,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(400, "Hz"),
        .StaticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_SUPER_FAST_LOOP] = {
        .TaskName = "SUPER_FAST_LOOP",
        .TaskFunction = Super_Fast_Loop,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(500, "Hz"),
        .StaticPriority = TASK_PRIORITY_HIGH,
    },

#endif

    [TASK_INTEGRAL_LOOP] = {
        .TaskName = "INTEGRAL_LOOP",
        .TaskFunction = Integral_Loop,
#ifndef __AVR_ATmega2560__
        .DesiredPeriod = THIS_LOOP_FREQUENCY,
#else
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(60, "Hz"),
#endif
        .StaticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_SYSTEM_LOAD] = {
        .TaskName = "SYSTEM_LOAD",
        .TaskFunction = SystemLoad,
        .DesiredPeriod = SCHEDULER_SET_FREQUENCY(10, "Hz"),
        .StaticPriority = TASK_PRIORITY_HIGH,
    },

};