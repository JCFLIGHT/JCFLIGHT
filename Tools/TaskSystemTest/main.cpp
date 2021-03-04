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

void setup()
{
  Serial.begin(115200);
  TaskQueueClear();
  TaskQueueAdd(&Task_Resources[TASK_SLOW_LOOP]);
  SetTaskEnabled(TASK_SLOW_LOOP, ACTIVE_THIS_TASK);
  SetTaskEnabled(TASK_MEDIUM_LOOP, ACTIVE_THIS_TASK);
  SetTaskEnabled(TASK_FAST_LOOP, ACTIVE_THIS_TASK);
  SetTaskEnabled(TASK_SYSTEM_LOAD, ACTIVE_THIS_TASK);
}

void loop()
{
  TaskSystemRun();
}
