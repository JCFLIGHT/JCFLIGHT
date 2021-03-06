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

#ifndef SCHEDULERTIME_h
#define SCHEDULERTIME_h
#include "Build/LIBDEPENDENCIES.h"
class SchedulerTimeClass
{
public:
  void Initialization(void);
  void Sleep(uint16_t MillisSeconds);
  void MicroSecondsSleep(uint16_t MicroSeconds);
  uint32_t GetMillis(void);
  uint32_t GetMicros(void);
};
extern SchedulerTimeClass SCHEDULERTIME;
#endif
