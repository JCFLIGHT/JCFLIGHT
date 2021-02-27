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

#ifndef PARACHUTE_H_
#define PARACHUTE_H_
#include "Build/LIBDEPENDENCIES.h"
class ParachuteClass
{
public:
  bool GetSafeStateToDisarmMotors();
  void Auto_Do_Now(bool ActiveParachute);
  void Manual_Do_Now();
  void Manual_Detect_Channel();

private:
  bool ParachuteInAuto = false;
  bool ParachuteReleased = false;
  uint16_t ManualDetectTrigger = 0;
  uint32_t OverFlowTime = 0;
  bool Released();
  bool ReleasedOverFlowTime();
};
extern ParachuteClass PARACHUTE;
#endif
