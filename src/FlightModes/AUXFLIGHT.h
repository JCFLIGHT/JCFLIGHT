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

#ifndef AUXFLIGHT_H_
#define AUXFLIGHT_H_
#include "Build/LIBDEPENDENCIES.h"

extern uint8_t GPSHoldConfig,
    RTHConfig,
    SimpleConfig,
    AltitudeHoldConfig,
    AcroConfig,
    AttackConfig,
    AutoFlipConfig,
    WayPointConfig,
    FlightMode,
    ReceiverModel,
    ArmDisarmConfig,
    AutoLandConfig,
    ParachuteDetectTrigger;
extern int16_t AltitudeHoldControlAux,
    GPSHoldControlAux,
    RTHControlAux,
    SimpleControlAux,
    GimbalControlAux,
    AcroControlAux,
    AttackControlAux,
    AutoFlipControlAux,
    WayPointControlAux,
    ArmDisarmControlAux,
    AutoLandControlAux;

class AUXFLIGHTCLASS
{
public:
  void Initialization(void);
  void Update(void);

private:
  void SelectMode(void);
  void FlightModesAuxSelect(void);
};
extern AUXFLIGHTCLASS AUXFLIGHT;
#endif
