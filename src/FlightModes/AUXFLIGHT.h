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
#include "Arduino.h"
#include "Common/ENUM.h"
extern uint8_t SetFlightMode[SIZE_OF_FLIGHT_MODES];
extern uint8_t GPSHoldConfig,
    RTHConfig,
    IOCConfig,
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
    IOCControlAux,
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
  AUXFLIGHTCLASS(){};
  void LoadEEPROM(void);
  void SelectMode(void);
  void FlightModesAuxSelect(void);
};
extern AUXFLIGHTCLASS AUXFLIGHT;
#endif
