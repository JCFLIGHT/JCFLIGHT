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

#include "FAILSAFE.h"
#include "Common/VARIABLES.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "FlightModes/AUXFLIGHT.h"

#define THIS_LOOP_RATE 50     //HZ
#define FAILSAFE_DELAY 1      //SEGUNDO
#define FAILSAFE_DELAY2 0.25f //MS

void FailSafeCheck()
{
  if (Fail_Safe_System > (THIS_LOOP_RATE * FAILSAFE_DELAY2) && COMMAND_ARM_DISARM)
  {
    ImmediatelyFailSafe = true;
  }
  else
  {
    ImmediatelyFailSafe = false;
  }
  if (Fail_Safe_System > (THIS_LOOP_RATE * FAILSAFE_DELAY) && COMMAND_ARM_DISARM)
  {
    NormalizeFailSafe();
    Fail_Safe_Event = true;
    RadioControllOutput[THROTTLE] = 1500;
    RadioControllOutput[YAW] = 1500;
    RadioControllOutput[PITCH] = 1500;
    RadioControllOutput[ROLL] = 1500;
    RadioControllOutput[AUX1] = 1000;
    RadioControllOutput[AUX2] = 1000;
    RadioControllOutput[AUX3] = 1000;
    RadioControllOutput[AUX4] = 1000;
    RadioControllOutput[AUX5] = 1000;
    RadioControllOutput[AUX6] = 1000;
    RadioControllOutput[AUX7] = 1000;
    RadioControllOutput[AUX8] = 1000;
  }
  else
  {
    Fail_Safe_Event = false;
  }
  if (COMMAND_ARM_DISARM)
  {
    Fail_Safe_System++;
  }
  else
  {
    if (!RTHControlAux)
    {
      SetFlightModes[RTH_MODE] = false;
    }
  }
}

void NormalizeFailSafe()
{
  SetFlightModes[STABILIZE_MODE] = true;
  SetFlightModes[IOC_MODE] = false;
  SetFlightModes[ALTITUDE_HOLD_MODE] = false;
  SetFlightModes[GPS_HOLD_MODE] = false;
  SetFlightModes[RTH_MODE] = true;
  SetFlightModes[ATACK_MODE] = false;
  SetFlightModes[FLIP_MODE] = false;
  SetFlightModes[WAYPOINT_MODE] = false;
}
