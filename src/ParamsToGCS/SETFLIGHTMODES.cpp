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

#include "SETFLIGHTMODES.h"
#include "FlightModes/AUXFLIGHT.h"
#include "GPSNavigation/NAVIGATION.h"

void SetFlightModeToGCS(void)
{
  if ((GPS_Resources.Mode.Navigation == DO_LAND_SETTLE) ||
      (GPS_Resources.Mode.Navigation == DO_LAND_DESCENT) ||
      (GPS_Resources.Mode.Navigation == DO_LAND_DETECTED) ||
      (GPS_Resources.Mode.Navigation == DO_LANDED))
  {
    if (GPS_Resources.Mode.Navigation == DO_LANDED)
    {
      AUXFLIGHT.FlightMode = GCS_LANDED_MODE;
    }
    else
    {
      AUXFLIGHT.FlightMode = GCS_LAND_MODE;
    }
  }
  else
  {
    if (AUXFLIGHT.GetModeState[STABILIZE_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_ACRO_MODE;
    }
    else if (AUXFLIGHT.GetModeState[SIMPLE_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_SIMPLE_MODE;
    }
    else if (AUXFLIGHT.GetModeState[ALTITUDE_HOLD_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_ALTITUDE_HOLD_MODE;
    }
    else if (AUXFLIGHT.GetModeState[POS_HOLD_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_POS_HOLD_MODE;
    }
    else if (AUXFLIGHT.GetModeState[RTH_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_RTH_MODE;
    }
    else if (AUXFLIGHT.GetModeState[ATTACK_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_ATTACK_MODE;
    }
    else if (AUXFLIGHT.GetModeState[FLIP_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_FLIP_MODE;
    }
    else if (AUXFLIGHT.GetModeState[WAYPOINT_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_WAYPOINT_MODE;
    }
    else if (AUXFLIGHT.GetModeState[LAND_MODE])
    {
      AUXFLIGHT.FlightMode = GCS_LAND_MODE;
    }
    else
    {
      AUXFLIGHT.FlightMode = GCS_STABILIZE_MODE;
    }
  }
}