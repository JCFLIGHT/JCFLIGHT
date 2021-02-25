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

void SetFlightModeToGCS()
{
  if ((NavigationMode == Do_Land_Settle) ||
      (NavigationMode == Do_LandInProgress) ||
      (NavigationMode == Do_Land_Detected) ||
      (NavigationMode == Do_Landed))
  {
    if (NavigationMode == Do_Landed)
    {
      FlightMode = GCS_LANDED_MODE;
    }
    else
    {
      FlightMode = GCS_LAND_MODE;
    }
  }
  else
  {
    if (AcroControlAux)
    {
      FlightMode = GCS_ACRO_MODE;
    }
    if (IOCControlAux)
    {
      FlightMode = GCS_IOC_MODE;
    }
    if (AltitudeHoldControlAux)
    {
      FlightMode = GCS_ALTITUDE_HOLD_MODE;
    }
    if (GPSHoldControlAux)
    {
      FlightMode = GCS_POS_HOLD_MODE;
    }
    if (RTHControlAux)
    {
      FlightMode = GCS_RTH_MODE;
    }
    if (AttackControlAux)
    {
      FlightMode = GCS_ATTACK_MODE;
    }
    if (AutoFlipControlAux)
    {
      FlightMode = GCS_FLIP_MODE;
    }
    if (WayPointControlAux)
    {
      FlightMode = GCS_WAYPOINT_MODE;
    }
    if (AutoLandControlAux)
    {
      FlightMode = GCS_LAND_MODE;
    }
    if (!AcroControlAux && !IOCControlAux && !AltitudeHoldControlAux &&
        !GPSHoldControlAux && !RTHControlAux && !AttackControlAux &&
        !AutoFlipControlAux && !WayPointControlAux && !AutoLandControlAux)
    {
      FlightMode = GCS_STABILIZE_MODE;
    }
  }
}