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

#include "SETFLIGHTMODE.h"
#include "Common/VARIABLES.h"
#include "AUXFLIGHT.h"

void SetFlightModesToGCS()
{
  if ((NavigationMode == Do_Land_Settle) ||
      (NavigationMode == Do_LandInProgress) ||
      (NavigationMode == Do_Land_Detected) ||
      (NavigationMode == Do_Landed))
  {
    if (NavigationMode == Do_Landed)
    {
      FlightMode = 13;
    }
    else
    {
      FlightMode = 8;
    }
  }
  else
  {
    if (AcroControlAux)
    {
      FlightMode = 0;
    }
    if (IOCControlAux)
    {
      FlightMode = 5;
    }
    if (AltitudeHoldControlAux)
    {
      FlightMode = 2;
    }
    if (GPSHoldControlAux)
    {
      FlightMode = 4;
    }
    if (RTHControlAux)
    {
      FlightMode = 6;
    }
    if (SportControlAux)
    {
      FlightMode = 3;
    }
    if (AutoFlipControlAux)
    {
      FlightMode = 11;
    }
    if (AutoPilotControlAux)
    {
      FlightMode = 12;
    }
    if (AutoLandControlAux && !COMMAND_ARM_DISARM)
    {
      FlightMode = 8;
    }
    if (!AcroControlAux && !IOCControlAux && !AltitudeHoldControlAux &&
        !GPSHoldControlAux && !RTHControlAux && !SportControlAux &&
        !AutoFlipControlAux && !AutoPilotControlAux && !AutoLandControlAux)
    {
      FlightMode = 1;
    }
  }
}