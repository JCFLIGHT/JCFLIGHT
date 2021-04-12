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

#include "GPSORIENTATION.h"
#include "Scheduler/SCHEDULER.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "InertialNavigation/INS.h"
#include "Math/MATHSUPPORT.h"

void GPS_Orientation_Update()
{
  if (GetFrameStateOfMultirotor())
  {
    bool AltHoldControlApplied = ApplyAltitudeHoldControl();
    static Scheduler_Struct GPSControlTimer;
    if (!AltHoldControlApplied && Scheduler(&GPSControlTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
    {
      if (GPS_Parameters.Navigation.AutoPilot.Control.Enabled)
      {
        if (Get_Safe_State_For_Pos_Hold())
        {
          float DeltaTime = GPSControlTimer.ActualTime * 1e-6f;
          ApplyPosHoldPIDControl(DeltaTime);
        }
        GPS_Parameters.Navigation.AutoPilot.Control.Angle[ROLL] = ConvertDeciDegreesToDegrees(GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS.Math.Cosine_Yaw - GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS.Math.Sine_Yaw);
        GPS_Parameters.Navigation.AutoPilot.Control.Angle[PITCH] = ConvertDeciDegreesToDegrees(GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] * INS.Math.Sine_Yaw + GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] * INS.Math.Cosine_Yaw);
      }
      else
      {
        GPS_Parameters.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
        GPS_Parameters.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
        GPS_Parameters.Navigation.AutoPilot.Control.Angle[YAW] = 0;
      }
    }
  }
  else
  {
    if (GPS_Parameters.Navigation.AutoPilot.Control.Enabled)
    {
      AirPlaneUpdateNavigation();
    }
    else
    {
      GPS_Parameters.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
      GPS_Parameters.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
      GPS_Parameters.Navigation.AutoPilot.Control.Angle[YAW] = 0;
    }
  }
}
