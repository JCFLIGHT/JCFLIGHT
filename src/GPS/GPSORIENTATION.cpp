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
#include "Common/VARIABLES.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "InertialNavigation/INS.h"

void GPS_Orientation_Update()
{
  if (GetFrameStateOfMultirotor())
  {
    bool AltHoldControlApplied = ApplyAltitudeHoldControl();
    static Scheduler_Struct GPSControlTimer;
    if (!AltHoldControlApplied && Scheduler(&GPSControlTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
    {
      if ((GPS_Flight_Mode != GPS_MODE_NONE) && Home_Point && (NavigationMode != Do_None))
      {
        if (NavStateForPosHold())
        {
          float DeltaTime = GPSControlTimer.ActualTime * 1e-6f;
          ApplyPosHoldPIDControl(&DeltaTime);
        }
        GPS_Angle[ROLL] = (GPS_Navigation_Array[1] * INERTIALNAVIGATION.Cosine_Yaw - GPS_Navigation_Array[0] * INERTIALNAVIGATION.Sine_Yaw) / 10;
        GPS_Angle[PITCH] = (GPS_Navigation_Array[1] * INERTIALNAVIGATION.Sine_Yaw + GPS_Navigation_Array[0] * INERTIALNAVIGATION.Cosine_Yaw) / 10;
      }
      else
      {
        GPS_Angle[ROLL] = 0;
        GPS_Angle[PITCH] = 0;
        GPS_Angle[YAW] = 0;
      }
    }
  }
  else
  {
    if ((GPS_Flight_Mode != GPS_MODE_NONE) && Home_Point && (NavigationMode != Do_None))
    {
      AirPlaneUpdateNavigation();
      //ApplyAltitudeHoldControl(); //É REALMENTE CORRETO USAR ESSA FUNÇÃO PARA O ALT-HOLD EM AEROS???
    }
    else
    {
      GPS_Angle[ROLL] = 0;
      GPS_Angle[PITCH] = 0;
      GPS_Angle[YAW] = 0;
    }
  }
}
