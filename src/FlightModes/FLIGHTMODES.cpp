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

#include "FLIGHTMODES.h"
#include "Common/VARIABLES.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "PID/PIDXYZ.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"

bool Multirotor_GPS_FlightModes_Once()
{
  static uint8_t Previous_Mode = 0;
  uint8_t Check_Actual_State = ((IS_FLIGHT_MODE_ACTIVE(LAND_MODE) << 2) +
                                (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE) << 1) +
                                (IS_FLIGHT_MODE_ACTIVE(RTH_MODE)));
  if (Previous_Mode != Check_Actual_State)
  {
    Previous_Mode = Check_Actual_State;
    return true;
  }
  return false;
}

bool AirPlane_GPS_FlightModes_Once()
{
  static uint8_t Previous_Mode = 0;
  uint8_t Check_Actual_State = ((IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) << 1) +
                                (IS_FLIGHT_MODE_ACTIVE(RTH_MODE)));
  if (Previous_Mode != Check_Actual_State)
  {
    Previous_Mode = Check_Actual_State;
    return true;
  }
  return false;
}

void ProcessFlightModesToMultirotor()
{
  if (!GetFrameStateOfMultirotor())
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE) || GPSHold_CallBaro || Mission_BaroMode || Do_GPS_Altitude)
  {
    if (!Do_AltitudeHold_Mode)
    {
      Do_AltitudeHold_Mode = true;
    }
  }
  else
  {
    Do_AltitudeHold_Mode = false;
  }

  if (!IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
  {
    Do_GPS_Altitude = false;
  }

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    if (Get_GPS_In_Good_Condition())
    {
      if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
      {
        DISABLE_THIS_FLIGHT_MODE(POS_HOLD_MODE);
      }
      else
      {
        if (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
        {
          ENABLE_DISABLE_FLIGHT_MODE_WITH_DEPENDENCY(POS_HOLD_MODE, SticksInAutoPilotPosition(20));
        }
      }
      if (Multirotor_GPS_FlightModes_Once())
      {
        if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
        {
          GPSHold_CallBaro = true;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          Do_Mode_RTH_Now();
        }
        else if (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
        {
          GPS_Flight_Mode = GPS_MODE_HOLD;
          Do_GPS_Altitude = false;
          GPSHold_CallBaro = true;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
          NavigationMode = Do_PositionHold;
        }
        else if (IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
        {
          GPS_Flight_Mode = GPS_MODE_HOLD;
          Do_GPS_Altitude = true;
          SetThisPointToPositionHold();
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          NavigationMode = Do_Land_Init;
        }
        else
        {
          GPS_Flight_Mode = GPS_MODE_NONE;
          GPSHold_CallBaro = false;
          Do_GPS_Altitude = false;
          DISABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          GPS_Reset_Navigation();
        }
      }
    }
    else
    {
      if (GPS_Flight_Mode != GPS_MODE_NONE)
      {
        if (Do_GPS_Altitude)
        {
          SetAltitudeHold(ALTITUDE.EstimatedAltitude);
        }
        GPS_Flight_Mode = GPS_MODE_NONE;
      }
      GPS_Reset_Navigation();
    }
  }
  else
  {
    Do_GPS_Altitude = false;
    GPS_Flight_Mode = GPS_MODE_NONE;
    GPS_Reset_Navigation();
  }
}

void ProcessFlightModesToAirPlane()
{
  if (!GetFrameStateOfAirPlane())
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
  {
    if (!Do_AltitudeHold_Mode)
    {
      Do_AltitudeHold_Mode = true;
    }
  }
  else
  {
    Do_AltitudeHold_Mode = false;
  }

  if (Get_State_Armed_With_GPS())
  {
    if (GPS_Flight_Mode != GPS_MODE_NONE && !Do_Stabilize_Mode)
    {
      Do_Stabilize_Mode = true;
    }
    if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE))
    {
      ENABLE_DISABLE_FLIGHT_MODE_WITH_DEPENDENCY(CIRCLE_MODE, SticksInAutoPilotPosition(20));
    }
    if (AirPlane_GPS_FlightModes_Once())
    {
      if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
      {
        Set_Next_Point_To_Navigation(&Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1]);
        GPS_Flight_Mode = Do_RTH_Enroute;
        GPS_AltitudeHold_For_Plane = GPS_Altitude;
        ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      }
      else
      {
        if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE))
        {
          NavigationMode = Do_PositionHold;
          GPS_Flight_Mode = GPS_MODE_HOLD;
          GPS_AltitudeHold_For_Plane = GPS_Altitude;
          Set_Next_Point_To_Navigation(&GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1]);
          DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
        }
        else
        {
          GPS_Flight_Mode = GPS_MODE_NONE;
          NavigationMode = Do_None;
          GPS_Reset_Navigation();
          DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
        }
      }
    }
  }
  else
  {
    GPS_Flight_Mode = GPS_MODE_NONE;
    NavigationMode = Do_None;
  }
}

void FlightModesUpdate()
{
  if (IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE) || Fail_Safe_Event)
  {
    if (!Do_Stabilize_Mode)
    {
      PIDXYZ.IntegralAccError[ROLL] = 0;
      PIDXYZ.IntegralAccError[PITCH] = 0;
      Do_Stabilize_Mode = true;
    }
  }
  else
  {
    if (Do_Stabilize_Mode)
    {
      PIDXYZ.IntegralGyroError[ROLL] = 0;
      PIDXYZ.IntegralGyroError[PITCH] = 0;
    }
    Do_Stabilize_Mode = false;
  }

  if (IS_FLIGHT_MODE_ACTIVE(HEADING_HOLD_MODE))
  {
    if (!Do_HeadingHold_Mode)
    {
      HeadingHoldTarget = ATTITUDE.AngleOut[YAW];
      Do_HeadingHold_Mode = true;
    }
  }
  else
  {
    Do_HeadingHold_Mode = false;
  }

  ProcessFlightModesToMultirotor();
  ProcessFlightModesToAirPlane();
}