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
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "PID/PIDXYZ.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPSNavigation/INSNAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"
#include "FailSafe/FAILSAFE.h"
#include "Yaw/HEADINGHOLD.h"
#include "GPS/GPSUBLOX.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "AHRS/AHRS.h"
#include "Barometer/BAROBACKEND.h"
#include "GPS/GPSSTATES.h"

bool Do_Altitude_Hold = false;
bool Do_RTH_Or_Land_Call_Alt_Hold = false;
bool Do_Pos_Hold_Call_Alt_Hold = false;

bool Get_Multirotor_GPS_FlightModes_Once()
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

bool Get_AirPlane_GPS_FlightModes_Once()
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
      if (Get_Multirotor_GPS_FlightModes_Once())
      {
        if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
        {
          GPSParameters.Mode.Flight = GPS_MODE_RTH;
          GPSParameters.Mode.Navigation = DO_START_RTH;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          Do_Mode_RTH_Now();
        }
        else if (IS_FLIGHT_MODE_ACTIVE(POS_HOLD_MODE))
        {
          GPSParameters.Mode.Flight = GPS_MODE_HOLD;
          GPSParameters.Mode.Navigation = DO_POSITION_HOLD;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          Do_Pos_Hold_Call_Alt_Hold = true;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
        }
        else if (IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
        {
          GPSParameters.Mode.Flight = GPS_MODE_HOLD;
          GPSParameters.Mode.Navigation = DO_LAND_INIT;
          Do_Pos_Hold_Call_Alt_Hold = false;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
        }
        else
        {
          GPSParameters.Mode.Flight = GPS_MODE_NONE;
          Do_Pos_Hold_Call_Alt_Hold = false;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          DISABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          GPS_Reset_Navigation();
        }
      }
    }
    else
    {
      if (Get_GPS_Only_Flight_Modes_In_Use())
      {
        if (Do_RTH_Or_Land_Call_Alt_Hold)
        {
          SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
        }
        GPSParameters.Mode.Flight = GPS_MODE_NONE;
      }
      GPS_Reset_Navigation();
    }
  }
  else
  {
    Do_RTH_Or_Land_Call_Alt_Hold = false;
    GPSParameters.Mode.Flight = GPS_MODE_NONE;
    GPS_Reset_Navigation();
  }

  if (IS_FLIGHT_MODE_ACTIVE(ALTITUDE_HOLD_MODE) || Do_Pos_Hold_Call_Alt_Hold || Do_WayPoint_Call_Alt_Hold || Do_RTH_Or_Land_Call_Alt_Hold)
  {
    if (!Do_Altitude_Hold)
    {
      Do_Altitude_Hold = true;
    }
  }
  else
  {
    Do_Altitude_Hold = false;
  }

  if (!IS_FLIGHT_MODE_ACTIVE(RTH_MODE) && !IS_FLIGHT_MODE_ACTIVE(LAND_MODE))
  {
    Do_RTH_Or_Land_Call_Alt_Hold = false;
  }
}

void ProcessFlightModesToAirPlane()
{
  if (!GetFrameStateOfAirPlane())
  {
    return;
  }

  if (Get_State_Armed_With_GPS())
  {
    if (Get_GPS_Only_Flight_Modes_In_Use() && !IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE))
    {
      ENABLE_THIS_FLIGHT_MODE(STABILIZE_MODE); //FORÇA O MODO STABILIZE EM MODO NAVEGAÇÃO POR GPS
    }

    if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE))
    {
      ENABLE_DISABLE_FLIGHT_MODE_WITH_DEPENDENCY(CIRCLE_MODE, SticksInAutoPilotPosition(20));
    }

    if (Get_AirPlane_GPS_FlightModes_Once())
    {
      if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
      {
        Set_Next_Point_To_Navigation(GPSParameters.Home.Coordinates[COORD_LATITUDE], GPSParameters.Home.Coordinates[COORD_LONGITUDE]);
        GPSParameters.Mode.Flight = DO_RTH_ENROUTE;
        AltitudeHold_For_Plane = GPSParameters.Navigation.Misc.Get.Altitude;
        ENABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      }
      else
      {
        if (IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE))
        {
          GPSParameters.Mode.Flight = GPS_MODE_HOLD;
          GPSParameters.Mode.Navigation = DO_POSITION_HOLD;
          AltitudeHold_For_Plane = GPSParameters.Navigation.Misc.Get.Altitude;
          Set_Next_Point_To_Navigation(GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE]);
          DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
        }
        else
        {
          GPSParameters.Mode.Flight = GPS_MODE_NONE;
          GPSParameters.Mode.Navigation = DO_NONE;
          GPS_Reset_Navigation();
          DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
        }
      }
    }
  }
  else
  {
    GPSParameters.Mode.Flight = GPS_MODE_NONE;
    GPSParameters.Mode.Navigation = DO_NONE;
  }

  if (IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE))
  {
    if (!Do_Altitude_Hold)
    {
      Do_Altitude_Hold = true;
    }
  }
  else
  {
    Do_Altitude_Hold = false;
  }
}

void FlightModesUpdate()
{
  ProcessFlightModesToMultirotor();
  ProcessFlightModesToAirPlane();

  if (IS_FLIGHT_MODE_ACTIVE_ONCE(HEADING_HOLD_MODE))
  {
    HeadingHoldTarget = Attitude.EulerAngles.Yaw;
  }

  GPSParameters.Navigation.AutoPilot.Control.Enabled = Get_GPS_Flight_Modes_And_Navigation_In_Use();
}