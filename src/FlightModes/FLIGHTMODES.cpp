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
#include "Math/AVRMATH.h"
#include "RadioControl/STATES.h"
#include "FrameStatus/FRAMESTATUS.h"

//APENAS PARA AERO E ASA-FIXA
bool GPS_HOLD_MODE_FW = false;
bool GPS_HOME_MODE_FW = false;
bool CLIMBOUT_FW = false;
bool GPSNavReset = true;

void FlightModesUpdate()
{
  if (SetFlightModes[STABILIZE_MODE] || (Fail_Safe_Event))
  {
    if (!Do_Stabilize_Mode)
    {
      IntegralAccError[ROLL] = 0;
      IntegralAccError[PITCH] = 0;
      Do_Stabilize_Mode = true;
    }
  }
  else
  {
    if (Do_Stabilize_Mode)
    {
      IntegralGyroError[ROLL] = 0;
      IntegralGyroError[PITCH] = 0;
    }
    Do_Stabilize_Mode = false;
  }

  if (GetFrameStateOfMultirotor())
  {
    SetFlightModes[ALTITUDE_HOLD_MODE] = (SetFlightModes[ALTITUDE_HOLD_MODE] || Do_GPS_Altitude);
    if (SetFlightModes[ALTITUDE_HOLD_MODE] || GPSHold_CallBaro || Mission_BaroMode)
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
  }

  if (SetFlightModes[HEADING_HOLD_MODE])
  {
    if (!Do_HeadingHold_Mode)
    {
      HeadingHoldTarget = ATTITUDE.CalculedHeading;
      Do_HeadingHold_Mode = true;
    }
  }
  else
  {
    Do_HeadingHold_Mode = false;
  }

  if (SetFlightModes[IOC_MODE])
  {
    if (!Do_IOC_Mode)
    {
      Do_IOC_Mode = true;
    }
  }
  else
  {
    Do_IOC_Mode = false;
  }

  if (GetFrameStateOfAirPlane())
  {
    if (GPS_NumberOfSatellites >= 5 && COMMAND_ARM_DISARM)
    {
      if (GPS_Flight_Mode != GPS_MODE_NONE && !Do_Stabilize_Mode)
        Do_AltitudeHold_Mode = true;
      if (SetFlightModes[RTH_MODE] || Fail_Safe_Event)
      {
        if (!GPS_HOME_MODE_FW)
        {
          GPS_HOME_MODE_FW = true;
          GPS_HOLD_MODE_FW = false;
          GPSNavReset = false;
          Set_Points_To_Navigation(&Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1], &GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1]);
          GPS_Flight_Mode = Do_RTH_Enroute;
          GPS_AltitudeHold_For_Plane = GPS_Altitude;
          CLIMBOUT_FW = true;
        }
      }
      else
      {
        GPS_HOME_MODE_FW = false;
        if (SetFlightModes[GPS_HOLD_MODE] && SticksInAutoPilotPosition(20))
        {
          if (!GPS_HOLD_MODE_FW)
          {
            GPS_HOLD_MODE_FW = true;
            GPSNavReset = false;
            NavigationMode = Do_PositionHold;
            GPS_Flight_Mode = GPS_MODE_HOLD;
            GPS_AltitudeHold_For_Plane = GPS_Altitude;
            Set_Points_To_Navigation(&GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1], &GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1]);
            CLIMBOUT_FW = false;
          }
        }
        else
        {
          GPS_HOLD_MODE_FW = false;
          GPS_HOME_MODE_FW = false;
          GPS_Flight_Mode = GPS_MODE_NONE;
          NavigationMode = Do_None;
          if (!GPSNavReset)
          {
            GPS_Reset_Navigation();
            CLIMBOUT_FW = false;
            GPSNavReset = true;
          }
        }
      }
    }
    else
    {
      GPS_HOLD_MODE_FW = false;
      GPS_HOME_MODE_FW = false;
      GPS_Flight_Mode = GPS_MODE_NONE;
      NavigationMode = Do_None;
    }
  }

  if (GetFrameStateOfMultirotor())
  {
    if (!SetFlightModes[RTH_MODE] && !SetFlightModes[LAND_MODE])
    {
      if (Do_GPS_Altitude)
        Do_GPS_Altitude = false;
    }
    if (COMMAND_ARM_DISARM)
    {
      if (GPS_3DFIX)
      {
        if (GPS_NumberOfSatellites >= 5)
        {
          if (SetFlightModes[RTH_MODE])
          {
            SetFlightModes[GPS_HOLD_MODE] = false;
          }
          else
          {
            if (SetFlightModes[GPS_HOLD_MODE])
              SetFlightModes[GPS_HOLD_MODE] = SticksInAutoPilotPosition(20);
          }
          if (CheckSafeStateToGPSMode())
          {
            if (SetFlightModes[RTH_MODE])
            {
              GPSHold_CallBaro = true;
              SetFlightModes[HEADING_HOLD_MODE] = true;
              Do_Mode_RTH_Now();
            }
            else if (SetFlightModes[GPS_HOLD_MODE])
            {
              GPS_Flight_Mode = GPS_MODE_HOLD;
              Do_GPS_Altitude = false;
              GPSHold_CallBaro = true;
              SetFlightModes[HEADING_HOLD_MODE] = true;
              SetThisPointToPositionHold();
              NavigationMode = Do_PositionHold;
            }
            else if (SetFlightModes[LAND_MODE])
            {
              GPS_Flight_Mode = GPS_MODE_HOLD;
              Do_GPS_Altitude = true;
              SetThisPointToPositionHold();
              SetFlightModes[HEADING_HOLD_MODE] = true;
              NavigationMode = Do_Land_Init;
            }
            else
            {
              GPS_Flight_Mode = GPS_MODE_NONE;
              GPSHold_CallBaro = false;
              Do_GPS_Altitude = false;
              SetFlightModes[HEADING_HOLD_MODE] = false;
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
              SetAltitudeHold(ALTITUDE.EstimateAltitude);
            }
            GPS_Flight_Mode = GPS_MODE_NONE;
            NavigationMode = Do_None;
          }
          GPS_Navigation_Array[0] = 0;
          GPS_Navigation_Array[1] = 0;
        }
      }
      else
      {
        if (GPS_Flight_Mode != GPS_MODE_NONE)
        {
          if (Do_GPS_Altitude)
            SetAltitudeHold(ALTITUDE.EstimateAltitude);
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
}

bool CheckSafeStateToGPSMode()
{
  static uint8_t Previous_Mode = 0;
  uint8_t Check_Actual_State = ((SetFlightModes[LAND_MODE] << 2) +
                                (SetFlightModes[GPS_HOLD_MODE] << 1) +
                                (SetFlightModes[RTH_MODE]));
  if (Previous_Mode != Check_Actual_State)
  {
    Previous_Mode = Check_Actual_State;
    return true;
  }
  else
    return false;
}