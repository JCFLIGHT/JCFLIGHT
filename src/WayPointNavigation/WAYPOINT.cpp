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

#include "WAYPOINT.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "GPSNavigation/NAVIGATION.h"
#include "RadioControl/RCCONFIG.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/STICKS.h"
#include "Scheduler/SCHEDULER.h"
#include "Yaw/HEADINGHOLD.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "Param/PARAM.h"
#include "GPSNavigation/NAVIGATIONGEO.h"
#include "GPSNavigation/INSNAVIGATION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Barometer/BAROBACKEND.h"

WayPointClass WAYPOINT;
WayPoint_Resources_Struct WayPointResources;
_GetWayPointPacketOne GetWayPointPacketOne;
_GetWayPointPacketTwo GetWayPointPacketTwo;

#define THROTTLE_TAKEOFF_ASCENT 1600    //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF ATÉ CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_TAKEOFF_NORMALIZE 1500 //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF AO CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_CANCEL_TAKEOFF 1450    //VALOR DO THROTTLE LIDO DO RECEPTOR PARA CANCELAR O AUTO-TAKEOFF E VOLTAR AO CONTROLE NORMAL
#define THROTTLE_INCREMENT 100          //NÚMERO DE INCREMENTAÇÕES A CADA ESTOURO DE TEMPO DEFINIDO PELO PARAMETRO "THROTTLE_INCREMENT_TIME"
#define THROTTLE_INCREMENT_TIME 1       //INCREMENTA A CADA 0.10 SEGUNDOS

void WayPointClass::Initialization(void)
{
  for (int16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
  {
    if (AddressCount < COORDINATES_ADDR_TO_COMPARE)
    {
      WayPointResources.Mission.Coordinates.Latitude[WayPointResources.Storage.ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }
    else
    {
      WayPointResources.Mission.Coordinates.Longitude[WayPointResources.Storage.ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }

    WayPointResources.Storage.ArrayCount++;

    if (WayPointResources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      WayPointResources.Storage.ArrayCount = 0;
    }
  }

  WayPointResources.Storage.ArrayCount = 0;

  for (int16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
  {
    if (AddressCount < GPS_HOLD_TIMED_ADDR_TO_COMPARE)
    {
      WayPointResources.Mission.OthersParams.PositionHoldTime[WayPointResources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= GPS_HOLD_TIMED_ADDR_TO_COMPARE && AddressCount < (FLIGHT_MODE_ADDR_TO_COMPARE - 1))
    {
      WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= FLIGHT_MODE_ADDR_TO_COMPARE && AddressCount < (ALTITUDE_ADDR_TO_COMPARE - 1))
    {
      WayPointResources.Mission.OthersParams.Altitude[WayPointResources.Storage.ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }

    WayPointResources.Storage.ArrayCount++;

    if (WayPointResources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      WayPointResources.Storage.ArrayCount = 0;
    }
  }
}

static void Push_WayPoint_Parameters(void)
{
  //NÃO VAMOS ZERAR AS VARIAVEIS,TALVEZ CONTÉM ALGO NA EEPROM
  if (GetWayPointPacketOne.LatitudeOne == 0 || GetWayPointPacketOne.LongitudeOne == 0)
  {
    return;
  }

  //OBTÉM TODAS AS LATITUDES DE CADA WAYPOINT
  WayPointResources.Mission.Coordinates.Latitude[0] = GetWayPointPacketOne.LatitudeOne;
  WayPointResources.Mission.Coordinates.Latitude[1] = GetWayPointPacketOne.LatitudeTwo;
  WayPointResources.Mission.Coordinates.Latitude[2] = GetWayPointPacketOne.LatitudeThree;
  WayPointResources.Mission.Coordinates.Latitude[3] = GetWayPointPacketOne.LatitudeFour;
  WayPointResources.Mission.Coordinates.Latitude[4] = GetWayPointPacketOne.LatitudeFive;
  WayPointResources.Mission.Coordinates.Latitude[5] = GetWayPointPacketTwo.LatitudeSix;
  WayPointResources.Mission.Coordinates.Latitude[6] = GetWayPointPacketTwo.LatitudeSeven;
  WayPointResources.Mission.Coordinates.Latitude[7] = GetWayPointPacketTwo.LatitudeEight;
  WayPointResources.Mission.Coordinates.Latitude[8] = GetWayPointPacketTwo.LatitudeNine;
  WayPointResources.Mission.Coordinates.Latitude[9] = GetWayPointPacketTwo.LatitudeTen;

  //OBTÉM TODAS AS LONGITUDES DE CADA WAYPOINT
  WayPointResources.Mission.Coordinates.Longitude[0] = GetWayPointPacketOne.LongitudeOne;
  WayPointResources.Mission.Coordinates.Longitude[1] = GetWayPointPacketOne.LongitudeTwo;
  WayPointResources.Mission.Coordinates.Longitude[2] = GetWayPointPacketOne.LongitudeThree;
  WayPointResources.Mission.Coordinates.Longitude[3] = GetWayPointPacketOne.LongitudeFour;
  WayPointResources.Mission.Coordinates.Longitude[4] = GetWayPointPacketOne.LongitudeFive;
  WayPointResources.Mission.Coordinates.Longitude[5] = GetWayPointPacketTwo.LongitudeSix;
  WayPointResources.Mission.Coordinates.Longitude[6] = GetWayPointPacketTwo.LongitudeSeven;
  WayPointResources.Mission.Coordinates.Longitude[7] = GetWayPointPacketTwo.LongitudeEight;
  WayPointResources.Mission.Coordinates.Longitude[8] = GetWayPointPacketTwo.LongitudeNine;
  WayPointResources.Mission.Coordinates.Longitude[9] = GetWayPointPacketTwo.LongitudeTen;

  //OBTÉM A ALTITUDE DE SUBIDA DE CADA WAYPOINT
  WayPointResources.Mission.OthersParams.Altitude[0] = GetWayPointPacketOne.AltitudeOne;
  WayPointResources.Mission.OthersParams.Altitude[1] = GetWayPointPacketOne.AltitudeTwo;
  WayPointResources.Mission.OthersParams.Altitude[2] = GetWayPointPacketOne.AltitudeThree;
  WayPointResources.Mission.OthersParams.Altitude[3] = GetWayPointPacketOne.AltitudeFour;
  WayPointResources.Mission.OthersParams.Altitude[4] = GetWayPointPacketOne.AltitudeFive;
  WayPointResources.Mission.OthersParams.Altitude[5] = GetWayPointPacketTwo.AltitudeSix;
  WayPointResources.Mission.OthersParams.Altitude[6] = GetWayPointPacketTwo.AltitudeSeven;
  WayPointResources.Mission.OthersParams.Altitude[7] = GetWayPointPacketTwo.AltitudeEight;
  WayPointResources.Mission.OthersParams.Altitude[8] = GetWayPointPacketTwo.AltitudeNine;
  WayPointResources.Mission.OthersParams.Altitude[9] = GetWayPointPacketTwo.AltitudeTen;

  //OBTÉM OS MODOS DE VOO DE CADA WAYPOINT
  WayPointResources.Mission.OthersParams.FlightMode[0] = GetWayPointPacketOne.FlightModeOne;
  WayPointResources.Mission.OthersParams.FlightMode[1] = GetWayPointPacketOne.FlightModeTwo;
  WayPointResources.Mission.OthersParams.FlightMode[2] = GetWayPointPacketOne.FlightModeThree;
  WayPointResources.Mission.OthersParams.FlightMode[3] = GetWayPointPacketOne.FlightModeFour;
  WayPointResources.Mission.OthersParams.FlightMode[4] = GetWayPointPacketOne.FlightModeFive;
  WayPointResources.Mission.OthersParams.FlightMode[5] = GetWayPointPacketTwo.FlightModeSix;
  WayPointResources.Mission.OthersParams.FlightMode[6] = GetWayPointPacketTwo.FlightModeSeven;
  WayPointResources.Mission.OthersParams.FlightMode[7] = GetWayPointPacketTwo.FlightModeEight;
  WayPointResources.Mission.OthersParams.FlightMode[8] = GetWayPointPacketTwo.FlightModeNine;
  WayPointResources.Mission.OthersParams.FlightMode[9] = GetWayPointPacketTwo.FlightModeTen;

  //OBTÉM O TEMPO DE VOO DO GPS-HOLD DE CADA WP
  WayPointResources.Mission.OthersParams.PositionHoldTime[0] = GetWayPointPacketOne.GPSHoldTimedOne;
  WayPointResources.Mission.OthersParams.PositionHoldTime[1] = GetWayPointPacketOne.GPSHoldTimedTwo;
  WayPointResources.Mission.OthersParams.PositionHoldTime[2] = GetWayPointPacketOne.GPSHoldTimedThree;
  WayPointResources.Mission.OthersParams.PositionHoldTime[3] = GetWayPointPacketOne.GPSHoldTimedFour;
  WayPointResources.Mission.OthersParams.PositionHoldTime[4] = GetWayPointPacketOne.GPSHoldTimedFive;
  WayPointResources.Mission.OthersParams.PositionHoldTime[5] = GetWayPointPacketTwo.GPSHoldTimedSix;
  WayPointResources.Mission.OthersParams.PositionHoldTime[6] = GetWayPointPacketTwo.GPSHoldTimedSeven;
  WayPointResources.Mission.OthersParams.PositionHoldTime[7] = GetWayPointPacketTwo.GPSHoldTimedEight;
  WayPointResources.Mission.OthersParams.PositionHoldTime[8] = GetWayPointPacketTwo.GPSHoldTimedNine;
  WayPointResources.Mission.OthersParams.PositionHoldTime[9] = GetWayPointPacketTwo.GPSHoldTimedTen;
}

void WayPointClass::Erase(void)
{
  //OS ÚLTIMOS ENDEREÇOS SÃO OS DE ARMAZENAMENTO DAS ALTITUDES DE SUBIDA DE CADA MISSÃO
  STORAGEMANAGER.Erase(INITIAL_ADDR_OF_COORDINATES, (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM));
  for (int16_t AddressCount = (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM + 1); AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount++)
  {
    //PADRÃO DE 10 METROS PARA AS ALTITUDES DE VOO POR WAYPOINT
    STORAGEMANAGER.Write_8Bits(AddressCount, 10);
  }
}

static void Store_And_Clear_WayPoints(void)
{
  if (WayPointResources.Storage.Function == WAYPOINT_STORAGE_RESET)
  {
    for (uint8_t CountVector = 0; CountVector < 10; CountVector++)
    {
      WayPointResources.Mission.Coordinates.Latitude[CountVector] = 0;
      WayPointResources.Mission.Coordinates.Longitude[CountVector] = 0;
      WayPointResources.Mission.OthersParams.FlightMode[CountVector] = 0;
      WayPointResources.Mission.OthersParams.Altitude[CountVector] = 0;
      WayPointResources.Mission.OthersParams.PositionHoldTime[CountVector] = 0;
      GetWayPointPacketOne.Reset();
      GetWayPointPacketTwo.Reset();
      WAYPOINT.Erase();
    }
    WayPointResources.Storage.Function = WAYPOINT_STORAGE_NONE;
  }

  if (WayPointResources.Storage.Function == WAYPOINT_STORAGE_SAVE)
  {
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {

      WayPointResources.Storage.ArrayCount = 0;

      for (int16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
      {
        if (AddressCount < COORDINATES_ADDR_TO_COMPARE)
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPointResources.Mission.Coordinates.Latitude[WayPointResources.Storage.ArrayCount]);
        }
        else
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPointResources.Mission.Coordinates.Longitude[WayPointResources.Storage.ArrayCount]);
        }

        WayPointResources.Storage.ArrayCount++;

        if (WayPointResources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          WayPointResources.Storage.ArrayCount = 0;
        }
      }

      WayPointResources.Storage.ArrayCount = 0;

      for (int16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
      {
        if (AddressCount < GPS_HOLD_TIMED_ADDR_TO_COMPARE)
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointResources.Mission.OthersParams.PositionHoldTime[WayPointResources.Storage.ArrayCount]);
        }
        else if (AddressCount >= GPS_HOLD_TIMED_ADDR_TO_COMPARE && AddressCount < (FLIGHT_MODE_ADDR_TO_COMPARE - 1))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Storage.ArrayCount]);
        }
        else if (AddressCount >= FLIGHT_MODE_ADDR_TO_COMPARE && AddressCount < (ALTITUDE_ADDR_TO_COMPARE - 1))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointResources.Mission.OthersParams.Altitude[WayPointResources.Storage.ArrayCount]);
        }

        WayPointResources.Storage.ArrayCount++;

        if (WayPointResources.Storage.ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          WayPointResources.Storage.ArrayCount = 0;
        }
      }
    }
    WayPointResources.Storage.Function = WAYPOINT_STORAGE_NONE;
  }
}

static bool WayPointSync10Hz(void)
{
  static Scheduler_Struct WayPointSyncTimer;
  return (Scheduler(&WayPointSyncTimer, SCHEDULER_SET_FREQUENCY(10, "Hz")));
}

static void SetWayPointAutoTakeOffState(uint8_t _AutoTakeOffState)
{
  if (_AutoTakeOffState == WAYPOINT_ENABLE_AUTO_TAKEOFF)
  {
    WayPointResources.AutoTakeOff.Flags.State = true;
  }
  if (_AutoTakeOffState == WAYPOINT_DISABLE_AUTO_TAKEOFF)
  {
    WayPointResources.AutoTakeOff.Flags.State = false;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_TAKEOFF)
  {
    WayPointResources.AutoTakeOff.Flags.Normalized = true;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_RESET)
  {
    WayPointResources.AutoTakeOff.Flags.Normalized = false;
  }
}

static bool GetWayPointAutoTakeOffState(void)
{
  return WayPointResources.AutoTakeOff.Flags.State;
}

static bool GetWayPointAutoTakeOffNormalized(void)
{
  return WayPointResources.AutoTakeOff.Flags.Normalized;
}

static void WayPointAutoTakeOffUpdate(void)
{
  if (!GetWayPointAutoTakeOffState())
  {
    return;
  }

  if (GetMultirotorEnabled())
  {
    if (WayPointSync10Hz())
    {
      if (WayPointResources.AutoTakeOff.Throttle.Increment < THROTTLE_TAKEOFF_ASCENT)
      {
        if (WayPointResources.AutoTakeOff.Throttle.IncrementCount >= THROTTLE_INCREMENT_TIME)
        {
          WayPointResources.AutoTakeOff.Throttle.Increment += THROTTLE_INCREMENT;
          WayPointResources.AutoTakeOff.Throttle.IncrementCount = 0;
        }
        else
        {
          WayPointResources.AutoTakeOff.Throttle.IncrementCount++;
        }
      }
      if (GetWayPointAutoTakeOffNormalized())
      {
        WayPointResources.AutoTakeOff.Throttle.Increment = THROTTLE_TAKEOFF_NORMALIZE;
      }
    }
    WayPointResources.AutoTakeOff.Throttle.Increment = Constrain_16Bits(WayPointResources.AutoTakeOff.Throttle.Increment, AttitudeThrottleMin, AttitudeThrottleMax);
    DECODE.SetRxChannelInput(THROTTLE, WayPointResources.AutoTakeOff.Throttle.Increment);
    RCController[THROTTLE] = WayPointResources.AutoTakeOff.Throttle.Increment;
  }
  else if (GetAirPlaneEnabled())
  {
    ENABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
  }
}

static void ResetAutoTakeOff(void)
{
  WayPointResources.AutoTakeOff.Throttle.Increment = 1000;
  WayPointResources.AutoTakeOff.Throttle.IncrementCount = 0;
  SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
  SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_RESET);
}

static void WayPointPredictPositionAndSetAltitude(void)
{
  if (!WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_ALT])
  {
    GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
    Do_Pos_Hold_Call_Alt_Hold = true;
    SetNewAltitudeToHold(ConverMetersToCM(WayPointResources.Mission.OthersParams.Altitude[WayPointResources.Mission.OthersParams.Number]));
    MultirotorSetThisPointToPositionHold();
    WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_ALT] = true;
  }
}

static void ResetWayPointNavigation(void)
{
  WayPointResources.Mission.OthersParams.Mode = WAYPOINT_INIT;
  WayPointResources.Mission.Flags.Reached = false;
  WayPointResources.Mission.OthersParams.Number = 0;
  WayPointResources.Mission.OthersParams.PositionHoldTimeToCompare = 0;
  WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_ALT] = false;
  WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT] = false;
  if (!WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_ALT])
  {
    GPS_Resources.Mode.Navigation = DO_NONE;
    Do_Pos_Hold_Call_Alt_Hold = false;
    WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_ALT] = true;
  }
}

void WayPointClass::Update(void)
{
  Push_WayPoint_Parameters();
  Store_And_Clear_WayPoints();

  if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE))
  {
    ResetWayPointNavigation();
    ResetAutoTakeOff();
    return;
  }

  if (WayPointResources.Mission.Coordinates.Latitude[0] == 0 || WayPointResources.Mission.Coordinates.Longitude[0] == 0)
  {
    return;
  }

  int16_t Navigation_Speed_Result = 0;

  WayPointAutoTakeOffUpdate();

  WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_RESET_POS_ALT] = false;

  switch (WayPointResources.Mission.OthersParams.Mode)
  {

  case WAYPOINT_INIT:
    for (uint8_t IndexCount = 0; IndexCount < WAYPOINTS_MAXIMUM; IndexCount++)
    {
      if (WayPointResources.Mission.OthersParams.FlightMode[IndexCount] == WAYPOINT_TAKEOFF)
      {
        WayPointResources.Mission.OthersParams.Mode = WAYPOINT_RUN_TAKEOFF;
        return;
      }
      else
      {
        WayPointResources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
      }
    }
    break;

  case WAYPOINT_RUN_TAKEOFF:
    WayPointPredictPositionAndSetAltitude();
    if (GetAltitudeReached())
    {
      if (GetMultirotorEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_TAKEOFF);
      }
      else if (GetAirPlaneEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
        DISABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
      }
      WayPointResources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
    }
    else
    {
      if (GetMultirotorEnabled())
      {
        if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
        }
        else
        {
          STICKS.PreArm_Run = true;
        }
      }
      else if (GetAirPlaneEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_ENABLE_AUTO_TAKEOFF);
      }
    }
    break;

  case WAYPOINT_SET_ALTITUDE:
    WayPointPredictPositionAndSetAltitude();
    if (GetAltitudeReached())
    {
      WayPointResources.Mission.OthersParams.Mode = WAYPOINT_START_MISSION;
    }
    break;

  case WAYPOINT_START_MISSION:
    WayPointResources.Mission.OthersParams.PositionHoldTimeToCompare = 0;
    WayPointResources.Mission.Flags.Reached = true;
    WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_PREDICT_POS_ALT] = false;
    WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT] = false;
    Set_Next_Point_To_Navigation(WayPointResources.Mission.Coordinates.Latitude[WayPointResources.Mission.OthersParams.Number], WayPointResources.Mission.Coordinates.Longitude[WayPointResources.Mission.OthersParams.Number]);
    WayPointResources.Mission.OthersParams.Mode = WAYPOINT_MISSION_ENROUTE;
    break;

  case WAYPOINT_MISSION_ENROUTE:
    Navigation_Speed_Result = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
    GPSCalculateNavigationRate(Navigation_Speed_Result);
    GPS_Resources.Navigation.HeadingHoldTarget = WRap_18000(GPS_Resources.Navigation.Bearing.ActualTarget) / 100;
    if ((GPS_Resources.Navigation.Coordinates.Distance <= ConverMetersToCM(JCF_Param.GPS_WP_Radius)) || Point_Reached())
    {
      for (uint8_t MissionCount = 0; MissionCount < WAYPOINTS_MAXIMUM; MissionCount++)
      {
        if (WayPointResources.Mission.Flags.Reached &&
            WayPointResources.Mission.OthersParams.Number == MissionCount &&
            WayPointResources.Mission.Coordinates.Latitude[MissionCount + 1] != 0 &&
            WayPointResources.Mission.Coordinates.Longitude[MissionCount + 1] != 0)
        {
          WayPointResources.Mission.OthersParams.Number = MissionCount + 1;
          WayPointResources.Mission.Flags.Reached = false;
          return;
        }
      }
      //DESATIVA O TAKEOFF SE A MISSÃO NÃO ESTIVER CONFIGURADA PARA O MESMO E SE O THROTTLE ESTIVER ACIMA DE UM CERTO NIVEL
      if (WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Mission.OthersParams.Number] != WAYPOINT_TAKEOFF && Throttle.Output >= THROTTLE_CANCEL_TAKEOFF && GetMultirotorEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
      }
      //AVANÇA O WAYPOINT
      if (WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Mission.OthersParams.Number] == WAYPOINT_ADVANCE)
      {
        WayPointResources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
        Do_RTH_Or_Land_Call_Alt_Hold = false;
        Do_Pos_Hold_Call_Alt_Hold = false;
      }
      //GPS-HOLD TIMERIZADO
      if (WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Mission.OthersParams.Number] == WAYPOINT_TIMED)
      {
        if (WayPointSync10Hz())
        {
          WayPointResources.Mission.OthersParams.PositionHoldTimeToCompare++; //10 ITERAÇÕES = 1 SEGUNDO
        }
        if (!WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPS_Resources.Mode.Navigation = DO_POSITION_HOLD;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          Do_Pos_Hold_Call_Alt_Hold = false;
          MultirotorSetThisPointToPositionHold();
          WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
        if (WayPointResources.Mission.OthersParams.PositionHoldTimeToCompare >= ConvertDegreesToDecidegrees(WayPointResources.Mission.OthersParams.PositionHoldTime[WayPointResources.Mission.OthersParams.Number]))
        {
          WayPointResources.Mission.OthersParams.Mode = WAYPOINT_SET_ALTITUDE;
        }
      }
      //LAND
      if (WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Mission.OthersParams.Number] == WAYPOINT_LAND)
      {
        if (!WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPS_Resources.Mode.Navigation = DO_LAND_INIT;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          MultirotorSetThisPointToPositionHold();
          WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
      }
      //RTH
      if (WayPointResources.Mission.OthersParams.FlightMode[WayPointResources.Mission.OthersParams.Number] == WAYPOINT_RTH)
      {
        if (!WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPS_Resources.Mode.Navigation = DO_START_RTH;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          Do_Mode_RTH_Now();
          WayPointResources.Mission.Flags.OnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
      }
    }
    break;
  }
}