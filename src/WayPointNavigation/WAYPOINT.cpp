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
#include "Common/STRUCTS.h"
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

struct _GetWayPointPacketOne GetWayPointPacketOne;
struct _GetWayPointPacketTwo GetWayPointPacketTwo;

#define THROTTLE_TAKEOFF_ASCENT 1600    //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF ATÉ CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_TAKEOFF_NORMALIZE 1500 //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF AO CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_CANCEL_TAKEOFF 1450    //VALOR DO THROTTLE LIDO DO RECEPTOR PARA CANCELAR O AUTO-TAKEOFF E VOLTAR AO CONTROLE NORMAL
#define THROTTLE_INCREMENT 100          //NÚMERO DE INCREMENTAÇÕES A CADA ESTOURO DE TEMPO DEFINIDO PELO PARAMETRO THROTTLE_INCREMENT_TIME
#define THROTTLE_INCREMENT_TIME 1       //INCREMENTA A CADA 0.10 SEGUNDOS

bool AutoTakeOffState = false;
bool MultirotorAutoTakeOffNormalized = false;
bool WayPointMissionReached = false;
bool WayPointOnceFlight[SIZE_OF_WAYPOINT_ONCE] = {false, false};
uint8_t WayPointFlightMode[WAYPOINTS_MAXIMUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t WayPointAltitude[WAYPOINTS_MAXIMUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t WayPointTimed[WAYPOINTS_MAXIMUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t ThrottleIncrementCount = 0;
uint8_t WayPointMode = 0;
uint8_t MissionNumber = 0;
uint8_t EEPROM_Function = 0;
int16_t ThrottleIncrement = 1000;
int32_t WayPointLatitude[WAYPOINTS_MAXIMUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t WayPointLongitude[WAYPOINTS_MAXIMUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t Mission_Timed_Count = 0;

void WayPointClass::Initialization(void)
{
  uint8_t ArrayCount = 0;

  for (uint16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
  {

    if (AddressCount < ((INITIAL_ADDR_OF_COORDINATES + FINAL_ADDR_OF_COORDINATES + 2) / 2))
    {
      WayPointLatitude[ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }
    else
    {
      WayPointLongitude[ArrayCount] = STORAGEMANAGER.Read_32Bits(AddressCount);
    }

    ArrayCount++;
    if (ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      ArrayCount = 0;
    }
  }

  ArrayCount = 0;

  for (uint16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
  {

    if (AddressCount < (INITIAL_ADDR_OF_OTHERS_PARAMS + WAYPOINTS_MAXIMUM - 1))
    {
      WayPointTimed[ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= (INITIAL_ADDR_OF_OTHERS_PARAMS + WAYPOINTS_MAXIMUM - 1) && AddressCount < ((WAYPOINTS_MAXIMUM * 2 - 1) + INITIAL_ADDR_OF_OTHERS_PARAMS))
    {
      WayPointFlightMode[ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }
    else if (AddressCount >= ((WAYPOINTS_MAXIMUM * 2) + INITIAL_ADDR_OF_OTHERS_PARAMS) && AddressCount < ((WAYPOINTS_MAXIMUM * 3 - 1) + INITIAL_ADDR_OF_OTHERS_PARAMS))
    {
      WayPointAltitude[ArrayCount] = STORAGEMANAGER.Read_8Bits(AddressCount);
    }

    ArrayCount++;
    if (ArrayCount >= WAYPOINTS_MAXIMUM)
    {
      ArrayCount = 0;
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
  WayPointLatitude[0] = GetWayPointPacketOne.LatitudeOne;
  WayPointLatitude[1] = GetWayPointPacketOne.LatitudeTwo;
  WayPointLatitude[2] = GetWayPointPacketOne.LatitudeThree;
  WayPointLatitude[3] = GetWayPointPacketOne.LatitudeFour;
  WayPointLatitude[4] = GetWayPointPacketOne.LatitudeFive;
  WayPointLatitude[5] = GetWayPointPacketTwo.LatitudeSix;
  WayPointLatitude[6] = GetWayPointPacketTwo.LatitudeSeven;
  WayPointLatitude[7] = GetWayPointPacketTwo.LatitudeEight;
  WayPointLatitude[8] = GetWayPointPacketTwo.LatitudeNine;
  WayPointLatitude[9] = GetWayPointPacketTwo.LatitudeTen;

  //OBTÉM TODAS AS LONGITUDES DE CADA WAYPOINT
  WayPointLongitude[0] = GetWayPointPacketOne.LongitudeOne;
  WayPointLongitude[1] = GetWayPointPacketOne.LongitudeTwo;
  WayPointLongitude[2] = GetWayPointPacketOne.LongitudeThree;
  WayPointLongitude[3] = GetWayPointPacketOne.LongitudeFour;
  WayPointLongitude[4] = GetWayPointPacketOne.LongitudeFive;
  WayPointLongitude[5] = GetWayPointPacketTwo.LongitudeSix;
  WayPointLongitude[6] = GetWayPointPacketTwo.LongitudeSeven;
  WayPointLongitude[7] = GetWayPointPacketTwo.LongitudeEight;
  WayPointLongitude[8] = GetWayPointPacketTwo.LongitudeNine;
  WayPointLongitude[9] = GetWayPointPacketTwo.LongitudeTen;

  //OBTÉM A ALTITUDE DE SUBIDA DE CADA WAYPOINT
  WayPointAltitude[0] = GetWayPointPacketOne.AltitudeOne;
  WayPointAltitude[1] = GetWayPointPacketOne.AltitudeTwo;
  WayPointAltitude[2] = GetWayPointPacketOne.AltitudeThree;
  WayPointAltitude[3] = GetWayPointPacketOne.AltitudeFour;
  WayPointAltitude[4] = GetWayPointPacketOne.AltitudeFive;
  WayPointAltitude[5] = GetWayPointPacketTwo.AltitudeSix;
  WayPointAltitude[6] = GetWayPointPacketTwo.AltitudeSeven;
  WayPointAltitude[7] = GetWayPointPacketTwo.AltitudeEight;
  WayPointAltitude[8] = GetWayPointPacketTwo.AltitudeNine;
  WayPointAltitude[9] = GetWayPointPacketTwo.AltitudeTen;

  //OBTÉM OS MODOS DE VOO DE CADA WAYPOINT
  WayPointFlightMode[0] = GetWayPointPacketOne.FlightModeOne;
  WayPointFlightMode[1] = GetWayPointPacketOne.FlightModeTwo;
  WayPointFlightMode[2] = GetWayPointPacketOne.FlightModeThree;
  WayPointFlightMode[3] = GetWayPointPacketOne.FlightModeFour;
  WayPointFlightMode[4] = GetWayPointPacketOne.FlightModeFive;
  WayPointFlightMode[5] = GetWayPointPacketTwo.FlightModeSix;
  WayPointFlightMode[6] = GetWayPointPacketTwo.FlightModeSeven;
  WayPointFlightMode[7] = GetWayPointPacketTwo.FlightModeEight;
  WayPointFlightMode[8] = GetWayPointPacketTwo.FlightModeNine;
  WayPointFlightMode[9] = GetWayPointPacketTwo.FlightModeTen;

  //OBTÉM O TEMPO DE VOO DO GPS-HOLD DE CADA WP
  WayPointTimed[0] = GetWayPointPacketOne.GPSHoldTimedOne;
  WayPointTimed[1] = GetWayPointPacketOne.GPSHoldTimedTwo;
  WayPointTimed[2] = GetWayPointPacketOne.GPSHoldTimedThree;
  WayPointTimed[3] = GetWayPointPacketOne.GPSHoldTimedFour;
  WayPointTimed[4] = GetWayPointPacketOne.GPSHoldTimedFive;
  WayPointTimed[5] = GetWayPointPacketTwo.GPSHoldTimedSix;
  WayPointTimed[6] = GetWayPointPacketTwo.GPSHoldTimedSeven;
  WayPointTimed[7] = GetWayPointPacketTwo.GPSHoldTimedEight;
  WayPointTimed[8] = GetWayPointPacketTwo.GPSHoldTimedNine;
  WayPointTimed[9] = GetWayPointPacketTwo.GPSHoldTimedTen;
}

void WayPointClass::Erase(void)
{
  //OS ÚLTIMOS ENDEREÇOS SÃO OS DE ARMAZENAMENTO DAS ALTITUDES DE SUBIDA DE CADA MISSÃO
  STORAGEMANAGER.Erase(INITIAL_ADDR_OF_COORDINATES, (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM));
  for (uint16_t AddressCount = (FINAL_ADDR_OF_OTHERS_PARAMS - WAYPOINTS_MAXIMUM + 1); AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount++)
  {
    //PADRÃO DE 10 METROS PARA AS ALTITUDES DE VOO POR WAYPOINT
    STORAGEMANAGER.Write_8Bits(AddressCount, 10);
  }
}

static void Store_And_Clear_WayPoints(void)
{
  if (EEPROM_Function == 1) //RESETA TUDO
  {
    for (uint8_t CountVector = 0; CountVector < 10; CountVector++)
    {
      WayPointLatitude[CountVector] = 0;
      WayPointLongitude[CountVector] = 0;
      WayPointFlightMode[CountVector] = 0;
      WayPointAltitude[CountVector] = 0;
      WayPointTimed[CountVector] = 0;
      GetWayPointPacketOne.Reset();
      GetWayPointPacketTwo.Reset();
      WAYPOINT.Erase();
    }
    EEPROM_Function = 0;
  }

  if (EEPROM_Function == 2) //SALVA NA EEPROM
  {
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {

      uint8_t ArrayCount = 0;

      for (uint16_t AddressCount = INITIAL_ADDR_OF_COORDINATES; AddressCount <= FINAL_ADDR_OF_COORDINATES; AddressCount += sizeof(int32_t))
      {

        if (AddressCount < ((INITIAL_ADDR_OF_COORDINATES + FINAL_ADDR_OF_COORDINATES + 2) / 2))
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPointLatitude[ArrayCount]);
        }
        else
        {
          STORAGEMANAGER.Write_32Bits(AddressCount, WayPointLongitude[ArrayCount]);
        }

        ArrayCount++;
        if (ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          ArrayCount = 0;
        }
      }

      ArrayCount = 0;

      for (uint16_t AddressCount = INITIAL_ADDR_OF_OTHERS_PARAMS; AddressCount <= FINAL_ADDR_OF_OTHERS_PARAMS; AddressCount += sizeof(uint8_t))
      {

        if (AddressCount < (INITIAL_ADDR_OF_OTHERS_PARAMS + WAYPOINTS_MAXIMUM - 1))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointTimed[ArrayCount]);
        }
        else if (AddressCount >= (INITIAL_ADDR_OF_OTHERS_PARAMS + WAYPOINTS_MAXIMUM - 1) && AddressCount < ((WAYPOINTS_MAXIMUM * 2 - 1) + INITIAL_ADDR_OF_OTHERS_PARAMS))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointFlightMode[ArrayCount]);
        }
        else if (AddressCount >= ((WAYPOINTS_MAXIMUM * 2) + INITIAL_ADDR_OF_OTHERS_PARAMS) && AddressCount < ((WAYPOINTS_MAXIMUM * 3 - 1) + INITIAL_ADDR_OF_OTHERS_PARAMS))
        {
          STORAGEMANAGER.Write_8Bits(AddressCount, WayPointAltitude[ArrayCount]);
        }

        ArrayCount++;
        if (ArrayCount >= WAYPOINTS_MAXIMUM)
        {
          ArrayCount = 0;
        }
      }
    }
    EEPROM_Function = 0;
  }
}

static bool WayPointSync10Hz()
{
  static Scheduler_Struct WayPointSyncTimer;
  return (Scheduler(&WayPointSyncTimer, SCHEDULER_SET_FREQUENCY(10, "Hz")));
}

static void SetWayPointAutoTakeOffState(uint8_t _AutoTakeOffState)
{
  if (_AutoTakeOffState == WAYPOINT_ENABLE_AUTO_TAKEOFF)
  {
    AutoTakeOffState = true;
  }
  if (_AutoTakeOffState == WAYPOINT_DISABLE_AUTO_TAKEOFF)
  {
    AutoTakeOffState = false;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_TAKEOFF)
  {
    MultirotorAutoTakeOffNormalized = true;
  }
  else if (_AutoTakeOffState == WAYPOINT_NORMALIZE_RESET)
  {
    MultirotorAutoTakeOffNormalized = false;
  }
}

static bool GetWayPointAutoTakeOffState(void)
{
  return AutoTakeOffState;
}

static bool GetWayPointAutoTakeOffNormalized(void)
{
  return MultirotorAutoTakeOffNormalized;
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
      if (ThrottleIncrement < THROTTLE_TAKEOFF_ASCENT)
      {
        if (ThrottleIncrementCount >= THROTTLE_INCREMENT_TIME)
        {
          ThrottleIncrement += THROTTLE_INCREMENT;
          ThrottleIncrementCount = 0;
        }
        else
        {
          ThrottleIncrementCount++;
        }
      }
      if (GetWayPointAutoTakeOffNormalized())
      {
        ThrottleIncrement = THROTTLE_TAKEOFF_NORMALIZE;
      }
    }
    DECODE.SetRxChannelInput(THROTTLE, ThrottleIncrement);
    RCController[THROTTLE] = Constrain_16Bits(ThrottleIncrement, AttitudeThrottleMin, AttitudeThrottleMax);
  }
  else if (GetAirPlaneEnabled())
  {
    ENABLE_THIS_FLIGHT_MODE(LAUNCH_MODE);
  }
}

static void ResetAutoTakeOff(void)
{
  ThrottleIncrement = 1000;
  ThrottleIncrementCount = 0;
  SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
  SetWayPointAutoTakeOffState(WAYPOINT_NORMALIZE_RESET);
}

static void WayPointPredictPositionAndSetAltitude(void)
{
  if (!WayPointOnceFlight[WAYPOINT_PREDICT_POS_ALT])
  {
    GPSParameters.Mode.Navigation = DO_POSITION_HOLD;
    Do_Pos_Hold_Call_Alt_Hold = true;
    SetNewAltitudeToHold(ConverMetersToCM(WayPointAltitude[MissionNumber]));
    SetThisPointToPositionHold();
    WayPointOnceFlight[WAYPOINT_PREDICT_POS_ALT] = true;
  }
}

void WayPointClass::Update()
{
  Push_WayPoint_Parameters();
  Store_And_Clear_WayPoints();

  if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE))
  {
    WayPointMode = WAYPOINT_INIT;
    WayPointMissionReached = false;
    MissionNumber = 0;
    Mission_Timed_Count = 0;
    WayPointOnceFlight[WAYPOINT_PREDICT_POS_ALT] = false;
    WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT] = false;
    ResetAutoTakeOff();
    return;
  }

  if (WayPointLatitude[0] == 0 || WayPointLongitude[0] == 0)
  {
    return;
  }

  int16_t Navigation_Speed_Result = 0;

  WayPointAutoTakeOffUpdate();

  switch (WayPointMode)
  {

  case WAYPOINT_INIT:
    for (uint8_t IndexCount = 0; IndexCount < WAYPOINTS_MAXIMUM; IndexCount++)
    {
      if (WayPointFlightMode[IndexCount] == WAYPOINT_TAKEOFF)
      {
        WayPointMode = WAYPOINT_RUN_TAKEOFF;
        return;
      }
      else
      {
        WayPointMode = WAYPOINT_SET_ALTITUDE;
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
      WayPointMode = WAYPOINT_START_MISSION;
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
      WayPointMode = WAYPOINT_START_MISSION;
    }
    break;

  case WAYPOINT_START_MISSION:
    Mission_Timed_Count = 0;
    WayPointOnceFlight[WAYPOINT_PREDICT_POS_ALT] = false;
    WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT] = false;
    Set_Next_Point_To_Navigation(WayPointLatitude[MissionNumber], WayPointLongitude[MissionNumber]);
    WayPointMissionReached = true;
    WayPointMode = WAYPOINT_MISSION_ENROUTE;
    break;

  case WAYPOINT_MISSION_ENROUTE:
    Navigation_Speed_Result = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
    GPSCalculateNavigationRate(Navigation_Speed_Result);
    GPSParameters.Navigation.HeadingHoldTarget = WRap_18000(GPSParameters.Navigation.Bearing.ActualTarget) / 100;
    if ((GPSParameters.Navigation.Coordinates.Distance <= ConverMetersToCM(JCF_Param.GPS_WP_Radius)) || Point_Reached())
    {
      for (uint8_t MissionCount = 0; MissionCount < WAYPOINTS_MAXIMUM; MissionCount++)
      {
        if (WayPointMissionReached &&
            MissionNumber == MissionCount &&
            WayPointLatitude[MissionCount + 1] != 0 &&
            WayPointLongitude[MissionCount + 1] != 0)
        {
          MissionNumber = MissionCount + 1;
          WayPointMissionReached = false;
          return;
        }
      }
      //DESATIVA O TAKEOFF SE A MISSÃO NÃO ESTIVER CONFIGURADA PARA O MESMO E SE O THROTTLE ESTIVER ACIMA DE UM CERTO NIVEL
      if (WayPointFlightMode[MissionNumber] != WAYPOINT_TAKEOFF && Throttle.Output >= THROTTLE_CANCEL_TAKEOFF && GetMultirotorEnabled())
      {
        SetWayPointAutoTakeOffState(WAYPOINT_DISABLE_AUTO_TAKEOFF);
      }
      //AVANÇA O WAYPOINT
      if (WayPointFlightMode[MissionNumber] == WAYPOINT_ADVANCE)
      {
        WayPointMode = WAYPOINT_SET_ALTITUDE;
        Do_RTH_Or_Land_Call_Alt_Hold = false;
        Do_Pos_Hold_Call_Alt_Hold = false;
      }
      //GPS-HOLD TIMERIZADO
      if (WayPointFlightMode[MissionNumber] == WAYPOINT_TIMED)
      {
        if (WayPointSync10Hz())
        {
          Mission_Timed_Count++; //10 ITERAÇÕES = 1 SEGUNDO
        }
        if (!WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPSParameters.Mode.Navigation = DO_POSITION_HOLD;
          Do_RTH_Or_Land_Call_Alt_Hold = false;
          Do_Pos_Hold_Call_Alt_Hold = false;
          SetThisPointToPositionHold();
          WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
        if (Mission_Timed_Count >= ConvertDegreesToDecidegrees(WayPointTimed[MissionNumber]))
        {
          WayPointMode = WAYPOINT_SET_ALTITUDE;
        }
      }
      //LAND
      if (WayPointFlightMode[MissionNumber] == WAYPOINT_LAND)
      {
        if (!WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPSParameters.Mode.Navigation = DO_LAND_INIT;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          SetThisPointToPositionHold();
          WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
      }
      //RTH
      if (WayPointFlightMode[MissionNumber] == WAYPOINT_RTH)
      {
        if (!WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT])
        {
          GPSParameters.Mode.Navigation = DO_START_RTH;
          Do_RTH_Or_Land_Call_Alt_Hold = true;
          Do_Pos_Hold_Call_Alt_Hold = false;
          ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
          Do_Mode_RTH_Now();
          WayPointOnceFlight[WAYPOINT_NORMAL_FLIGHT] = true;
        }
      }
    }
    break;
  }
}