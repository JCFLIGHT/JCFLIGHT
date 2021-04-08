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

struct _GetWayPointGCSParameters GetWayPointGCSParameters;
struct _GetWayPointGCSParametersTwo GetWayPointGCSParametersTwo;

#define THROTTLE_TAKEOFF_ASCENT 1600    //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF ATÉ CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_TAKEOFF_NORMALIZE 1500 //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF AO CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_CANCEL_TAKEOFF 1450    //VALOR DO THROTTLE LIDO DO RECEPTOR PARA CANCELAR O AUTO-TAKEOFF E VOLTAR AO CONTROLE NORMAL
#define THROTTLE_INCREMENT 100          //NÚMERO DE INCREMENTAÇÕES A CADA ESTOURO DE TEMPO DEFINIDO PELO PARAMETRO THROTTLE_INCREMENT_TIME
#define THROTTLE_INCREMENT_TIME 1       //INCREMENTA A CADA 0.10 SEGUNDOS

bool WPTakeOffNomalized = false;
bool Do_WayPoint_Call_Alt_Hold = false;
bool WPSucess = false;
bool ClearEEPROM = false;
bool StoreEEPROM = false;
uint8_t WayPointFlightMode[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t WayPointAltitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t WayPointTimed[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t ThrottleIncrementCount = 0;
uint8_t WayPointMode = 0;
uint8_t MissionNumber = 0;
uint8_t EEPROM_Function = 0;
uint16_t ThrottleIncrement = 1000;
int32_t WayPointLatitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t WayPointLongitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t Mission_Timed_Count = 0;

void WayPoint_Initialization()
{
  //CARREGA TODAS AS LATITUDES
  WayPointLatitude[0] = STORAGEMANAGER.Read_32Bits(704);
  WayPointLatitude[1] = STORAGEMANAGER.Read_32Bits(708);
  WayPointLatitude[2] = STORAGEMANAGER.Read_32Bits(712);
  WayPointLatitude[3] = STORAGEMANAGER.Read_32Bits(716);
  WayPointLatitude[4] = STORAGEMANAGER.Read_32Bits(720);
  WayPointLatitude[5] = STORAGEMANAGER.Read_32Bits(724);
  WayPointLatitude[6] = STORAGEMANAGER.Read_32Bits(728);
  WayPointLatitude[7] = STORAGEMANAGER.Read_32Bits(732);
  WayPointLatitude[8] = STORAGEMANAGER.Read_32Bits(736);
  WayPointLatitude[9] = STORAGEMANAGER.Read_32Bits(740);
  //CARREGA TODAS AS LONGITUDES
  WayPointLongitude[0] = STORAGEMANAGER.Read_32Bits(744);
  WayPointLongitude[1] = STORAGEMANAGER.Read_32Bits(748);
  WayPointLongitude[2] = STORAGEMANAGER.Read_32Bits(752);
  WayPointLongitude[3] = STORAGEMANAGER.Read_32Bits(756);
  WayPointLongitude[4] = STORAGEMANAGER.Read_32Bits(760);
  WayPointLongitude[5] = STORAGEMANAGER.Read_32Bits(764);
  WayPointLongitude[6] = STORAGEMANAGER.Read_32Bits(768);
  WayPointLongitude[7] = STORAGEMANAGER.Read_32Bits(772);
  WayPointLongitude[8] = STORAGEMANAGER.Read_32Bits(776);
  WayPointLongitude[9] = STORAGEMANAGER.Read_32Bits(780);
  //CARREGA O TIMER DAS MISSÕES COM GPS-HOLD
  WayPointTimed[0] = STORAGEMANAGER.Read_8Bits(784);
  WayPointTimed[1] = STORAGEMANAGER.Read_8Bits(785);
  WayPointTimed[2] = STORAGEMANAGER.Read_8Bits(786);
  WayPointTimed[3] = STORAGEMANAGER.Read_8Bits(787);
  WayPointTimed[4] = STORAGEMANAGER.Read_8Bits(788);
  WayPointTimed[5] = STORAGEMANAGER.Read_8Bits(789);
  WayPointTimed[6] = STORAGEMANAGER.Read_8Bits(790);
  WayPointTimed[7] = STORAGEMANAGER.Read_8Bits(791);
  WayPointTimed[8] = STORAGEMANAGER.Read_8Bits(792);
  WayPointTimed[9] = STORAGEMANAGER.Read_8Bits(793);
  //CARREGA O MODO DE VOO DAS MISSÕES
  WayPointFlightMode[0] = STORAGEMANAGER.Read_8Bits(794);
  WayPointFlightMode[1] = STORAGEMANAGER.Read_8Bits(795);
  WayPointFlightMode[2] = STORAGEMANAGER.Read_8Bits(796);
  WayPointFlightMode[3] = STORAGEMANAGER.Read_8Bits(797);
  WayPointFlightMode[4] = STORAGEMANAGER.Read_8Bits(798);
  WayPointFlightMode[5] = STORAGEMANAGER.Read_8Bits(799);
  WayPointFlightMode[6] = STORAGEMANAGER.Read_8Bits(800);
  WayPointFlightMode[7] = STORAGEMANAGER.Read_8Bits(801);
  WayPointFlightMode[8] = STORAGEMANAGER.Read_8Bits(802);
  WayPointFlightMode[9] = STORAGEMANAGER.Read_8Bits(803);
  //CARREGA A ALTITUDE DAS MISSÕES
  WayPointAltitude[0] = STORAGEMANAGER.Read_8Bits(804);
  WayPointAltitude[1] = STORAGEMANAGER.Read_8Bits(805);
  WayPointAltitude[2] = STORAGEMANAGER.Read_8Bits(806);
  WayPointAltitude[3] = STORAGEMANAGER.Read_8Bits(807);
  WayPointAltitude[4] = STORAGEMANAGER.Read_8Bits(808);
  WayPointAltitude[5] = STORAGEMANAGER.Read_8Bits(809);
  WayPointAltitude[6] = STORAGEMANAGER.Read_8Bits(810);
  WayPointAltitude[7] = STORAGEMANAGER.Read_8Bits(811);
  WayPointAltitude[8] = STORAGEMANAGER.Read_8Bits(812);
  WayPointAltitude[9] = STORAGEMANAGER.Read_8Bits(813);
}

void PushWayPointParameters()
{
  //NÃO VAMOS ZERAR AS VARIAVEIS,TALVEZ CONTÉM ALGO NA EEPROM
  if (GetWayPointGCSParameters.LatitudeOne == 0 || GetWayPointGCSParameters.LongitudeOne == 0)
  {
    return;
  }

  //OBTÉM TODAS AS LATITUDES DE CADA WAYPOINT
  WayPointLatitude[0] = GetWayPointGCSParameters.LatitudeOne;
  WayPointLatitude[1] = GetWayPointGCSParameters.LatitudeTwo;
  WayPointLatitude[2] = GetWayPointGCSParameters.LatitudeThree;
  WayPointLatitude[3] = GetWayPointGCSParameters.LatitudeFour;
  WayPointLatitude[4] = GetWayPointGCSParameters.LatitudeFive;
  WayPointLatitude[5] = GetWayPointGCSParametersTwo.LatitudeSix;
  WayPointLatitude[6] = GetWayPointGCSParametersTwo.LatitudeSeven;
  WayPointLatitude[7] = GetWayPointGCSParametersTwo.LatitudeEight;
  WayPointLatitude[8] = GetWayPointGCSParametersTwo.LatitudeNine;
  WayPointLatitude[9] = GetWayPointGCSParametersTwo.LatitudeTen;

  //OBTÉM TODAS AS LONGITUDES DE CADA WAYPOINT
  WayPointLongitude[0] = GetWayPointGCSParameters.LongitudeOne;
  WayPointLongitude[1] = GetWayPointGCSParameters.LongitudeTwo;
  WayPointLongitude[2] = GetWayPointGCSParameters.LongitudeThree;
  WayPointLongitude[3] = GetWayPointGCSParameters.LongitudeFour;
  WayPointLongitude[4] = GetWayPointGCSParameters.LongitudeFive;
  WayPointLongitude[5] = GetWayPointGCSParametersTwo.LongitudeSix;
  WayPointLongitude[6] = GetWayPointGCSParametersTwo.LongitudeSeven;
  WayPointLongitude[7] = GetWayPointGCSParametersTwo.LongitudeEight;
  WayPointLongitude[8] = GetWayPointGCSParametersTwo.LongitudeNine;
  WayPointLongitude[9] = GetWayPointGCSParametersTwo.LongitudeTen;

  //OBTÉM A ALTITUDE DE SUBIDA DE CADA WAYPOINT
  WayPointAltitude[0] = GetWayPointGCSParameters.AltitudeOne;
  WayPointAltitude[1] = GetWayPointGCSParameters.AltitudeTwo;
  WayPointAltitude[2] = GetWayPointGCSParameters.AltitudeThree;
  WayPointAltitude[3] = GetWayPointGCSParameters.AltitudeFour;
  WayPointAltitude[4] = GetWayPointGCSParameters.AltitudeFive;
  WayPointAltitude[5] = GetWayPointGCSParametersTwo.AltitudeSix;
  WayPointAltitude[6] = GetWayPointGCSParametersTwo.AltitudeSeven;
  WayPointAltitude[7] = GetWayPointGCSParametersTwo.AltitudeEight;
  WayPointAltitude[8] = GetWayPointGCSParametersTwo.AltitudeNine;
  WayPointAltitude[9] = GetWayPointGCSParametersTwo.AltitudeTen;

  //OBTÉM OS MODOS DE VOO DE CADA WAYPOINT
  WayPointFlightMode[0] = GetWayPointGCSParameters.FlightModeOne;
  WayPointFlightMode[1] = GetWayPointGCSParameters.FlightModeTwo;
  WayPointFlightMode[2] = GetWayPointGCSParameters.FlightModeThree;
  WayPointFlightMode[3] = GetWayPointGCSParameters.FlightModeFour;
  WayPointFlightMode[4] = GetWayPointGCSParameters.FlightModeFive;
  WayPointFlightMode[5] = GetWayPointGCSParametersTwo.FlightModeSix;
  WayPointFlightMode[6] = GetWayPointGCSParametersTwo.FlightModeSeven;
  WayPointFlightMode[7] = GetWayPointGCSParametersTwo.FlightModeEight;
  WayPointFlightMode[8] = GetWayPointGCSParametersTwo.FlightModeNine;
  WayPointFlightMode[9] = GetWayPointGCSParametersTwo.FlightModeTen;

  //OBTÉM O TEMPO DE VOO DO GPS-HOLD DE CADA WP
  WayPointTimed[0] = GetWayPointGCSParameters.GPSHoldTimedOne;
  WayPointTimed[1] = GetWayPointGCSParameters.GPSHoldTimedTwo;
  WayPointTimed[2] = GetWayPointGCSParameters.GPSHoldTimedThree;
  WayPointTimed[3] = GetWayPointGCSParameters.GPSHoldTimedFour;
  WayPointTimed[4] = GetWayPointGCSParameters.GPSHoldTimedFive;
  WayPointTimed[5] = GetWayPointGCSParametersTwo.GPSHoldTimedSix;
  WayPointTimed[6] = GetWayPointGCSParametersTwo.GPSHoldTimedSeven;
  WayPointTimed[7] = GetWayPointGCSParametersTwo.GPSHoldTimedEight;
  WayPointTimed[8] = GetWayPointGCSParametersTwo.GPSHoldTimedNine;
  WayPointTimed[9] = GetWayPointGCSParametersTwo.GPSHoldTimedTen;
}

bool WayPointSync10Hz()
{
  static Scheduler_Struct WayPointSyncTimer;
  if (Scheduler(&WayPointSyncTimer, SCHEDULER_SET_FREQUENCY(10, "Hz")))
  {
    return true;
  }
  return false;
}

void MulticopterAutoTakeOff(bool MulticopterAutoTakeOff)
{
  if (!MulticopterAutoTakeOff)
  {
    return;
  }
  if (WayPointSync10Hz())
  {
    if (ThrottleIncrement < THROTTLE_TAKEOFF_ASCENT && !WPTakeOffNomalized)
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
    else
    {
      WPTakeOffNomalized = true;
      ThrottleIncrement = THROTTLE_TAKEOFF_NORMALIZE;
    }
  }
  DECODE.SetRxChannelInput(THROTTLE, ThrottleIncrement);
  RCController[THROTTLE] = Constrain_16Bits(ThrottleIncrement, AttitudeThrottleMin, AttitudeThrottleMax);
}

void WayPointRun()
{
  int16_t Navigation_Speed_Result = 0;

  Store_And_Clear_WayPoints();

  if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE))
  {
    WayPointMode = WP_MISSION_INIT;
    WPSucess = false;
    MissionNumber = 0;
    Navigation_Speed_Result = 0;
    Mission_Timed_Count = 0;
    ThrottleIncrement = 1000;
    ThrottleIncrementCount = 0;
    Do_WayPoint_Call_Alt_Hold = false;
    WPTakeOffNomalized = false;
    return;
  }

  if (WayPointLatitude[0] == 0 || WayPointLongitude[0] == 0)
  {
    return;
  }

  switch (WayPointMode)
  {

  case WP_MISSION_INIT:
    //ATIVA O MODO ALTITUDE-HOLD
    Do_WayPoint_Call_Alt_Hold = true;
    //TAKEOFF
    if (WayPointFlightMode[0] == WP_TAKEOFF || WayPointFlightMode[1] == WP_TAKEOFF || WayPointFlightMode[2] == WP_TAKEOFF ||
        WayPointFlightMode[3] == WP_TAKEOFF || WayPointFlightMode[4] == WP_TAKEOFF || WayPointFlightMode[5] == WP_TAKEOFF ||
        WayPointFlightMode[6] == WP_TAKEOFF || WayPointFlightMode[7] == WP_TAKEOFF || WayPointFlightMode[8] == WP_TAKEOFF ||
        WayPointFlightMode[9] == WP_TAKEOFF)
    {
      WayPointMode = GET_ALTITUDE_TAKEOFF;
    }
    else
    {
      WayPointMode = GET_ALTITUDE;
    }
    break;

  case GET_ALTITUDE_TAKEOFF:
    Mission_Timed_Count = 0;
    Get_Altitude();
    GPS_Flight_Mode = GPS_MODE_HOLD;
    SetThisPointToPositionHold();
    GPS_Navigation_Mode = DO_POSITION_HOLD;
    if (GetAltitudeReached() && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      if (ThrottleIncrement >= THROTTLE_TAKEOFF_ASCENT)
      {
        STICKS.PreArm_Run = false;
        WayPointMode = WP_START_MISSION;
      }
    }
    else
    {
      if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
      {
        MulticopterAutoTakeOff(true);
      }
      else
      {
        STICKS.PreArm_Run = true;
      }
    }
    break;

  case GET_ALTITUDE:
    Mission_Timed_Count = 0;
    Get_Altitude();
    GPS_Flight_Mode = GPS_MODE_HOLD;
    SetThisPointToPositionHold();
    GPS_Navigation_Mode = DO_POSITION_HOLD;
    if (GetAltitudeReached())
    {
      WayPointMode = WP_START_MISSION;
    }
    break;

  case WP_START_MISSION:
    Set_Next_Point_To_Navigation(&WayPointLatitude[MissionNumber], &WayPointLongitude[MissionNumber]);
    WPSucess = true;
    WayPointMode = WP_EN_ROUTE;
    break;

  case WP_EN_ROUTE:
    Navigation_Speed_Result = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
    GPSCalculateNavigationRate(Navigation_Speed_Result);
    HeadingHoldTarget = WRap_18000(Target_Bearing) / 100;
    if ((Two_Points_Distance <= (JCF_Param.GPS_WP_Radius * 100)) || Point_Reached())
    {
      if (WPSucess && MissionNumber == 0 && WayPointLatitude[1] != 0 && WayPointLongitude[1] != 0)
      {
        MissionNumber = 1;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 1 && WayPointLatitude[2] != 0 && WayPointLongitude[2] != 0)
      {
        MissionNumber = 2;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 2 && WayPointLatitude[3] != 0 && WayPointLongitude[3] != 0)
      {
        MissionNumber = 3;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 3 && WayPointLatitude[4] != 0 && WayPointLongitude[4] != 0)
      {
        MissionNumber = 4;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 4 && WayPointLatitude[5] != 0 && WayPointLongitude[5] != 0)
      {
        MissionNumber = 5;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 5 && WayPointLatitude[6] != 0 && WayPointLongitude[6] != 0)
      {
        MissionNumber = 6;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 6 && WayPointLatitude[7] != 0 && WayPointLongitude[7] != 0)
      {
        MissionNumber = 7;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 7 && WayPointLatitude[8] != 0 && WayPointLongitude[8] != 0)
      {
        MissionNumber = 8;
        WPSucess = false;
      }
      else if (WPSucess && MissionNumber == 8 && WayPointLatitude[9] != 0 && WayPointLongitude[9] != 0)
      {
        MissionNumber = 9;
        WPSucess = false;
      }
      //DESATIVA O TAKEOFF SE A MISSÃO NÃO ESTIVER CONFIGURADA PARA O MESMO E SE O THROTTLE ESTIVER ACIMA DE UM CERTO NIVEL
      if (WayPointFlightMode[MissionNumber] != WP_TAKEOFF && Throttle.Output >= THROTTLE_CANCEL_TAKEOFF)
      {
        MulticopterAutoTakeOff(false);
      }
      //AVANÇA O WAYPOINT
      if (WayPointFlightMode[MissionNumber] == WP_ADVANCE)
      {
        WayPointMode = GET_ALTITUDE;
      }
      //GPS-HOLD TIMERIZADO
      if (WayPointFlightMode[MissionNumber] == WP_TIMED)
      {
        if (WayPointSync10Hz())
        {
          Mission_Timed_Count++; //10 ITERAÇÕES = 1 SEGUNDO
        }
        Do_RTH_Or_Land_Call_Alt_Hold = false;
        GPS_Flight_Mode = GPS_MODE_HOLD;
        SetThisPointToPositionHold();
        GPS_Navigation_Mode = DO_POSITION_HOLD;
        if (Mission_Timed_Count >= ConvertDegreesToDecidegrees(WayPointTimed[MissionNumber])) //MULT POR 10 PARA OBTÉR O VALOR EM SEGUNDOS PARA TRABALHAR EM CONJUNTO COM A FUNÇÃO WayPointSync10Hz()
        {
          WayPointMode = GET_ALTITUDE;
        }
      }
      //LAND
      if (WayPointFlightMode[MissionNumber] == WP_LAND)
      {
        GPS_Flight_Mode = GPS_MODE_HOLD;
        SetThisPointToPositionHold();
        GPS_Navigation_Mode = DO_POSITION_HOLD;
        GPS_Navigation_Mode = DO_LAND_INIT;
      }
      //RTH
      if (WayPointFlightMode[MissionNumber] == WP_RTH)
      {
        Do_Mode_RTH_Now();
        ENABLE_THIS_FLIGHT_MODE(HEADING_HOLD_MODE);
      }
    }
    break;
  }
}

void Get_Altitude()
{
  static uint8_t MissionNumberCount;
  static uint8_t AltitudeSum;

  if (MissionNumberCount != MissionNumber)
  {
    if (MissionNumberCount < MissionNumber)
    {
      MissionNumberCount++;
      AltitudeSum += 5;
    }
    else
    {
      MissionNumberCount--;
      AltitudeSum -= 5;
    }
  }

  SetAltitudeToHold(ConvertCMToMeters(10 + AltitudeSum));
}

void Store_And_Clear_WayPoints()
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
      GetWayPointGCSParameters.Reset();
      GetWayPointGCSParametersTwo.Reset();
    }
    if (!ClearEEPROM)
    {
      STORAGEMANAGER.Erase(704, 813);
      EEPROM_Function = 0;
      ClearEEPROM = true;
    }
  }
  else
  {
    ClearEEPROM = false;
  }

  if (EEPROM_Function == 2) //SALVA NA EEPROM
  {
    if (!StoreEEPROM)
    {
      //SALVA TODAS AS LATITUDES
      STORAGEMANAGER.Write_32Bits(704, WayPointLatitude[0]);
      STORAGEMANAGER.Write_32Bits(708, WayPointLatitude[1]);
      STORAGEMANAGER.Write_32Bits(712, WayPointLatitude[2]);
      STORAGEMANAGER.Write_32Bits(716, WayPointLatitude[3]);
      STORAGEMANAGER.Write_32Bits(720, WayPointLatitude[4]);
      STORAGEMANAGER.Write_32Bits(724, WayPointLatitude[5]);
      STORAGEMANAGER.Write_32Bits(728, WayPointLatitude[6]);
      STORAGEMANAGER.Write_32Bits(732, WayPointLatitude[7]);
      STORAGEMANAGER.Write_32Bits(736, WayPointLatitude[8]);
      STORAGEMANAGER.Write_32Bits(740, WayPointLatitude[9]);
      //SALVA TODAS AS LONGITUDES
      STORAGEMANAGER.Write_32Bits(744, WayPointLongitude[0]);
      STORAGEMANAGER.Write_32Bits(748, WayPointLongitude[1]);
      STORAGEMANAGER.Write_32Bits(752, WayPointLongitude[2]);
      STORAGEMANAGER.Write_32Bits(756, WayPointLongitude[3]);
      STORAGEMANAGER.Write_32Bits(760, WayPointLongitude[4]);
      STORAGEMANAGER.Write_32Bits(764, WayPointLongitude[5]);
      STORAGEMANAGER.Write_32Bits(768, WayPointLongitude[6]);
      STORAGEMANAGER.Write_32Bits(772, WayPointLongitude[7]);
      STORAGEMANAGER.Write_32Bits(776, WayPointLongitude[8]);
      STORAGEMANAGER.Write_32Bits(780, WayPointLongitude[9]);
      //SALVA O TIMER DAS MISSÕES COM GPS-HOLD
      STORAGEMANAGER.Write_8Bits(784, WayPointTimed[0]);
      STORAGEMANAGER.Write_8Bits(785, WayPointTimed[1]);
      STORAGEMANAGER.Write_8Bits(786, WayPointTimed[2]);
      STORAGEMANAGER.Write_8Bits(787, WayPointTimed[3]);
      STORAGEMANAGER.Write_8Bits(788, WayPointTimed[4]);
      STORAGEMANAGER.Write_8Bits(789, WayPointTimed[5]);
      STORAGEMANAGER.Write_8Bits(790, WayPointTimed[6]);
      STORAGEMANAGER.Write_8Bits(791, WayPointTimed[7]);
      STORAGEMANAGER.Write_8Bits(792, WayPointTimed[8]);
      STORAGEMANAGER.Write_8Bits(793, WayPointTimed[9]);
      //SALVA O MODO DE VOO DAS MISSÕES
      STORAGEMANAGER.Write_8Bits(794, WayPointFlightMode[0]);
      STORAGEMANAGER.Write_8Bits(795, WayPointFlightMode[1]);
      STORAGEMANAGER.Write_8Bits(796, WayPointFlightMode[2]);
      STORAGEMANAGER.Write_8Bits(797, WayPointFlightMode[3]);
      STORAGEMANAGER.Write_8Bits(798, WayPointFlightMode[4]);
      STORAGEMANAGER.Write_8Bits(799, WayPointFlightMode[5]);
      STORAGEMANAGER.Write_8Bits(800, WayPointFlightMode[6]);
      STORAGEMANAGER.Write_8Bits(801, WayPointFlightMode[7]);
      STORAGEMANAGER.Write_8Bits(802, WayPointFlightMode[8]);
      STORAGEMANAGER.Write_8Bits(803, WayPointFlightMode[9]);
      //SALVA A ALTITUDE DAS MISSÕES
      STORAGEMANAGER.Write_8Bits(804, WayPointAltitude[0]);
      STORAGEMANAGER.Write_8Bits(805, WayPointAltitude[1]);
      STORAGEMANAGER.Write_8Bits(806, WayPointAltitude[2]);
      STORAGEMANAGER.Write_8Bits(807, WayPointAltitude[3]);
      STORAGEMANAGER.Write_8Bits(808, WayPointAltitude[4]);
      STORAGEMANAGER.Write_8Bits(809, WayPointAltitude[5]);
      STORAGEMANAGER.Write_8Bits(810, WayPointAltitude[6]);
      STORAGEMANAGER.Write_8Bits(811, WayPointAltitude[7]);
      STORAGEMANAGER.Write_8Bits(812, WayPointAltitude[8]);
      STORAGEMANAGER.Write_8Bits(813, WayPointAltitude[9]);
      EEPROM_Function = 0;
      StoreEEPROM = true;
    }
  }
  else
  {
    StoreEEPROM = false;
  }
}
