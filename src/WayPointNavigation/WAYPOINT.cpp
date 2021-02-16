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
#include "Common/VARIABLES.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "RadioControl/RCCONFIG.h"
#include "Common/STRUCTS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/STICKS.h"
#include "Scheduler/SCHEDULER.h"

struct _GetWayPointGCSParameters GetWayPointGCSParameters;
struct _GetWayPointGCSParametersTwo GetWayPointGCSParametersTwo;

#define WP_NAVIGATION_SPEED 400         //VELOCIDADE DE NAVEGAÇÃO NO RTH E WAYPOINT (400 CM/SEGUNDO)
#define WAYPOINT_RADIUS 2               //RAIO PARA VERIFICAR SE A CONTROLADORA CHEGOU PERTO DO WP E INICIAR A IDA PARA O PROXIMO (RAIO EM METROS)
#define THROTTLE_TAKEOFF_ASCENT 1600    //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF ATÉ CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_TAKEOFF_NORMALIZE 1500 //VALOR DO THROTTLE AO FAZER O AUTO-TAKEOFF AO CHEGAR NA ALTITUDE SETADA PELO GCS
#define THROTTLE_CANCEL_TAKEOFF 1450    //VALOR DO THROTTLE LIDO DO RECEPTOR PARA CANCELAR O AUTO-TAKEOFF E VOLTAR AO CONTROLE NORMAL
#define THROTTLE_INCREMENT 100          //NÚMERO DE INCREMENTAÇÕES A CADA ESTOURO DE TEMPO DEFINIDO PELO PARAMETRO THROTTLE_INCREMENT_TIME
#define THROTTLE_INCREMENT_TIME 1       //INCREMENTA A CADA 0.10 SEGUNDOS

bool WPTakeOffNomalized = false;
bool Mission_BaroMode = false;
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

void WayPointRun()
{
  int16_t Navigation_Speed_Result = 0;

  Store_And_Clear_WayPoints();

  if (!SetFlightModes[WAYPOINT_MODE])
  {
    WayPointMode = WP_MISSION_INIT;
    WPSucess = false;
    MissionNumber = 0;
    Navigation_Speed_Result = 0;
    Mission_Timed_Count = 0;
    ThrottleIncrement = 1000;
    ThrottleIncrementCount = 0;
    Mission_BaroMode = false;
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
    Mission_BaroMode = true;
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
    NavigationMode = Do_PositionHold;
    if (GetAltitudeReached() && COMMAND_ARM_DISARM)
    {
      if (ThrottleIncrement >= THROTTLE_TAKEOFF_ASCENT)
      {
        STICKS.PreArm_Delay = false;
        WayPointMode = WP_START_MISSION;
      }
    }
    else
    {
      if (COMMAND_ARM_DISARM)
      {
        AutoTakeOff(true);
      }
      else
      {
        STICKS.PreArm_Delay = true;
      }
    }
    break;

  case GET_ALTITUDE:
    Mission_Timed_Count = 0;
    Get_Altitude();
    GPS_Flight_Mode = GPS_MODE_HOLD;
    SetThisPointToPositionHold();
    NavigationMode = Do_PositionHold;
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
    Navigation_Speed_Result = Calculate_Navigation_Speed(WP_NAVIGATION_SPEED);
    GPSCalculateNavigationRate(Navigation_Speed_Result);
    HeadingHoldTarget = WRap_18000(Target_Bearing) / 100;
    if ((Two_Points_Distance <= WAYPOINT_RADIUS * 100) || Point_Reached())
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
        AutoTakeOff(false);
      }
      //AVANÇA O WAYPOINT
      if (WayPointFlightMode[MissionNumber] == WP_ADVANCE)
      {
        GPS_Flight_Mode = WAYPOINT;
        WayPointMode = GET_ALTITUDE;
      }
      //GPS-HOLD TIMERIZADO
      if (WayPointFlightMode[MissionNumber] == WP_TIMED)
      {
        if (WayPointSync10Hz())
        {
          Mission_Timed_Count++; //10 ITERAÇÕES = 1 SEGUNDO
        }
        Do_GPS_Altitude = false;
        GPS_Flight_Mode = GPS_MODE_HOLD;
        SetThisPointToPositionHold();
        NavigationMode = Do_PositionHold;
        GPS_Flight_Mode = WAYPOINT;
        NavigationMode = Do_PositionHold;
        if (Mission_Timed_Count >= WayPointTimed[MissionNumber] * 10)
        {
          WayPointMode = GET_ALTITUDE;
        }
      }
      //LAND
      if (WayPointFlightMode[MissionNumber] == WP_LAND)
      {
        GPS_Flight_Mode = WAYPOINT;
        GPS_Flight_Mode = GPS_MODE_HOLD;
        SetThisPointToPositionHold();
        NavigationMode = Do_PositionHold;
        NavigationMode = Do_Land_Init;
      }
      //RTH
      if (WayPointFlightMode[MissionNumber] == WP_RTH)
      {
        Do_Mode_RTH_Now();
        HeadingHoldTarget = ATTITUDE.AngleOut[YAW];
      }
    }
    break;
  }
}

void AutoTakeOff(bool AutoTakeOff)
{
  if (!AutoTakeOff)
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
  RadioControllOutput[THROTTLE] = ThrottleIncrement;
  RCController[THROTTLE] = Constrain_16Bits(ThrottleIncrement, AttitudeThrottleMin, AttitudeThrottleMax);
}

void Get_Altitude()
{
  if (WayPointAltitude[MissionNumber] == 0)
  {
    SetAltitudeHold(10 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 1)
  {
    SetAltitudeHold(15 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 2)
  {
    SetAltitudeHold(20 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 3)
  {
    SetAltitudeHold(25 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 4)
  {
    SetAltitudeHold(30 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 5)
  {
    SetAltitudeHold(35 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 6)
  {
    SetAltitudeHold(40 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 7)
  {
    SetAltitudeHold(45 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 8)
  {
    SetAltitudeHold(50 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 9)
  {
    SetAltitudeHold(55 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 10)
  {
    SetAltitudeHold(60 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 11)
  {
    SetAltitudeHold(65 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 12)
  {
    SetAltitudeHold(70 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 13)
  {
    SetAltitudeHold(75 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 14)
  {
    SetAltitudeHold(80 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 15)
  {
    SetAltitudeHold(85 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 16)
  {
    SetAltitudeHold(90 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 17)
  {
    SetAltitudeHold(95 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 18)
  {
    SetAltitudeHold(100 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 19)
  {
    SetAltitudeHold(105 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 20)
  {
    SetAltitudeHold(110 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 21)
  {
    SetAltitudeHold(115 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 22)
  {
    SetAltitudeHold(120 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 23)
  {
    SetAltitudeHold(125 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 24)
  {
    SetAltitudeHold(130 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 25)
  {
    SetAltitudeHold(135 * 100);
  }
  else if (WayPointAltitude[MissionNumber] == 26)
  {
    SetAltitudeHold(140 * 100);
  }
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
      //4 BYTES DA PRIMEIRA LATITUDE
      STORAGEMANAGER.Write_8Bits(704, 0);
      STORAGEMANAGER.Write_8Bits(705, 0);
      STORAGEMANAGER.Write_8Bits(706, 0);
      STORAGEMANAGER.Write_8Bits(707, 0);
      //4 BYTES DA SEGUNDA LATITUDE
      STORAGEMANAGER.Write_8Bits(708, 0);
      STORAGEMANAGER.Write_8Bits(709, 0);
      STORAGEMANAGER.Write_8Bits(710, 0);
      STORAGEMANAGER.Write_8Bits(711, 0);
      //4 BYTES DA TERCEIRA LATITUDE
      STORAGEMANAGER.Write_8Bits(712, 0);
      STORAGEMANAGER.Write_8Bits(713, 0);
      STORAGEMANAGER.Write_8Bits(714, 0);
      STORAGEMANAGER.Write_8Bits(715, 0);
      //4 BYTES DA QUARTA LATITUDE
      STORAGEMANAGER.Write_8Bits(716, 0);
      STORAGEMANAGER.Write_8Bits(717, 0);
      STORAGEMANAGER.Write_8Bits(718, 0);
      STORAGEMANAGER.Write_8Bits(719, 0);
      //4 BYTES DA QUINTA LATITUDE
      STORAGEMANAGER.Write_8Bits(720, 0);
      STORAGEMANAGER.Write_8Bits(721, 0);
      STORAGEMANAGER.Write_8Bits(722, 0);
      STORAGEMANAGER.Write_8Bits(723, 0);
      //4 BYTES DA SEXTA LATITUDE
      STORAGEMANAGER.Write_8Bits(724, 0);
      STORAGEMANAGER.Write_8Bits(725, 0);
      STORAGEMANAGER.Write_8Bits(726, 0);
      STORAGEMANAGER.Write_8Bits(727, 0);
      //4 BYTES DA SETIMA LATITUDE
      STORAGEMANAGER.Write_8Bits(728, 0);
      STORAGEMANAGER.Write_8Bits(729, 0);
      STORAGEMANAGER.Write_8Bits(730, 0);
      STORAGEMANAGER.Write_8Bits(731, 0);
      //4 BYTES DA OITAVA LATITUDE
      STORAGEMANAGER.Write_8Bits(732, 0);
      STORAGEMANAGER.Write_8Bits(733, 0);
      STORAGEMANAGER.Write_8Bits(734, 0);
      STORAGEMANAGER.Write_8Bits(735, 0);
      //4 BYTES DA NONA LATITUDE
      STORAGEMANAGER.Write_8Bits(736, 0);
      STORAGEMANAGER.Write_8Bits(737, 0);
      STORAGEMANAGER.Write_8Bits(738, 0);
      STORAGEMANAGER.Write_8Bits(739, 0);
      //4 BYTES DA DECIMA LATITUDE
      STORAGEMANAGER.Write_8Bits(740, 0);
      STORAGEMANAGER.Write_8Bits(741, 0);
      STORAGEMANAGER.Write_8Bits(742, 0);
      STORAGEMANAGER.Write_8Bits(743, 0);
      //4 BYTES DA PRIMEIRA LONGITUDE
      STORAGEMANAGER.Write_8Bits(744, 0);
      STORAGEMANAGER.Write_8Bits(745, 0);
      STORAGEMANAGER.Write_8Bits(746, 0);
      STORAGEMANAGER.Write_8Bits(747, 0);
      //4 BYTES DA SEGUNDA LONGITUDE
      STORAGEMANAGER.Write_8Bits(748, 0);
      STORAGEMANAGER.Write_8Bits(749, 0);
      STORAGEMANAGER.Write_8Bits(750, 0);
      STORAGEMANAGER.Write_8Bits(751, 0);
      //4 BYTES DA TERCEIRA LONGITUDE
      STORAGEMANAGER.Write_8Bits(752, 0);
      STORAGEMANAGER.Write_8Bits(753, 0);
      STORAGEMANAGER.Write_8Bits(754, 0);
      STORAGEMANAGER.Write_8Bits(755, 0);
      //4 BYTES DA QUARTA LONGITUDE
      STORAGEMANAGER.Write_8Bits(756, 0);
      STORAGEMANAGER.Write_8Bits(757, 0);
      STORAGEMANAGER.Write_8Bits(758, 0);
      STORAGEMANAGER.Write_8Bits(759, 0);
      //4 BYTES DA QUINTA LONGITUDE
      STORAGEMANAGER.Write_8Bits(760, 0);
      STORAGEMANAGER.Write_8Bits(761, 0);
      STORAGEMANAGER.Write_8Bits(762, 0);
      STORAGEMANAGER.Write_8Bits(763, 0);
      //4 BYTES DA SEXTA LONGITUDE
      STORAGEMANAGER.Write_8Bits(764, 0);
      STORAGEMANAGER.Write_8Bits(765, 0);
      STORAGEMANAGER.Write_8Bits(766, 0);
      STORAGEMANAGER.Write_8Bits(767, 0);
      //4 BYTES DA SETIMA LONGITUDE
      STORAGEMANAGER.Write_8Bits(768, 0);
      STORAGEMANAGER.Write_8Bits(769, 0);
      STORAGEMANAGER.Write_8Bits(770, 0);
      STORAGEMANAGER.Write_8Bits(771, 0);
      //4 BYTES DA OITAVA LONGITUDE
      STORAGEMANAGER.Write_8Bits(772, 0);
      STORAGEMANAGER.Write_8Bits(773, 0);
      STORAGEMANAGER.Write_8Bits(774, 0);
      STORAGEMANAGER.Write_8Bits(775, 0);
      //4 BYTES DA NONA LONGITUDE
      STORAGEMANAGER.Write_8Bits(776, 0);
      STORAGEMANAGER.Write_8Bits(777, 0);
      STORAGEMANAGER.Write_8Bits(778, 0);
      STORAGEMANAGER.Write_8Bits(779, 0);
      //4 BYTES DA DECIMA LONGITUDE
      STORAGEMANAGER.Write_8Bits(780, 0);
      STORAGEMANAGER.Write_8Bits(781, 0);
      STORAGEMANAGER.Write_8Bits(782, 0);
      STORAGEMANAGER.Write_8Bits(783, 0);
      //LIMPA O TIMER DAS MISSÕES COM GPS-HOLD
      STORAGEMANAGER.Write_8Bits(784, 0);
      STORAGEMANAGER.Write_8Bits(785, 0);
      STORAGEMANAGER.Write_8Bits(786, 0);
      STORAGEMANAGER.Write_8Bits(787, 0);
      STORAGEMANAGER.Write_8Bits(788, 0);
      STORAGEMANAGER.Write_8Bits(789, 0);
      STORAGEMANAGER.Write_8Bits(790, 0);
      STORAGEMANAGER.Write_8Bits(791, 0);
      STORAGEMANAGER.Write_8Bits(792, 0);
      STORAGEMANAGER.Write_8Bits(793, 0);
      //LIMPA O MODO DE VOO DAS MISSÕES
      STORAGEMANAGER.Write_8Bits(794, 0);
      STORAGEMANAGER.Write_8Bits(795, 0);
      STORAGEMANAGER.Write_8Bits(796, 0);
      STORAGEMANAGER.Write_8Bits(797, 0);
      STORAGEMANAGER.Write_8Bits(798, 0);
      STORAGEMANAGER.Write_8Bits(799, 0);
      STORAGEMANAGER.Write_8Bits(800, 0);
      STORAGEMANAGER.Write_8Bits(801, 0);
      STORAGEMANAGER.Write_8Bits(802, 0);
      STORAGEMANAGER.Write_8Bits(803, 0);
      //LIMPA A ALTITUDE DAS MISSÕES
      STORAGEMANAGER.Write_8Bits(804, 0);
      STORAGEMANAGER.Write_8Bits(805, 0);
      STORAGEMANAGER.Write_8Bits(806, 0);
      STORAGEMANAGER.Write_8Bits(807, 0);
      STORAGEMANAGER.Write_8Bits(808, 0);
      STORAGEMANAGER.Write_8Bits(809, 0);
      STORAGEMANAGER.Write_8Bits(810, 0);
      STORAGEMANAGER.Write_8Bits(811, 0);
      STORAGEMANAGER.Write_8Bits(812, 0);
      STORAGEMANAGER.Write_8Bits(813, 0);
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
