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

#include "AIRPLANENAVIGATION.h"
#include "MULTIROTORNAVIGATION.h"
#include "Common/VARIABLES.h"
#include "FlightModes/FLIGHTMODES.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"

//PARAMETROS DE NAVEGAÇÃO
#define CRUISE_DISTANCE 500   //DISTANCIA (EM CM)
#define SAFE_NAV_ALT 25       //ALTITUDE SEGURA PARA O MODO DE NAVEGAÇÃO COM GPS HEADING
#define SAFE_DECSCEND_ZONE 50 //VOLOR SEGURO PARA MANTER A ALTITUDE DO PLANE (EM METROS)
#define PITCH_COMP 0.5f       //COMPENSAÇÃO DE ANGULO DINAMICO
#define GPS_MINSPEED 500      //500 = ~18KM/H
#define I_TERM 0.1f           //FATOR DE MULTIPLICAÇÃO

//PARAMETROS DO PID
#define ALTITUDE_PROPORTIONAL 30
#define ALTITUDE_INTEGRAL 20
#define ALTITUDE_DERIVATIVE 45

#define NAVIGATION_PROPORTIONAL 30
#define NAVIGATION_INTEGRAL 20
#define NAVIGATION_DERIVATIVE 45

float IntegralErrorOfNavigation;
float IntegralErrorOfAltitude;

float Alt_kP = (float)ALTITUDE_PROPORTIONAL / 10.0f;
float Alt_kI = (float)ALTITUDE_INTEGRAL / 100.0f;
float Alt_kD = (float)ALTITUDE_DERIVATIVE / 1000.0f;

float Nav_kP = (float)NAVIGATION_PROPORTIONAL / 10.0f;
float Nav_kI = (float)NAVIGATION_INTEGRAL / 100.0f;
float Nav_kD = (float)NAVIGATION_DERIVATIVE / 1000.0f;

static int16_t PreviousAltitudeDifference;
static int16_t PreviousHeadingDifference;
static int16_t ThrottleBoost;
static int16_t AltitudeVector[6];
static int16_t NavigationDifferenceVector[6];
int32_t GPS_Altitude_For_Plane;
int32_t GPS_AltitudeHold_For_Plane;

void Cruise_Mode_Update()
{
  if (GetFrameStateOfMultirotor())
    return;
  float Latitude_To_Cruise;
  float Longitude_To_Cruise;
  float Scale_Of_Cruise;
  int32_t HeadingToCruise = GPS_Ground_Course / 10;
  if (HeadingToCruise > 180)
    HeadingToCruise -= 360;
  Scale_Of_Cruise = (89.832f / ScaleDownOfLongitude) * CRUISE_DISTANCE;
  Latitude_To_Cruise = cos(HeadingToCruise * 0.0174532925f);
  Longitude_To_Cruise = sin(HeadingToCruise * 0.0174532925f) * ScaleDownOfLongitude;
  Coordinates_To_Navigation[0] += Latitude_To_Cruise * Scale_Of_Cruise;
  Coordinates_To_Navigation[1] += Longitude_To_Cruise * Scale_Of_Cruise;
}

void PlaneUpdateNavigation(void)
{
  int16_t GPS_Heading = GPS_Ground_Course;
  int16_t Current_Heading;
  int16_t AltitudeDifference = 0;
  uint8_t RTH_AltitudeOfPlane = RTH_Altitude;
  int16_t Saturation[2] = {0, 0};
  static int16_t NavigationDeltaSumPID;
  static int16_t AltitudeDeltaSumPID;
  static int16_t GPSTargetBearing;
  static int16_t AltitudeError;
  static int16_t GetThrottleToNavigation;
  int16_t Read_Throttle = RadioControllOutput[THROTTLE];
  int16_t HeadingDifference;
  float TimePIDOfNavigationPrimary;
  static uint32_t TimePIDOfNavigation;
  int16_t GroundSpeed;
  int16_t SpeedDifference;
  static uint32_t NavTimer = 0;
  int16_t CurrentAltitude = GPS_Altitude - GPS_Altitude_For_Plane;
  int16_t TargetAltitude = GPS_AltitudeHold_For_Plane - GPS_Altitude_For_Plane;
  if (CLIMBOUT_FW && CurrentAltitude < RTH_AltitudeOfPlane)
    GPS_AltitudeHold_For_Plane = GPS_Altitude_For_Plane + RTH_AltitudeOfPlane;
  GPS_Heading = WRap_180(GPS_Heading * 10) / 10;
  Current_Heading = GPS_Heading / 10;
  GPSTargetBearing = Original_Target_Bearing / 100;
  HeadingDifference = GPSTargetBearing - Current_Heading;
  AltitudeError = CurrentAltitude - TargetAltitude;
  if (AVRTIME.SchedulerMillis() - NavTimer >= 200)
  {
    NavTimer = AVRTIME.SchedulerMillis();
    if (ABS_16BITS(AltitudeError) < 1)
      GetThrottleToNavigation = 1500;
    else
      GetThrottleToNavigation = Constrain_16Bits(1500 - (AltitudeError * 8), 1300, 1900);
    if (CLIMBOUT_FW && AltitudeError >= 0)
      CLIMBOUT_FW = false;
    if (GPS_HOME_MODE_FW)
    {
      if (CLIMBOUT_FW)
      {
        AltitudeError = -(150);
        GetThrottleToNavigation = 1800;
        if (CurrentAltitude < SAFE_NAV_ALT)
          HeadingDifference = 0;
      }
      if ((DistanceToHome < SAFE_DECSCEND_ZONE) && CurrentAltitude > RTH_AltitudeOfPlane)
        GPS_AltitudeHold_For_Plane = GPS_Altitude_For_Plane + RTH_AltitudeOfPlane;
    }
    if (Fail_Safe_Event && (DistanceToHome < 10))
    {
      COMMAND_ARM_DISARM = false;
      CLIMBOUT_FW = false;
      GPS_AltitudeHold_For_Plane = GPS_Altitude_For_Plane + 5;
    }
    if (DistanceToHome < 10)
      HeadingDifference *= 0.1f;
    HeadingDifference = WRap_180(HeadingDifference * 100) / 100;
    if (ABS_16BITS(HeadingDifference) > 170)
      HeadingDifference = 175;
    TimePIDOfNavigationPrimary = (float)(AVRTIME.SchedulerMillis() - TimePIDOfNavigation) / 1000;
    TimePIDOfNavigation = AVRTIME.SchedulerMillis();
    if (ABS_16BITS(AltitudeError) <= 3)
      IntegralErrorOfAltitude *= TimePIDOfNavigationPrimary;
    AltitudeError *= 10;
    IntegralErrorOfAltitude += (AltitudeError * Alt_kI) * TimePIDOfNavigationPrimary;
    IntegralErrorOfAltitude = Constrain_Float(IntegralErrorOfAltitude, -500, 500);
    Saturation[0] = (AltitudeError - PreviousAltitudeDifference);
    PreviousAltitudeDifference = AltitudeError;
    if (ABS_16BITS(Saturation[0]) > 100)
      Saturation[0] = 0;
    AltitudeVector[0] = AltitudeVector[1];
    AltitudeVector[1] = AltitudeVector[2];
    AltitudeVector[2] = AltitudeVector[3];
    AltitudeVector[3] = AltitudeVector[4];
    AltitudeVector[4] = AltitudeVector[5];
    AltitudeVector[4] = Saturation[0];
    AltitudeDeltaSumPID = 0;
    AltitudeDeltaSumPID += AltitudeVector[0];
    AltitudeDeltaSumPID += AltitudeVector[1];
    AltitudeDeltaSumPID += AltitudeVector[2];
    AltitudeDeltaSumPID += AltitudeVector[3];
    AltitudeDeltaSumPID += AltitudeVector[4];
    AltitudeDeltaSumPID = (AltitudeDeltaSumPID * Alt_kD) / TimePIDOfNavigationPrimary;
    AltitudeDifference = AltitudeError * Alt_kP;
    AltitudeDifference += (IntegralErrorOfAltitude);
    if (ABS_16BITS(HeadingDifference) <= 3)
      IntegralErrorOfNavigation *= TimePIDOfNavigationPrimary;
    HeadingDifference *= 10;
    IntegralErrorOfNavigation += (HeadingDifference * Nav_kI) * TimePIDOfNavigationPrimary;
    IntegralErrorOfNavigation = Constrain_Float(IntegralErrorOfNavigation, -500, 500);
    Saturation[1] = (HeadingDifference - PreviousHeadingDifference);
    PreviousHeadingDifference = HeadingDifference;
    if (ABS_16BITS(Saturation[1]) > 100)
      Saturation[1] = 0;
    NavigationDifferenceVector[0] = NavigationDifferenceVector[1];
    NavigationDifferenceVector[1] = NavigationDifferenceVector[2];
    NavigationDifferenceVector[2] = NavigationDifferenceVector[3];
    NavigationDifferenceVector[3] = NavigationDifferenceVector[4];
    NavigationDifferenceVector[4] = NavigationDifferenceVector[5];
    NavigationDifferenceVector[4] = Saturation[1];
    NavigationDeltaSumPID = 0;
    NavigationDeltaSumPID += NavigationDifferenceVector[0];
    NavigationDeltaSumPID += NavigationDifferenceVector[1];
    NavigationDeltaSumPID += NavigationDifferenceVector[2];
    NavigationDeltaSumPID += NavigationDifferenceVector[3];
    NavigationDeltaSumPID += NavigationDifferenceVector[4];
    NavigationDeltaSumPID = (NavigationDeltaSumPID * Nav_kD) / TimePIDOfNavigationPrimary;
    HeadingDifference *= Nav_kP;
    HeadingDifference += IntegralErrorOfNavigation;
    GPS_Angle[PITCH] = Constrain_16Bits(AltitudeDifference / 10, -150, 150) + AltitudeDeltaSumPID;
    GPS_Angle[ROLL] = Constrain_16Bits(HeadingDifference / 10, -200, 200) + NavigationDeltaSumPID;
    GPS_Angle[YAW] = Constrain_16Bits(HeadingDifference / 10, -150, 150) + NavigationDeltaSumPID;
    if (!COMMAND_ARM_DISARM)
      GPS_Angle[PITCH] = Constrain_16Bits(GPS_Angle[PITCH], 0, 150);
    if (!CLIMBOUT_FW)
      GPS_Angle[PITCH] -= (ABS_16BITS(ATTITUDE.AngleOut[ROLL]));
    GetThrottleToNavigation -= Constrain_16Bits(ATTITUDE.AngleOut[PITCH] * PITCH_COMP, 0, 450);
    GroundSpeed = GPS_Ground_Speed;
    SpeedDifference = (GPS_MINSPEED - GroundSpeed) * I_TERM;
    if (GPS_Ground_Speed < GPS_MINSPEED - 50 || GPS_Ground_Speed > GPS_MINSPEED + 50)
      ThrottleBoost += SpeedDifference;
    ThrottleBoost = Constrain_16Bits(ThrottleBoost, 0, 500);
    GetThrottleToNavigation += ThrottleBoost;
    GetThrottleToNavigation = Constrain_16Bits(GetThrottleToNavigation, 1300, 1900);
  }
  if ((!Do_Stabilize_Mode) || (Do_IOC_Mode && !Fail_Safe_Event))
  {
    GetThrottleToNavigation = Read_Throttle;
    GPS_Angle[PITCH] = 0;
    GPS_Angle[ROLL] = 0;
    GPS_Angle[YAW] = 0;
  }
  RCController[THROTTLE] = GetThrottleToNavigation;
  RCController[YAW] += GPS_Angle[YAW];
}

void PlaneResetNavigation(void)
{
  IntegralErrorOfNavigation = 0;
  IntegralErrorOfAltitude = 0;
  PreviousAltitudeDifference = 0;
  PreviousHeadingDifference = 0;
  ThrottleBoost = 0;
  AltitudeVector[0] = 0;
  NavigationDifferenceVector[0] = 0;
  AltitudeVector[1] = 0;
  NavigationDifferenceVector[1] = 0;
  AltitudeVector[2] = 0;
  NavigationDifferenceVector[2] = 0;
  AltitudeVector[3] = 0;
  NavigationDifferenceVector[3] = 0;
  AltitudeVector[4] = 0;
  NavigationDifferenceVector[4] = 0;
}