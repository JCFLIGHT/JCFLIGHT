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
#include "NAVIGATION.h"
#include "FlightModes/FLIGHTMODES.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Scheduler/SCHEDULER.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "I2C/I2C.h"
#include "RadioControl/DECODE.h"
#include "FailSafe/FAILSAFE.h"
#include "GPS/GPSUBLOX.h"
#include "PID/RCPID.h"
#include "AHRS/AHRS.h"
#include "PID/PIDPARAMS.h"

//PARAMETROS DE NAVEGAÇÃO
#define CRUISE_DISTANCE 500   //DISTANCIA (EM CM) - DISTANCIA ENTRE O PRIMEIRO E O SEGUNDO PONTO A SER ATINGIDO
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

#define MAX_YAW_BANKANGLE 15

float Alt_kP = (float)ALTITUDE_PROPORTIONAL / 10.0f;
float Alt_kI = (float)ALTITUDE_INTEGRAL / 100.0f;
float Alt_kD = (float)ALTITUDE_DERIVATIVE / 1000.0f;

float Nav_kP = (float)NAVIGATION_PROPORTIONAL / 10.0f;
float Nav_kI = (float)NAVIGATION_INTEGRAL / 100.0f;
float Nav_kD = (float)NAVIGATION_DERIVATIVE / 1000.0f;

float Latitude_To_Circle;
float Longitude_To_Circle;
float Scale_Of_Circle;
float IntegralErrorOfNavigation;
float IntegralErrorOfAltitude;

uint8_t RTH_AltitudeOfPlane;

int16_t GPS_Heading;
int16_t AttitudeHeading;
int16_t Current_Heading;
int16_t AltitudeDifference = 0;
int16_t AltitudeSaturation[2] = {0, 0};
int16_t HeadingDifference;
int16_t SpeedDifference;
int16_t CurrentAltitude;
int16_t TargetAltitude;
int16_t Read_Throttle;

static int16_t PreviousAltitudeDifference;
static int16_t PreviousHeadingDifference;
static int16_t ThrottleBoost;
static int16_t AltitudeVector[6];
static int16_t NavigationDifferenceVector[6];
static int16_t NavigationDeltaSumPID;
static int16_t AltitudeDeltaSumPID;
static int16_t GPSTargetBearing;
static int16_t AltitudeError;
static int16_t GetThrottleToNavigation;

int32_t Altitude_For_Plane;
int32_t AltitudeHold_For_Plane;
int32_t HeadingToCircle;

void Circle_Mode_Update()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  if (!I2CResources.Found.Compass)
  {
    HeadingToCircle = WRap_18000(GPSParameters.Navigation.Misc.Get.GroundCourse * 10) / 100;
  }
  else
  {
    HeadingToCircle = GPSParameters.Navigation.Misc.Get.GroundCourse;
  }

  if (HeadingToCircle > 180)
  {
    HeadingToCircle -= 360;
  }

  Scale_Of_Circle = (89.832f / GPSParameters.ScaleDownOfLongitude) * CRUISE_DISTANCE;
  Latitude_To_Circle = Fast_Cosine(ConvertToRadians(HeadingToCircle));
  Longitude_To_Circle = Fast_Sine(ConvertToRadians(HeadingToCircle)) * GPSParameters.ScaleDownOfLongitude;
  GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE] += Latitude_To_Circle * Scale_Of_Circle;
  GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE] += Longitude_To_Circle * Scale_Of_Circle;
}

void AirPlaneUpdateNavigation(void)
{
  RTH_AltitudeOfPlane = GPSParameters.Home.Altitude;
  Read_Throttle = DECODE.GetRxChannelOutput(THROTTLE);
  CurrentAltitude = GPSParameters.Navigation.Misc.Get.Altitude - Altitude_For_Plane;
  TargetAltitude = AltitudeHold_For_Plane - Altitude_For_Plane;

  if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE) && CurrentAltitude < RTH_AltitudeOfPlane)
  {
    AltitudeHold_For_Plane = Altitude_For_Plane + RTH_AltitudeOfPlane;
  }

  if (!I2CResources.Found.Compass)
  {
    GPS_Heading = WRap_18000(GPSParameters.Navigation.Misc.Get.GroundCourse * 10) / 10;
    AttitudeHeading = GPS_Heading / 10;
  }
  else
  {
    GPS_Heading = GPSParameters.Navigation.Misc.Get.GroundCourse;
    AttitudeHeading = Attitude.EulerAngles.Yaw;
  }

  GPS_Heading = WRap_18000(GPS_Heading * 10) / 10;

  if (I2CResources.Found.Compass)
  {
    if (ABS(AttitudeHeading - (GPS_Heading / 10)) > 10 && GPSParameters.Navigation.Misc.Get.GroundSpeed > 200)
    {
      Current_Heading = GPS_Heading / 10;
    }
    else
    {
      Current_Heading = AttitudeHeading;
    }
  }
  else
  {
    Current_Heading = GPS_Heading / 10;
  }

  GPSTargetBearing = GPSParameters.Navigation.Bearing.TargetPrev / 100;
  HeadingDifference = GPSTargetBearing - Current_Heading;
  AltitudeError = CurrentAltitude - TargetAltitude;

  static Scheduler_Struct AirPlaneNavigationTimer;
  if (Scheduler(&AirPlaneNavigationTimer, SCHEDULER_SET_FREQUENCY(5, "Hz")))
  {

    const float DeltaTime_Navigation_PID = AirPlaneNavigationTimer.ActualTime * 1e-6f;

    if (ABS(AltitudeError) < 1)
    {
      GetThrottleToNavigation = MIDDLE_STICKS_PULSE;
    }
    else
    {
      GetThrottleToNavigation = Constrain_16Bits(MIDDLE_STICKS_PULSE - (AltitudeError * 8), 1300, AttitudeThrottleMax);
    }

    if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE) && AltitudeError >= 0)
    {
      DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
    }

    if (IS_FLIGHT_MODE_ACTIVE(RTH_MODE))
    {
      if (IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE))
      {
        AltitudeError = -ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue);
        GetThrottleToNavigation = AttitudeThrottleMax;
        if (CurrentAltitude < SAFE_NAV_ALT)
        {
          HeadingDifference = 0; //FORÇA UMA SUBIDA ATÉ UM VALOR MAIOR OU IGUAL A SAFE_NAV_ALT
        }
      }
      if ((GPSParameters.Home.Distance < SAFE_DECSCEND_ZONE) && CurrentAltitude > RTH_AltitudeOfPlane)
      {
        AltitudeHold_For_Plane = Altitude_For_Plane + RTH_AltitudeOfPlane;
      }
    }

    if (SystemInFailSafe() && (GPSParameters.Home.Distance < 10))
    {
      DISABLE_STATE(PRIMARY_ARM_DISARM);
      DISABLE_THIS_FLIGHT_MODE(CLIMBOUT_MODE);
      AltitudeHold_For_Plane = Altitude_For_Plane + 5;
    }

    if (GPSParameters.Home.Distance < 10)
    {
      HeadingDifference *= 0.1f;
    }

    HeadingDifference = WRap_18000(HeadingDifference * 100) / 100;

    if (ABS(HeadingDifference) > 170)
    {
      HeadingDifference = 175;
    }

    if (ABS(AltitudeError) <= 3)
    {
      IntegralErrorOfAltitude *= DeltaTime_Navigation_PID;
    }

    AltitudeError *= 10;
    IntegralErrorOfAltitude += (AltitudeError * Alt_kI) * DeltaTime_Navigation_PID;
    IntegralErrorOfAltitude = Constrain_Float(IntegralErrorOfAltitude, -500, 500);
    AltitudeSaturation[0] = (AltitudeError - PreviousAltitudeDifference);
    PreviousAltitudeDifference = AltitudeError;

    if (ABS(AltitudeSaturation[0]) > 100)
    {
      AltitudeSaturation[0] = 0;
    }

    AltitudeVector[0] = AltitudeVector[1];
    AltitudeVector[1] = AltitudeVector[2];
    AltitudeVector[2] = AltitudeVector[3];
    AltitudeVector[3] = AltitudeVector[4];
    AltitudeVector[4] = AltitudeVector[5];
    AltitudeVector[4] = AltitudeSaturation[0];
    AltitudeDeltaSumPID = 0;
    AltitudeDeltaSumPID += AltitudeVector[0];
    AltitudeDeltaSumPID += AltitudeVector[1];
    AltitudeDeltaSumPID += AltitudeVector[2];
    AltitudeDeltaSumPID += AltitudeVector[3];
    AltitudeDeltaSumPID += AltitudeVector[4];
    AltitudeDeltaSumPID = (AltitudeDeltaSumPID * Alt_kD) / DeltaTime_Navigation_PID;
    AltitudeDifference = AltitudeError * Alt_kP;
    AltitudeDifference += IntegralErrorOfAltitude;

    if (ABS(HeadingDifference) <= 3)
    {
      IntegralErrorOfNavigation *= DeltaTime_Navigation_PID;
    }

    HeadingDifference *= 10;
    IntegralErrorOfNavigation += (HeadingDifference * Nav_kI) * DeltaTime_Navigation_PID;
    IntegralErrorOfNavigation = Constrain_Float(IntegralErrorOfNavigation, -500, 500);
    AltitudeSaturation[1] = (HeadingDifference - PreviousHeadingDifference);
    PreviousHeadingDifference = HeadingDifference;

    if (ABS(AltitudeSaturation[1]) > 100)
    {
      AltitudeSaturation[1] = 0;
    }

    NavigationDifferenceVector[0] = NavigationDifferenceVector[1];
    NavigationDifferenceVector[1] = NavigationDifferenceVector[2];
    NavigationDifferenceVector[2] = NavigationDifferenceVector[3];
    NavigationDifferenceVector[3] = NavigationDifferenceVector[4];
    NavigationDifferenceVector[4] = NavigationDifferenceVector[5];
    NavigationDifferenceVector[4] = AltitudeSaturation[1];
    NavigationDeltaSumPID = 0;
    NavigationDeltaSumPID += NavigationDifferenceVector[0];
    NavigationDeltaSumPID += NavigationDifferenceVector[1];
    NavigationDeltaSumPID += NavigationDifferenceVector[2];
    NavigationDeltaSumPID += NavigationDifferenceVector[3];
    NavigationDeltaSumPID += NavigationDifferenceVector[4];
    NavigationDeltaSumPID = (NavigationDeltaSumPID * Nav_kD) / DeltaTime_Navigation_PID;
    HeadingDifference *= Nav_kP;
    HeadingDifference += IntegralErrorOfNavigation;
    GPSParameters.Navigation.AutoPilot.Control.Angle[PITCH] = Constrain_16Bits(AltitudeDifference / 10, -ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue)) + AltitudeDeltaSumPID;
    GPSParameters.Navigation.AutoPilot.Control.Angle[ROLL] = Constrain_16Bits(HeadingDifference / 10, -ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[ROLL_BANK_MAX].MinMaxValue)) + NavigationDeltaSumPID;
    GPSParameters.Navigation.AutoPilot.Control.Angle[YAW] = Constrain_16Bits(HeadingDifference / 10, -MAX_YAW_BANKANGLE * 10, MAX_YAW_BANKANGLE * 10) + NavigationDeltaSumPID;

    if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      GPSParameters.Navigation.AutoPilot.Control.Angle[PITCH] = Constrain_16Bits(GPSParameters.Navigation.AutoPilot.Control.Angle[PITCH], 0, ConvertDegreesToDecidegrees(GET_SET[PITCH_BANK_MAX].MinMaxValue));
    }

    if (!IS_FLIGHT_MODE_ACTIVE(CLIMBOUT_MODE))
    {
      GPSParameters.Navigation.AutoPilot.Control.Angle[PITCH] -= (ABS(Attitude.EulerAngles.Roll));
    }

    GetThrottleToNavigation -= Constrain_16Bits(Attitude.EulerAngles.Pitch * PITCH_COMP, 0, 450);
    SpeedDifference = (GPS_MINSPEED - GPSParameters.Navigation.Misc.Get.GroundSpeed) * I_TERM;

    if (GPSParameters.Navigation.Misc.Get.GroundSpeed < GPS_MINSPEED - 50 || GPSParameters.Navigation.Misc.Get.GroundSpeed > GPS_MINSPEED + 50)
    {
      ThrottleBoost += SpeedDifference;
    }

    ThrottleBoost = Constrain_16Bits(ThrottleBoost, 0, 500);
    GetThrottleToNavigation += ThrottleBoost;
    GetThrottleToNavigation = Constrain_16Bits(GetThrottleToNavigation, 1300, AttitudeThrottleMax);
  }

  if ((!IS_FLIGHT_MODE_ACTIVE(STABILIZE_MODE)) || (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE) && !SystemInFailSafe()))
  {
    GetThrottleToNavigation = Read_Throttle;
    GPSParameters.Navigation.AutoPilot.Control.Angle[PITCH] = 0;
    GPSParameters.Navigation.AutoPilot.Control.Angle[ROLL] = 0;
    GPSParameters.Navigation.AutoPilot.Control.Angle[YAW] = 0;
  }

  RCController[THROTTLE] = GetThrottleToNavigation;
  RCController[YAW] += GPSParameters.Navigation.AutoPilot.Control.Angle[YAW];
}

void PlaneResetNavigation(void)
{
  ThrottleBoost = 0;
  IntegralErrorOfNavigation = 0;
  IntegralErrorOfAltitude = 0;
  PreviousAltitudeDifference = 0;
  PreviousHeadingDifference = 0;
  AltitudeVector[0] = 0;
  AltitudeVector[1] = 0;
  AltitudeVector[2] = 0;
  AltitudeVector[3] = 0;
  AltitudeVector[4] = 0;
  NavigationDifferenceVector[0] = 0;
  NavigationDifferenceVector[1] = 0;
  NavigationDifferenceVector[2] = 0;
  NavigationDifferenceVector[3] = 0;
  NavigationDifferenceVector[4] = 0;
}