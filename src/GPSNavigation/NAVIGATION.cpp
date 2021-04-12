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

#include "NAVIGATION.h"
#include "PID/PIDPARAMS.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Declination/AUTODECLINATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "PID/GPSPID.h"
#include "BAR/BAR.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"
#include "Yaw/HEADINGHOLD.h"
#include "GPS/GPSUBLOX.h"
#include "FlightModes/FLIGHTMODES.h"
#include "InertialNavigation/INS.h"
#include "AHRS/AHRS.h"
#include "Barometer/BAROBACKEND.h"
#include "Param/PARAM.h"

GPS_Parameters_Struct GPS_Parameters;

#ifdef __AVR_ATmega2560__
#define NAVTILTCOMPENSATION 20 //RETIRADO DA ARDUPILOT
#else
#define NAVTILTCOMPENSATION JCF_Param.GPS_TiltCompensation
#endif
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR 1.113195f //RETIRADO DA ARDUPILOT - VALOR ANTERIOR 1.11318845f
#define MIN_NAVIGATION_SPEED 100                                   //100CM/S ~ 3.6KM/M -> VELOCIDADE MINIMA SUPORTADA
#define TIME_TO_INIT_LAND 100                                      //MS
#define CROSSTRACK_ERROR 0.4                                       //TESTAR COM 1 FUTURAMENTE

void GPS_Adjust_Heading()
{
  HeadingHoldTarget = WRap_18000(GPS_Parameters.Navigation.Bearing.ActualTarget) / 100;
}

void GPS_Calcule_Bearing(int32_t InputLatitude, int32_t InputLongitude, int32_t *Bearing)
{
  int32_t Adjust_OffSet_Lat = (InputLatitude - GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE]) / GPS_Parameters.ScaleDownOfLongitude;
  int32_t Adjust_OffSet_Long = InputLongitude - GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
  *Bearing = 9000 + Fast_Atan2(-Adjust_OffSet_Lat, Adjust_OffSet_Long) * 5729.57795f;
  if (*Bearing < 0)
  {
    *Bearing += 36000;
  }
}

void GPS_Calcule_Distance_In_CM(int32_t InputLatitude, int32_t InputLongitude, int32_t *CalculateDistance)
{
  float DistanceOfLatitude = (float)(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - InputLatitude);
  float DistanceOfLongitude = (float)(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - InputLongitude) * GPS_Parameters.ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(SquareFloat(DistanceOfLatitude) + SquareFloat(DistanceOfLongitude)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
}

void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance)
{
  GPS_Parameters.Home.INS.Distance[COORD_LATITUDE] = (GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - GPS_Parameters.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
  GPS_Parameters.Home.INS.Distance[COORD_LONGITUDE] = (GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - GPS_Parameters.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPS_Parameters.ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(Square32Bits(GPS_Parameters.Home.INS.Distance[COORD_LATITUDE]) + Square32Bits(GPS_Parameters.Home.INS.Distance[COORD_LONGITUDE]));
}

int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity)
{
  static int16_t Coordinates_Navigation_Speed = MIN_NAVIGATION_SPEED;
  Maximum_Velocity = MIN(Maximum_Velocity, GPS_Parameters.Navigation.Coordinates.Distance);
  Maximum_Velocity = MAX(Maximum_Velocity, MIN_NAVIGATION_SPEED);
  if (Maximum_Velocity > Coordinates_Navigation_Speed)
  {
    Coordinates_Navigation_Speed += (int16_t)(100.0f * GPS_Parameters.DeltaTime.Navigation);
    Maximum_Velocity = Coordinates_Navigation_Speed;
  }
  return Maximum_Velocity;
}

static void GPS_Calcule_Velocity(void)
{
  static int16_t Previous_Velocity[2] = {0, 0};
  static int32_t Last_CoordinatesOfGPS[2] = {0, 0};
  static bool IgnoreFirstPeak = false;
  if (IgnoreFirstPeak)
  {
    float DeltaTimeStored;
    if (GPS_Parameters.DeltaTime.Navigation >= 0.07f && GPS_Parameters.DeltaTime.Navigation <= 0.13f)
    {
      DeltaTimeStored = 0.1f;
    }
    else if (GPS_Parameters.DeltaTime.Navigation >= 0.17f && GPS_Parameters.DeltaTime.Navigation <= 0.23f)
    {
      DeltaTimeStored = 0.2f;
    }
    else
    {
      DeltaTimeStored = GPS_Parameters.DeltaTime.Navigation;
    }
    DeltaTimeStored = 1.0f / DeltaTimeStored;
    GPS_Parameters.Navigation.Speed[COORD_LATITUDE] = (float)(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - Last_CoordinatesOfGPS[COORD_LATITUDE]) * DeltaTimeStored;
    GPS_Parameters.Navigation.Speed[COORD_LONGITUDE] = (float)(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - Last_CoordinatesOfGPS[COORD_LONGITUDE]) * GPS_Parameters.ScaleDownOfLongitude * DeltaTimeStored;
    GPS_Parameters.Navigation.Speed[COORD_LATITUDE] = (GPS_Parameters.Navigation.Speed[COORD_LATITUDE] + Previous_Velocity[COORD_LATITUDE]) / 2;
    GPS_Parameters.Navigation.Speed[COORD_LONGITUDE] = (GPS_Parameters.Navigation.Speed[COORD_LONGITUDE] + Previous_Velocity[COORD_LONGITUDE]) / 2;
    Previous_Velocity[COORD_LATITUDE] = GPS_Parameters.Navigation.Speed[COORD_LATITUDE];
    Previous_Velocity[COORD_LONGITUDE] = GPS_Parameters.Navigation.Speed[COORD_LONGITUDE];
  }
  IgnoreFirstPeak = true;
  Last_CoordinatesOfGPS[COORD_LATITUDE] = GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE];
  Last_CoordinatesOfGPS[COORD_LONGITUDE] = GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
}

bool Point_Reached(void)
{
  int32_t TargetCalculed;
  TargetCalculed = GPS_Parameters.Navigation.Bearing.ActualTarget - GPS_Parameters.Navigation.Bearing.TargetPrev;
  TargetCalculed = WRap_18000(TargetCalculed);
  return (ABS(TargetCalculed) > 10000);
}

void GPS_Process_FlightModes(float DeltaTime)
{
  uint32_t CalculateDistance;
  int32_t CalculateDirection;
  //SAIA DA FUNÇÃO SE O GPS ESTIVER RUIM
  if (Get_GPS_In_Bad_Condition())
  {
    GPS_Parameters.Navigation.Misc.Velocity.NEDStatus = false;
    return;
  }
  //SAFE PARA RESETAR O HOME-POINT
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    GPS_Parameters.Home.Marked = false;
  }
  //RESETA O HOME-POINT AO ARMAR
  if (!GPS_Parameters.Home.Marked && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Reset_Home_Point();
  }
  //OBTÉM A DECLINAÇÃO MAGNETICA AUTOMATICAMENTE
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !GPS_Parameters.Declination.Pushed)
  {
    AUTODECLINATION.Set_Initial_Location(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE], GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE]);
    GPS_Parameters.Declination.PushedCount++;
  }
  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (AUTODECLINATION.GetDeclination() != STORAGEMANAGER.Read_Float(MAG_DECLINATION_ADDR) && //VERIFICA SE O VALOR É DIFERENTE
      !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&                                                //CHECA SE ESTÁ DESARMADO
      AUTODECLINATION.GetDeclination() != 0 &&                                               //CHECA SE O VALOR É DIFERENTE DE ZERO
      !GPS_Parameters.Declination.Pushed &&                                                  //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      GPS_Parameters.Declination.PushedCount > 250)                                          //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(MAG_DECLINATION_ADDR, AUTODECLINATION.GetDeclination());
    GPS_Parameters.Declination.Pushed = true;
  }
  GPS_Parameters.DeltaTime.Navigation = DeltaTime;
  GPS_Parameters.DeltaTime.Navigation = MIN(GPS_Parameters.DeltaTime.Navigation, 1.0f);
  GPS_Calcule_Bearing(GPS_Parameters.Home.Coordinates[COORD_LATITUDE], GPS_Parameters.Home.Coordinates[COORD_LONGITUDE], &CalculateDirection);
  GPS_Calcule_Distance_To_Home(&CalculateDistance);
  if (!GPS_Parameters.Home.Marked)
  {
    GPS_Parameters.Home.Distance = 0;
    GPS_Parameters.Home.Direction = 0;
  }
  else
  {
    GPS_Parameters.Home.Direction = CalculateDirection / 100; //FUTURAMENTE PARA O GCS E OSD
    GPS_Parameters.Home.Distance = CalculateDistance / 100;
  }
  GPS_Calcule_Velocity();
  if (Get_GPS_Only_Flight_Modes_In_Use())
  {
    GPS_Calcule_Bearing(GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Parameters.Navigation.Bearing.ActualTarget);
    GPS_Calcule_Distance_In_CM(GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Parameters.Navigation.Coordinates.Distance);

    int16_t CalculateNavigationSpeed = 0;

    if (GetFrameStateOfAirPlane())
    {
      GPS_Parameters.Mode.Navigation = DO_RTH_ENROUTE;
    }

    switch (GPS_Parameters.Mode.Navigation)
    {

    case DO_NONE:
      break;

    case DO_POSITION_HOLD:
      break;

    case DO_START_RTH:
      if (GPS_Parameters.Home.Distance <= JCF_Param.GPS_RTH_Land)
      {
        GPS_Parameters.Mode.Navigation = DO_LAND_INIT;
        HeadingHoldTarget = GPS_Parameters.Navigation.Bearing.InitalTarget;
      }
      else if (GetAltitudeReached())
      {
        Set_Next_Point_To_Navigation(GPS_Parameters.Home.Coordinates[COORD_LATITUDE], GPS_Parameters.Home.Coordinates[COORD_LONGITUDE]);
        GPS_Parameters.Mode.Navigation = DO_RTH_ENROUTE;
      }
      break;

    case DO_RTH_ENROUTE:
      CalculateNavigationSpeed = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
      GPSCalculateNavigationRate(CalculateNavigationSpeed);
      GPS_Adjust_Heading();
      if ((GPS_Parameters.Navigation.Coordinates.Distance <= ConvertCMToMeters(JCF_Param.GPS_WP_Radius)) || Point_Reached())
      {
        if (GetFrameStateOfMultirotor())
        {
          GPS_Parameters.Mode.Navigation = DO_LAND_INIT;
        }
        HeadingHoldTarget = GPS_Parameters.Navigation.Bearing.InitalTarget;
      }
      break;

    case DO_LAND_INIT:
      Do_RTH_Or_Land_Call_Alt_Hold = true;
      SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
      GPS_Parameters.DeltaTime.InitLand = SCHEDULERTIME.GetMillis() + TIME_TO_INIT_LAND;
      GPS_Parameters.Mode.Navigation = DO_LAND_SETTLE;
      break;

    case DO_LAND_SETTLE:
      if (SCHEDULERTIME.GetMillis() >= GPS_Parameters.DeltaTime.InitLand)
      {
        GPS_Parameters.Mode.Navigation = DO_LAND_IN_PROGRESS;
      }
      break;

    case DO_LAND_IN_PROGRESS:
      if (GetLanded())
      {
        GPS_Parameters.Mode.Navigation = DO_LANDED;
      }
      else if (GetGroundDetected())
      {
        GPS_Parameters.Mode.Navigation = DO_LAND_DETECTED;
      }
      break;

    case DO_LAND_DETECTED:
      if (GetLanded())
      {
        GPS_Parameters.Mode.Navigation = DO_LANDED;
      }
      break;

    case DO_LANDED:
      DISABLE_STATE(PRIMARY_ARM_DISARM);
      Do_RTH_Or_Land_Call_Alt_Hold = false;
      BEEPER.Play(BEEPER_ACTION_SUCCESS);
      GPS_Reset_Navigation();
      break;
    }
  }
}

void Do_Mode_RTH_Now()
{
  if (Barometer.INS.Altitude.Estimated < ConvertCMToMeters(GPS_Parameters.Home.Altitude))
  {
    SetAltitudeToHold(ConvertCMToMeters(GPS_Parameters.Home.Altitude));
  }
  else
  {
    SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
  }
  SetThisPointToPositionHold();
}

void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput)
{
  GPS_Parameters.ScaleDownOfLongitude = Constrain_Float(Fast_Cosine(ConvertToRadians((ConvertCoordinateToFloatingPoint(LatitudeVectorInput)))), 0.01f, 1.0f);
}

void Set_Next_Point_To_Navigation(int32_t Latitude_Destiny, int32_t Longitude_Destiny)
{
  GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE] = Latitude_Destiny;
  GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE] = Longitude_Destiny;
  GPS_Calcule_Longitude_Scaling(Latitude_Destiny);
  Circle_Mode_Update();
  GPS_Calcule_Bearing(GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Parameters.Navigation.Bearing.ActualTarget);
  GPS_Calcule_Distance_In_CM(GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPS_Parameters.Navigation.Coordinates.Distance);
  INS.Position.Hold[COORD_LATITUDE] = (GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LATITUDE] - GPS_Parameters.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
  INS.Position.Hold[COORD_LONGITUDE] = (GPS_Parameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE] - GPS_Parameters.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPS_Parameters.ScaleDownOfLongitude;
  GPS_Parameters.Navigation.Bearing.TargetPrev = GPS_Parameters.Navigation.Bearing.ActualTarget;
}

void SetThisPointToPositionHold()
{
  INS.Position.Hold[COORD_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + INS.EarthFrame.Velocity[INS_LATITUDE] * PositionHoldPID.kI;
  INS.Position.Hold[COORD_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + INS.EarthFrame.Velocity[INS_LONGITUDE] * PositionHoldPID.kI;
}

static void ApplyINSPositionHoldPIDControl(float DeltaTime)
{
  for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
  {
    int32_t INSPositionError = INS.Position.Hold[IndexCount] - INS.EarthFrame.Position[IndexCount];
    int32_t GPSTargetSpeed = GPSGetProportional(INSPositionError, &PositionHoldPID);
    GPSTargetSpeed = Constrain_32Bits(GPSTargetSpeed, -1000, 1000);
    int32_t RateError = GPSTargetSpeed - INS.EarthFrame.Velocity[IndexCount];
    RateError = Constrain_32Bits(RateError, -1000, 1000);
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(RateError, &PositionHoldRatePID) + GPSGetIntegral(RateError, DeltaTime, &PositionHoldRatePIDArray[IndexCount], &PositionHoldRatePID);
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] -= Constrain_16Bits((INS.AccelerationEarthFrame_Filtered[IndexCount] * PositionHoldRatePID.kD), -2000, 2000);
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount], -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue));
    NavigationPIDArray[IndexCount].GPSFilter.IntegralSum = PositionHoldRatePIDArray[IndexCount].GPSFilter.IntegralSum;
  }
}

void ApplyPosHoldPIDControl(float DeltaTime)
{
  if (!GetTakeOffInProgress() && !GetGroundDetectedFor100ms())
  {
    ApplyINSPositionHoldPIDControl(DeltaTime);
  }
  else
  {
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LATITUDE]);
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LONGITUDE]);
  }
}

bool Get_Safe_State_For_Pos_Hold(void)
{
  return GPS_Parameters.Mode.Navigation == DO_LAND_INIT || GPS_Parameters.Mode.Navigation == DO_LAND_SETTLE ||
         GPS_Parameters.Mode.Navigation == DO_LAND_IN_PROGRESS || GPS_Parameters.Mode.Navigation == DO_POSITION_HOLD ||
         GPS_Parameters.Mode.Navigation == DO_START_RTH;
}

int16_t GPS_Update_CrossTrackError(void)
{
  float TargetCalculed = (GPS_Parameters.Navigation.Bearing.ActualTarget - GPS_Parameters.Navigation.Bearing.TargetPrev) * 0.000174532925f;
  return Fast_Sine(TargetCalculed) * GPS_Parameters.Navigation.Coordinates.Distance;
}

void GPSCalculateNavigationRate(uint16_t Maximum_Velocity)
{
  float Trigonometry[2];
  float NavCompensation;
  int32_t Target_Speed[2];
  int16_t Cross_Speed = GPS_Update_CrossTrackError() * CROSSTRACK_ERROR;
  Cross_Speed = Constrain_16Bits(Cross_Speed, -200, 200);
  Cross_Speed = -Cross_Speed;
  float TargetCalculed = (9000L - GPS_Parameters.Navigation.Bearing.ActualTarget) * 0.000174532925f;
  Trigonometry[COORD_LATITUDE] = Fast_Sine(TargetCalculed);
  Trigonometry[COORD_LONGITUDE] = Fast_Cosine(TargetCalculed);
  Target_Speed[COORD_LATITUDE] = Cross_Speed * Trigonometry[COORD_LONGITUDE] + Maximum_Velocity * Trigonometry[COORD_LATITUDE];
  Target_Speed[COORD_LONGITUDE] = Maximum_Velocity * Trigonometry[COORD_LONGITUDE] - Cross_Speed * Trigonometry[COORD_LATITUDE];
  for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
  {
    GPS_Parameters.Navigation.RateError[IndexCount] = Target_Speed[IndexCount] - GPS_Parameters.Navigation.Speed[IndexCount];
    GPS_Parameters.Navigation.RateError[IndexCount] = Constrain_16Bits(GPS_Parameters.Navigation.RateError[IndexCount], -1000, 1000);
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(GPS_Parameters.Navigation.RateError[IndexCount], &NavigationPID) +
                                                                GPSGetIntegral(GPS_Parameters.Navigation.RateError[IndexCount], GPS_Parameters.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID) +
                                                                GPSGetDerivative(GPS_Parameters.Navigation.RateError[IndexCount], GPS_Parameters.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID);

    if (NAVTILTCOMPENSATION != 0)
    {
      NavCompensation = Target_Speed[IndexCount] * Target_Speed[IndexCount] * ((float)NAVTILTCOMPENSATION * 0.0001f);
      if (Target_Speed[IndexCount] < 0)
      {
        NavCompensation = -NavCompensation;
      }
    }
    else
    {
      NavCompensation = 0;
    }
    GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPS_Parameters.Navigation.AutoPilot.INS.Angle[IndexCount] + NavCompensation, -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue));
    PositionHoldRatePIDArray[IndexCount].GPSFilter.IntegralSum = NavigationPIDArray[IndexCount].GPSFilter.IntegralSum;
  }
}

void Reset_Home_Point(void)
{
  GPS_Parameters.Home.Coordinates[COORD_LATITUDE] = GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE];
  GPS_Parameters.Home.Coordinates[COORD_LONGITUDE] = GPS_Parameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
  GPS_Calcule_Longitude_Scaling(GPS_Parameters.Navigation.Coordinates.Actual[COORD_LATITUDE]);
  GPS_Parameters.Navigation.Bearing.InitalTarget = Attitude.EulerAngles.Yaw;
  Altitude_For_Plane = GPS_Parameters.Navigation.Misc.Get.Altitude;
  GPS_Parameters.Home.Marked = true;
}

void GPS_Reset_Navigation(void)
{
  GPS_Parameters.Mode.Navigation = DO_NONE;
  GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
  GPS_Parameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
  ResetAllGPSPID();
  if (GetFrameStateOfAirPlane())
  {
    PlaneResetNavigation();
  }
}

void Load_RTH_Altitude(void)
{
  if (GPS_Parameters.Home.Altitude != STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR))
  {
    GPS_Parameters.Home.Altitude = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
  }
}

void LoadGPSParameters(void)
{
  Load_RTH_Altitude();

  PositionHoldPID.kP = (float)GET_SET[PID_GPS_POSITION].kP / 100.0;
  PositionHoldPID.kI = (float)GET_SET[PID_GPS_POSITION].kI / 100.0;
  PositionHoldPID.GPSFilter.LastInput = 20 * 100;

  PositionHoldRatePID.kP = (float)GET_SET[PID_GPS_POSITION_RATE].kP / 10.0;
  PositionHoldRatePID.kI = (float)GET_SET[PID_GPS_POSITION_RATE].kI / 100.0;
  PositionHoldRatePID.kD = (float)GET_SET[PID_GPS_POSITION_RATE].kD / 100.0;
  PositionHoldRatePID.GPSFilter.IntegralMax = 20 * 100;

  NavigationPID.kP = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kP / 10.0;
  NavigationPID.kI = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kI / 100.0;
  NavigationPID.kD = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kD / 1000.0;
  NavigationPID.GPSFilter.IntegralMax = 20 * 100;
}
