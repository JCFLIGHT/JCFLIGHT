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

#define NAVTILTCOMPENSATION 20 //RETIRADO DA ARDUPILOT

static void GPS_Calcule_Bearing(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *Bearing);
static void GPS_Calcule_Distance_In_CM(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *CalculateDistance);
static void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance);
static void GPS_Calcule_Velocity(void);
static void GPS_Update_CrossTrackError(void);
void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput);

bool DeclinationPushed = false;
bool Home_Point;

uint8_t DeclinationPushedCount = 0;
uint8_t GPS_Flight_Mode;
uint8_t GPS_Navigation_Mode;
uint8_t RTH_Altitude;

static float DeltaTimeGPSNavigation;
float ScaleDownOfLongitude = 1.0f;

int16_t GPSActualSpeed[2] = {0, 0};
int16_t GPS_Navigation_Array[2];
int16_t DirectionToHome;

static int16_t Coordinates_Navigation_Speed;
static int16_t Crosstrack_Error;
static int16_t GPS_Rate_Error[2];
static int16_t Navigation_Bearing_RTH;

uint16_t DistanceToHome;

int32_t Stored_Coordinates_Home_Point[2];
int32_t Two_Points_Distance;
int32_t Target_Bearing;
int32_t GPSDistanceToHome[2];
int32_t Original_Target_Bearing;
int32_t Coordinates_To_Navigation[2];

uint32_t Time_To_Start_The_Land;

void GPS_Process_FlightModes(float DeltaTime)
{
  uint32_t CalculateDistance;
  int32_t CalculateDirection;
  //SAIA DA FUNÇÃO SE O GPS ESTIVER RUIM
  if (Get_GPS_In_Bad_Condition())
  {
    ValidNED = false;
    return;
  }
  //SAFE PARA RESETAR O HOME-POINT
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Home_Point = false;
  }
  //RESETA O HOME-POINT AO ARMAR
  if (!Home_Point && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Reset_Home_Point();
  }
  //OBTÉM A DECLINAÇÃO MAGNETICA AUTOMATICAMENTE
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !DeclinationPushed)
  {
    AUTODECLINATION.Set_Initial_Location(GPS_Coordinates_Vector[COORD_LATITUDE], GPS_Coordinates_Vector[COORD_LONGITUDE]);
    DeclinationPushedCount++;
  }
  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (AUTODECLINATION.GetDeclination() != STORAGEMANAGER.Read_Float(DECLINATION_ADDR) && //VERIFICA SE O VALOR É DIFERENTE
      !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&                                            //CHECA SE ESTÁ DESARMADO
      AUTODECLINATION.GetDeclination() != 0 &&                                           //CHECA SE O VALOR É DIFERENTE DE ZERO
      !DeclinationPushed &&                                                              //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      DeclinationPushedCount > 250)                                                      //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(DECLINATION_ADDR, AUTODECLINATION.GetDeclination());
    DeclinationPushed = true;
  }
  DeltaTimeGPSNavigation = DeltaTime;
  DeltaTimeGPSNavigation = MIN(DeltaTimeGPSNavigation, 1.0f);
  GPS_Calcule_Bearing(&Stored_Coordinates_Home_Point[COORD_LATITUDE], &Stored_Coordinates_Home_Point[COORD_LONGITUDE], &CalculateDirection);
  DirectionToHome = CalculateDirection / 100; //FUTURAMENTE PARA O GCS
  GPS_Calcule_Distance_To_Home(&CalculateDistance);
  DistanceToHome = CalculateDistance / 100;
  if (!Home_Point)
  {
    DistanceToHome = 0;
    DirectionToHome = 0;
  }
  GPS_Calcule_Velocity();
  if (GPS_Flight_Mode != GPS_MODE_NONE)
  {
    GPS_Calcule_Bearing(&Coordinates_To_Navigation[COORD_LATITUDE], &Coordinates_To_Navigation[COORD_LONGITUDE], &Target_Bearing);
    GPS_Calcule_Distance_In_CM(&Coordinates_To_Navigation[COORD_LATITUDE], &Coordinates_To_Navigation[COORD_LONGITUDE], &Two_Points_Distance);

    int16_t CalculateNavigationSpeed = 0;

    if (GetFrameStateOfAirPlane())
    {
      GPS_Navigation_Mode = DO_RTH_ENROUTE;
    }

    switch (GPS_Navigation_Mode)
    {

    case DO_NONE:
      break;

    case DO_POSITION_HOLD:
      break;

    case DO_START_RTH:
      if (DistanceToHome <= JCF_Param.GPS_RTH_Land)
      {
        GPS_Navigation_Mode = DO_LAND_INIT;
        HeadingHoldTarget = Navigation_Bearing_RTH;
      }
      else if (GetAltitudeReached())
      {
        Set_Next_Point_To_Navigation(&Stored_Coordinates_Home_Point[COORD_LATITUDE], &Stored_Coordinates_Home_Point[COORD_LONGITUDE]);
        GPS_Navigation_Mode = DO_RTH_ENROUTE;
      }
      break;

    case DO_RTH_ENROUTE:
      CalculateNavigationSpeed = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
      GPSCalculateNavigationRate(CalculateNavigationSpeed);
      GPS_Adjust_Heading();
      if ((Two_Points_Distance <= (JCF_Param.GPS_WP_Radius * 100)) || Point_Reached())
      {
        if (GetFrameStateOfMultirotor())
        {
          GPS_Navigation_Mode = DO_LAND_INIT;
        }
        HeadingHoldTarget = Navigation_Bearing_RTH;
      }
      break;

    case DO_LAND_INIT:
      Do_RTH_Or_Land_Call_Alt_Hold = true;
      SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
      Time_To_Start_The_Land = SCHEDULERTIME.GetMillis() + 100;
      GPS_Navigation_Mode = DO_LAND_SETTLE;
      break;

    case DO_LAND_SETTLE:
      if (SCHEDULERTIME.GetMillis() >= Time_To_Start_The_Land)
      {
        GPS_Navigation_Mode = DO_LAND_IN_PROGRESS;
      }
      break;

    case DO_LAND_IN_PROGRESS:
      if (GetLanded())
      {
        GPS_Navigation_Mode = DO_LANDED;
      }
      else if (GetGroundDetected())
      {
        GPS_Navigation_Mode = DO_LAND_DETECTED;
      }
      break;

    case DO_LAND_DETECTED:
      if (GetLanded())
      {
        GPS_Navigation_Mode = DO_LANDED;
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
  if (Barometer.INS.Altitude.Estimated < ConvertCMToMeters(RTH_Altitude))
  {
    SetAltitudeToHold(ConvertCMToMeters(RTH_Altitude));
  }
  else
  {
    SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
  }
  SetThisPointToPositionHold();
}

void GPS_Adjust_Heading()
{
  HeadingHoldTarget = WRap_18000(Target_Bearing) / 100;
}

void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput)
{
  ScaleDownOfLongitude = Fast_Cosine(ConvertToRadians((ConvertCoordinateToFloatingPoint(LatitudeVectorInput))));
}

void Set_Next_Point_To_Navigation(int32_t *Latitude_Destiny, int32_t *Longitude_Destiny)
{
  Coordinates_To_Navigation[COORD_LATITUDE] = *Latitude_Destiny;
  Coordinates_To_Navigation[COORD_LONGITUDE] = *Longitude_Destiny;
  GPS_Calcule_Longitude_Scaling(*Latitude_Destiny);
  Circle_Mode_Update();
  GPS_Calcule_Bearing(&Coordinates_To_Navigation[COORD_LATITUDE], &Coordinates_To_Navigation[COORD_LONGITUDE], &Target_Bearing);
  GPS_Calcule_Distance_In_CM(&Coordinates_To_Navigation[COORD_LATITUDE], &Coordinates_To_Navigation[COORD_LONGITUDE], &Two_Points_Distance);
  INS.Position.Hold[COORD_LATITUDE] = (Coordinates_To_Navigation[COORD_LATITUDE] - Stored_Coordinates_Home_Point[COORD_LATITUDE]) * 1.11318845f;
  INS.Position.Hold[COORD_LONGITUDE] = (Coordinates_To_Navigation[COORD_LONGITUDE] - Stored_Coordinates_Home_Point[COORD_LONGITUDE]) * 1.11318845f * ScaleDownOfLongitude;
  Coordinates_Navigation_Speed = 100;
  Original_Target_Bearing = Target_Bearing;
}

bool Point_Reached(void)
{
  int32_t TargetCalculed;
  TargetCalculed = Target_Bearing - Original_Target_Bearing;
  TargetCalculed = WRap_18000(TargetCalculed);
  return (ABS(TargetCalculed) > 10000);
}

void GPS_Calcule_Bearing(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *Bearing)
{
  int32_t Adjust_OffSet_Lat = (*InputLatitude - GPS_Coordinates_Vector[COORD_LATITUDE]) / ScaleDownOfLongitude;
  int32_t Adjust_OffSet_Long = *InputLongitude - GPS_Coordinates_Vector[COORD_LONGITUDE];
  *Bearing = 9000 + atan2(-Adjust_OffSet_Lat, Adjust_OffSet_Long) * 5729.57795f;
  if (*Bearing < 0)
  {
    *Bearing += 36000;
  }
}

void GPS_Calcule_Distance_In_CM(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *CalculateDistance)
{
  float DistanceOfLatitude = (float)(GPS_Coordinates_Vector[COORD_LATITUDE] - *InputLatitude);
  float DistanceOfLongitude = (float)(GPS_Coordinates_Vector[COORD_LONGITUDE] - *InputLongitude) * ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(SquareFloat(DistanceOfLatitude) + SquareFloat(DistanceOfLongitude)) * 1.11318845f;
}

void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance)
{
  GPSDistanceToHome[COORD_LATITUDE] = (GPS_Coordinates_Vector[COORD_LATITUDE] - Stored_Coordinates_Home_Point[COORD_LATITUDE]) * 1.11318845f;
  GPSDistanceToHome[COORD_LONGITUDE] = (GPS_Coordinates_Vector[COORD_LONGITUDE] - Stored_Coordinates_Home_Point[COORD_LONGITUDE]) * 1.11318845f * ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(Square32Bits(GPSDistanceToHome[COORD_LATITUDE]) + Square32Bits(GPSDistanceToHome[COORD_LONGITUDE]));
}

static void GPS_Calcule_Velocity(void)
{
  static int16_t Previous_Velocity[2] = {0, 0};
  static int32_t Last_CoordinatesOfGPS[2] = {0, 0};
  static bool IgnoreFirstPeak = false;
  if (IgnoreFirstPeak)
  {
    float DeltaTimeStored;
    if (DeltaTimeGPSNavigation >= 0.07f && DeltaTimeGPSNavigation <= 0.13f)
    {
      DeltaTimeStored = 0.1f;
    }
    else if (DeltaTimeGPSNavigation >= 0.17f && DeltaTimeGPSNavigation <= 0.23f)
    {
      DeltaTimeStored = 0.2f;
    }
    else
    {
      DeltaTimeStored = DeltaTimeGPSNavigation;
    }
    DeltaTimeStored = 1.0 / DeltaTimeStored;
    GPSActualSpeed[COORD_LATITUDE] = (float)(GPS_Coordinates_Vector[COORD_LATITUDE] - Last_CoordinatesOfGPS[COORD_LATITUDE]) * DeltaTimeStored;
    GPSActualSpeed[COORD_LONGITUDE] = (float)(GPS_Coordinates_Vector[COORD_LONGITUDE] - Last_CoordinatesOfGPS[COORD_LONGITUDE]) * ScaleDownOfLongitude * DeltaTimeStored;
    GPSActualSpeed[COORD_LATITUDE] = (GPSActualSpeed[COORD_LATITUDE] + Previous_Velocity[COORD_LATITUDE]) / 2;
    GPSActualSpeed[COORD_LONGITUDE] = (GPSActualSpeed[COORD_LONGITUDE] + Previous_Velocity[COORD_LONGITUDE]) / 2;
    Previous_Velocity[COORD_LATITUDE] = GPSActualSpeed[COORD_LATITUDE];
    Previous_Velocity[COORD_LONGITUDE] = GPSActualSpeed[COORD_LONGITUDE];
  }
  IgnoreFirstPeak = true;
  Last_CoordinatesOfGPS[COORD_LATITUDE] = GPS_Coordinates_Vector[COORD_LATITUDE];
  Last_CoordinatesOfGPS[COORD_LONGITUDE] = GPS_Coordinates_Vector[COORD_LONGITUDE];
}

void SetThisPointToPositionHold()
{
  INS.Position.Hold[COORD_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + INS.EarthFrame.Velocity[INS_LATITUDE] * PositionHoldPID.kI;
  INS.Position.Hold[COORD_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + INS.EarthFrame.Velocity[INS_LONGITUDE] * PositionHoldPID.kI;
}

static void ApplyINSPositionHoldPIDControl(float *DeltaTime)
{
  uint8_t axis;
  for (axis = 0; axis < 2; axis++)
  {
    int32_t INSPositionError = INS.Position.Hold[axis] - INS.EarthFrame.Position[axis];
    int32_t GPSTargetSpeed = GPSGetProportional(INSPositionError, &PositionHoldPID);
    GPSTargetSpeed = Constrain_32Bits(GPSTargetSpeed, -1000, 1000);
    int32_t RateError = GPSTargetSpeed - INS.EarthFrame.Velocity[axis];
    RateError = Constrain_32Bits(RateError, -1000, 1000);
    GPS_Navigation_Array[axis] = GPSGetProportional(RateError, &PositionHoldRatePID) + GPSGetIntegral(RateError, DeltaTime, &PositionHoldRatePIDArray[axis], &PositionHoldRatePID);
    GPS_Navigation_Array[axis] -= Constrain_16Bits((INS.AccelerationEarthFrame_Filtered[axis] * PositionHoldRatePID.kD), -2000, 2000);
    GPS_Navigation_Array[axis] = Constrain_16Bits(GPS_Navigation_Array[axis], -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValueVector), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValueVector));
    NavigationPIDArray[axis].Integral = PositionHoldRatePIDArray[axis].Integral;
  }
}

void ApplyPosHoldPIDControl(float *DeltaTime)
{
  if (!GetTakeOffInProgress() && !GetGroundDetectedFor100ms())
  {
    ApplyINSPositionHoldPIDControl(DeltaTime);
  }
  else
  {
    GPS_Navigation_Array[COORD_LATITUDE] = 0;
    GPS_Navigation_Array[COORD_LONGITUDE] = 0;
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LATITUDE]);
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LONGITUDE]);
  }
}

bool NavStateForPosHold(void)
{
  return GPS_Navigation_Mode == DO_LAND_INIT || GPS_Navigation_Mode == DO_LAND_SETTLE ||
         GPS_Navigation_Mode == DO_LAND_IN_PROGRESS || GPS_Navigation_Mode == DO_POSITION_HOLD ||
         GPS_Navigation_Mode == DO_START_RTH;
}

void GPSCalculateNavigationRate(uint16_t Maximum_Velocity)
{
#define CROSSTRACK_ERROR 0.4 //TESTAR COM 1 FUTURAMENTE
  uint8_t axis;
  float Trigonometry[2];
  float NavCompensation;
  int32_t Target_Speed[2];
  GPS_Update_CrossTrackError();
  int16_t Cross_Speed = Crosstrack_Error * CROSSTRACK_ERROR;
  Cross_Speed = Constrain_16Bits(Cross_Speed, -200, 200);
  Cross_Speed = -Cross_Speed;
  float TargetCalculed = (9000L - Target_Bearing) * 0.000174532925f;
  Trigonometry[0] = Fast_Sine(TargetCalculed);
  Trigonometry[1] = Fast_Cosine(TargetCalculed);
  Target_Speed[0] = Cross_Speed * Trigonometry[1] + Maximum_Velocity * Trigonometry[0];
  Target_Speed[1] = Maximum_Velocity * Trigonometry[1] - Cross_Speed * Trigonometry[0];
  for (axis = 0; axis < 2; axis++)
  {
    GPS_Rate_Error[axis] = Target_Speed[axis] - GPSActualSpeed[axis];
    GPS_Rate_Error[axis] = Constrain_16Bits(GPS_Rate_Error[axis], -1000, 1000);
    GPS_Navigation_Array[axis] = GPSGetProportional(GPS_Rate_Error[axis], &NavigationPID) +
                                 GPSGetIntegral(GPS_Rate_Error[axis], &DeltaTimeGPSNavigation, &NavigationPIDArray[axis], &NavigationPID) +
                                 GPSGetDerivative(GPS_Rate_Error[axis], &DeltaTimeGPSNavigation, &NavigationPIDArray[axis], &NavigationPID);

    if (NAVTILTCOMPENSATION != 0)
    {
      NavCompensation = Target_Speed[axis] * Target_Speed[axis] * ((float)NAVTILTCOMPENSATION * 0.0001f);
      if (Target_Speed[axis] < 0)
      {
        NavCompensation = -NavCompensation;
      }
    }
    else
    {
      NavCompensation = 0;
    }
    GPS_Navigation_Array[axis] = Constrain_16Bits(GPS_Navigation_Array[axis] + NavCompensation, -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValueVector), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValueVector));
    PositionHoldRatePIDArray[axis].Integral = NavigationPIDArray[axis].Integral;
  }
}

static void GPS_Update_CrossTrackError(void)
{
  float TargetCalculed = (Target_Bearing - Original_Target_Bearing) * 0.000174532925f;
  Crosstrack_Error = Fast_Sine(TargetCalculed) * Two_Points_Distance;
}

int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity)
{
  Maximum_Velocity = MIN(Maximum_Velocity, Two_Points_Distance);
  Maximum_Velocity = MAX(Maximum_Velocity, 100); //100CM/S ~ 3.6KM/M -> VELOCIDADE MINIMA SUPORTADA
  if (Maximum_Velocity > Coordinates_Navigation_Speed)
  {
    Coordinates_Navigation_Speed += (int16_t)(100.0f * DeltaTimeGPSNavigation);
    Maximum_Velocity = Coordinates_Navigation_Speed;
  }
  return Maximum_Velocity;
}

void Reset_Home_Point(void)
{
  Stored_Coordinates_Home_Point[COORD_LATITUDE] = GPS_Coordinates_Vector[COORD_LATITUDE];
  Stored_Coordinates_Home_Point[COORD_LONGITUDE] = GPS_Coordinates_Vector[COORD_LONGITUDE];
  GPS_Calcule_Longitude_Scaling(GPS_Coordinates_Vector[COORD_LATITUDE]);
  Navigation_Bearing_RTH = ATTITUDE.AngleOut[YAW];
  GPS_Altitude_For_Plane = GPS_Altitude;
  Home_Point = true;
}

void GPS_Reset_Navigation(void)
{
  GPS_Navigation_Mode = DO_NONE;
  GPS_Navigation_Array[COORD_LATITUDE] = 0;
  GPS_Navigation_Array[COORD_LONGITUDE] = 0;
  ResetAllGPSPID();
  if (GetFrameStateOfAirPlane())
  {
    PlaneResetNavigation();
  }
}

void Load_RTH_Altitude(void)
{
  if (RTH_Altitude != STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR))
  {
    RTH_Altitude = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
  }
}

void LoadGPSParameters(void)
{
  Load_RTH_Altitude();

  PositionHoldPID.kP = (float)GET_SET[PID_GPS_POSITION].ProportionalVector / 100.0;
  PositionHoldPID.kI = (float)GET_SET[PID_GPS_POSITION].IntegralVector / 100.0;
  PositionHoldPID.IntegralMax = 20 * 100;

  PositionHoldRatePID.kP = (float)GET_SET[PID_GPS_POSITION_RATE].ProportionalVector / 10.0;
  PositionHoldRatePID.kI = (float)GET_SET[PID_GPS_POSITION_RATE].IntegralVector / 100.0;
  PositionHoldRatePID.kD = (float)GET_SET[PID_GPS_POSITION_RATE].DerivativeVector / 100.0;
  PositionHoldRatePID.IntegralMax = 20 * 100;

  NavigationPID.kP = (float)GET_SET[PID_GPS_NAVIGATION_RATE].ProportionalVector / 10.0;
  NavigationPID.kI = (float)GET_SET[PID_GPS_NAVIGATION_RATE].IntegralVector / 100.0;
  NavigationPID.kD = (float)GET_SET[PID_GPS_NAVIGATION_RATE].DerivativeVector / 1000.0;
  NavigationPID.IntegralMax = 20 * 100;
}
