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

#define NAVTILTCOMPENSATION 20 //RETIRADO DA ARDUPILOT

static void GPS_Calcule_Bearing(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *Bearing);
static void GPS_Calcule_Distance_In_CM(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *CalculateDistance);
static void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance);
static void GPS_Calcule_Velocity(void);
static void GPS_Update_CrossTrackError(void);
void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput);

bool DeclinationPushed = false;
bool GPSHold_CallBaro = false;
bool Home_Point;

uint8_t DeclinationPushedCount = 0;
uint8_t GPS_Flight_Mode;
uint8_t NavigationMode;
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
int32_t GPS_CoordinatesToHold[2];
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
    Set_Initial_Location(GPS_Coordinates_Vector[0], GPS_Coordinates_Vector[1]);
    DeclinationPushedCount++;
  }
  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (Declination() != STORAGEMANAGER.Read_Float(DECLINATION_ADDR) && //VERIFICA SE O VALOR É DIFERENTE
      !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&                         //CHECA SE ESTÁ DESARMADO
      Declination() != 0 &&                                           //CHECA SE O VALOR É DIFERENTE DE ZERO
      !DeclinationPushed &&                                           //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      DeclinationPushedCount > 250)                                   //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(DECLINATION_ADDR, Declination());
    DeclinationPushed = true;
  }
  DeltaTimeGPSNavigation = DeltaTime;
  DeltaTimeGPSNavigation = MIN(DeltaTimeGPSNavigation, 1.0f);
  GPS_Calcule_Bearing(&Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1], &CalculateDirection);
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
    GPS_Calcule_Bearing(&Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Target_Bearing);
    GPS_Calcule_Distance_In_CM(&Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Two_Points_Distance);

    int16_t CalculateNavigationSpeed = 0;

    if (GetFrameStateOfAirPlane())
    {
      NavigationMode = Do_RTH_Enroute;
    }

    switch (NavigationMode)
    {

    case Do_None:
      break;

    case Do_PositionHold:
      break;

    case Do_Start_RTH:
      if (DistanceToHome <= 10)
      {
        NavigationMode = Do_Land_Init;
        HeadingHoldTarget = Navigation_Bearing_RTH;
      }
      else if (GetAltitudeReached())
      {
        Set_Next_Point_To_Navigation(&Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1]);
        NavigationMode = Do_RTH_Enroute;
      }
      break;

    case Do_RTH_Enroute:
      CalculateNavigationSpeed = Calculate_Navigation_Speed(400);
      GPSCalculateNavigationRate(CalculateNavigationSpeed);
      GPS_Adjust_Heading();
      if ((Two_Points_Distance <= 200) || Point_Reached())
      {
        if (GetFrameStateOfMultirotor())
        {
          NavigationMode = Do_Land_Init;
        }
        HeadingHoldTarget = Navigation_Bearing_RTH;
      }
      break;

    case Do_Land_Init:
      Do_GPS_Altitude = true;
      SetAltitudeHold(ALTITUDE.EstimatedAltitude);
      Time_To_Start_The_Land = SCHEDULERTIME.GetMillis() + 100;
      NavigationMode = Do_Land_Settle;
      break;

    case Do_Land_Settle:
      if (SCHEDULERTIME.GetMillis() >= Time_To_Start_The_Land)
      {
        NavigationMode = Do_LandInProgress;
      }
      break;

    case Do_LandInProgress:
      if (GetLanded())
      {
        NavigationMode = Do_Landed;
      }
      else if (GetGroundDetected())
      {
        NavigationMode = Do_Land_Detected;
      }
      break;

    case Do_Land_Detected:
      if (GetLanded())
      {
        NavigationMode = Do_Landed;
      }
      break;

    case Do_Landed:
      DISABLE_STATE(PRIMARY_ARM_DISARM);
      Do_GPS_Altitude = false;
      BEEPER.Play(BEEPER_ACTION_SUCCESS);
      GPS_Reset_Navigation();
      break;
    }
  }
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
  Coordinates_To_Navigation[0] = *Latitude_Destiny;
  Coordinates_To_Navigation[1] = *Longitude_Destiny;
  GPS_Calcule_Longitude_Scaling(*Latitude_Destiny);
  Circle_Mode_Update();
  GPS_Calcule_Bearing(&Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Target_Bearing);
  GPS_Calcule_Distance_In_CM(&Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Two_Points_Distance);
  INS.PositionToHold[0] = (Coordinates_To_Navigation[0] - Stored_Coordinates_Home_Point[0]) * 1.11318845f;
  INS.PositionToHold[1] = (Coordinates_To_Navigation[1] - Stored_Coordinates_Home_Point[1]) * 1.11318845f * ScaleDownOfLongitude;
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
  int32_t Adjust_OffSet_Lat = (*InputLatitude - GPS_Coordinates_Vector[0]) / ScaleDownOfLongitude;
  int32_t Adjust_OffSet_Long = *InputLongitude - GPS_Coordinates_Vector[1];
  *Bearing = 9000 + atan2(-Adjust_OffSet_Lat, Adjust_OffSet_Long) * 5729.57795f;
  if (*Bearing < 0)
  {
    *Bearing += 36000;
  }
}

void GPS_Calcule_Distance_In_CM(int32_t *InputLatitude, int32_t *InputLongitude, int32_t *CalculateDistance)
{
  float DistanceOfLatitude = (float)(GPS_Coordinates_Vector[0] - *InputLatitude);
  float DistanceOfLongitude = (float)(GPS_Coordinates_Vector[1] - *InputLongitude) * ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(SquareFloat(DistanceOfLatitude) + SquareFloat(DistanceOfLongitude)) * 1.11318845f;
}

void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance)
{
  GPSDistanceToHome[0] = (GPS_Coordinates_Vector[0] - Stored_Coordinates_Home_Point[0]) * 1.11318845f;
  GPSDistanceToHome[1] = (GPS_Coordinates_Vector[1] - Stored_Coordinates_Home_Point[1]) * 1.11318845f * ScaleDownOfLongitude;
  *CalculateDistance = SquareRootU32Bits(Square32Bits(GPSDistanceToHome[0]) + Square32Bits(GPSDistanceToHome[1]));
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
    GPSActualSpeed[0] = (float)(GPS_Coordinates_Vector[0] - Last_CoordinatesOfGPS[0]) * DeltaTimeStored;
    GPSActualSpeed[1] = (float)(GPS_Coordinates_Vector[1] - Last_CoordinatesOfGPS[1]) * ScaleDownOfLongitude * DeltaTimeStored;
    GPSActualSpeed[0] = (GPSActualSpeed[0] + Previous_Velocity[0]) / 2;
    GPSActualSpeed[1] = (GPSActualSpeed[1] + Previous_Velocity[1]) / 2;
    Previous_Velocity[0] = GPSActualSpeed[0];
    Previous_Velocity[1] = GPSActualSpeed[1];
  }
  IgnoreFirstPeak = true;
  Last_CoordinatesOfGPS[0] = GPS_Coordinates_Vector[0];
  Last_CoordinatesOfGPS[1] = GPS_Coordinates_Vector[1];
}

void SetThisPointToPositionHold()
{
  INS.PositionToHold[0] = INS.Position_EarthFrame[0] + INS.Velocity_EarthFrame[0] * PositionHoldPID.kI;
  INS.PositionToHold[1] = INS.Position_EarthFrame[1] + INS.Velocity_EarthFrame[1] * PositionHoldPID.kI;
  GPS_CoordinatesToHold[0] = Stored_Coordinates_Home_Point[0] + INS.PositionToHold[0] / 1.11318845f;
  GPS_CoordinatesToHold[1] = Stored_Coordinates_Home_Point[1] + INS.PositionToHold[1] / (1.11318845f * ScaleDownOfLongitude);
}

static void ApplyINSPositionHoldPIDControl(float *DeltaTime)
{
  uint8_t axis;
  for (axis = 0; axis < 2; axis++)
  {
    int32_t INSPositionError = INS.PositionToHold[axis] - INS.Position_EarthFrame[axis];
    int32_t GPSTargetSpeed = GPSGetProportional(INSPositionError, &PositionHoldPID);
    GPSTargetSpeed = Constrain_32Bits(GPSTargetSpeed, -1000, 1000);
    int32_t RateError = GPSTargetSpeed - INS.Velocity_EarthFrame[axis];
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
    GPS_Navigation_Array[0] = 0;
    GPS_Navigation_Array[1] = 0;
    GPSResetPID(&PositionHoldRatePIDArray[0]);
    GPSResetPID(&PositionHoldRatePIDArray[1]);
  }
}

bool NavStateForPosHold()
{
  return NavigationMode == Do_Land_Init || NavigationMode == Do_Land_Settle ||
         NavigationMode == Do_LandInProgress || NavigationMode == Do_PositionHold ||
         NavigationMode == Do_Start_RTH;
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

void Do_Mode_RTH_Now()
{
  GPS_Flight_Mode = GPS_MODE_RTH;
  Do_GPS_Altitude = true;
  if (ALTITUDE.EstimatedAltitude < ConvertCMToMeters(RTH_Altitude))
  {
    SetAltitudeHold(ConvertCMToMeters(RTH_Altitude));
  }
  else
  {
    SetAltitudeHold(ALTITUDE.EstimatedAltitude);
  }
  SetThisPointToPositionHold();
  NavigationMode = Do_Start_RTH;
}

void Reset_Home_Point(void)
{
  Stored_Coordinates_Home_Point[0] = GPS_Coordinates_Vector[0];
  Stored_Coordinates_Home_Point[1] = GPS_Coordinates_Vector[1];
  GPS_Calcule_Longitude_Scaling(GPS_Coordinates_Vector[0]);
  Navigation_Bearing_RTH = ATTITUDE.AngleOut[YAW];
  GPS_Altitude_For_Plane = GPS_Altitude;
  Home_Point = true;
}

void GPS_Reset_Navigation(void)
{
  NavigationMode = Do_None;
  GPS_Navigation_Array[0] = 0;
  GPS_Navigation_Array[1] = 0;
  GPSResetPID(&PositionHoldPIDArray[0]);
  GPSResetPID(&PositionHoldRatePIDArray[0]);
  GPSResetPID(&NavigationPIDArray[0]);
  GPSResetPID(&PositionHoldPIDArray[1]);
  GPSResetPID(&PositionHoldRatePIDArray[1]);
  GPSResetPID(&NavigationPIDArray[1]);
  if (GetFrameStateOfAirPlane())
  {
    PlaneResetNavigation();
  }
}

void LoadGPSParameters(void)
{
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

void RTH_Altitude_EEPROM()
{
  if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 0)
  {
    RTH_Altitude = 10;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 1)
  {
    RTH_Altitude = 15;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 2)
  {
    RTH_Altitude = 20;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 3)
  {
    RTH_Altitude = 25;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 4)
  {
    RTH_Altitude = 30;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 5)
  {
    RTH_Altitude = 35;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 6)
  {
    RTH_Altitude = 40;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 7)
  {
    RTH_Altitude = 45;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 8)
  {
    RTH_Altitude = 50;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 9)
  {
    RTH_Altitude = 55;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 10)
  {
    RTH_Altitude = 60;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 11)
  {
    RTH_Altitude = 65;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 12)
  {
    RTH_Altitude = 70;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 13)
  {
    RTH_Altitude = 75;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 14)
  {
    RTH_Altitude = 80;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 15)
  {
    RTH_Altitude = 85;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 16)
  {
    RTH_Altitude = 90;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 17)
  {
    RTH_Altitude = 95;
  }
  else if (STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR) == 18)
  {
    RTH_Altitude = 100;
  }
}
