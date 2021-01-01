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

#include "MULTIROTORNAVIGATION.h"
#include "Common/VARIABLES.h"
#include "PID/PIDPARAMS.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Declination/AUTODECLINATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"
#include "PID/GPSPID.h"
#include "BAR/BAR.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"

#define NavTiltCompensation 20 //RETIRADO DA ARDUPILOT

static void GPS_Calcule_Bearing(int32_t *Latitude_One, int32_t *Longitude_One, int32_t *Latitude_Two, int32_t *Longitude_Two, int32_t *bearing);
static void GPS_Calcule_Distance_In_CM(int32_t *Latitude_One, int32_t *Longitude_One, int32_t *Latitude_Two, int32_t *Longitude_Two, uint32_t *CalculateDistance);
static void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance);
static void GPS_Calcule_Velocity(void);
static void GPS_Update_CrossTrackError(void);
void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput);

bool DeclinationPushed = false;

uint8_t DeclinationPushedCount = 0;

static float DeltaTimeGPSNavigation;
float ScaleDownOfLongitude = 1.0f;

static uint16_t Coordinates_Navigation_Speed;
int16_t GPSActualSpeed[2] = {0, 0};
static int16_t Crosstrack_Error;
static int16_t GPS_Rate_Error[2];
static int16_t Navigation_Bearing_RTH;

uint32_t Two_Points_Distance;
int32_t Target_Bearing;
int32_t GPSDistanceToHome[2];
int32_t INSPositionToHold[2];
int32_t Original_Target_Bearing;
int32_t Coordinates_To_Navigation[2];
static int32_t Coordinates_From_Navigation[2];

void GPS_Process_FlightModes(void)
{
  uint32_t CalculateDistance;
  int32_t CalculateDirection;
  //SAIA DA FUNÇÃO SE O NÚMERO DE SATELITES FOR MENOR QUE 5
  if (GPS_NumberOfSatellites < 5 && !GPS_3DFIX)
  {
    return;
  }
  //SAFE PARA RESETAR O HOME-POINT
  if (!COMMAND_ARM_DISARM)
  {
    Home_Point = false;
  }
  //RESETA O HOME-POINT AO ARMAR
  if (!Home_Point && COMMAND_ARM_DISARM)
  {
    Reset_Home_Point();
  }
  //OBTÉM A DECLINAÇÃO MAGNETICA AUTOMATICAMENTE
  if (!COMMAND_ARM_DISARM && !DeclinationPushed)
  {
    Set_Initial_Location(GPS_Coordinates_Vector[0], GPS_Coordinates_Vector[1]);
    DeclinationPushedCount++;
  }
  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (Declination() != STORAGEMANAGER.Read_Float(DECLINATION_ADDR) && //VERIFICA SE O VALOR É DIFERENTE
      !COMMAND_ARM_DISARM &&                                          //CHECA SE ESTÁ DESARMADO
      Declination() != 0 &&                                           //CHECA SE O VALOR É DIFERENTE DE ZERO
      !DeclinationPushed &&                                           //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      DeclinationPushedCount > 250)                                   //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(DECLINATION_ADDR, Declination());
    DeclinationPushed = true;
  }
  uint32_t ActualCurrentTime = AVRTIME.SchedulerMillis();
  static uint32_t StoredCurrentTime;
  DeltaTimeGPSNavigation = (ActualCurrentTime - StoredCurrentTime) * 1e-3f;
  StoredCurrentTime = ActualCurrentTime;
  DeltaTimeGPSNavigation = MIN_FLOAT(DeltaTimeGPSNavigation, 1.0);
  GPS_Calcule_Bearing(&GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1], &Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1], &CalculateDirection);
  DirectionToHome = CalculateDirection / 100;
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
    GPS_Calcule_Bearing(&GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1], &Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Target_Bearing);
    GPS_Calcule_Distance_In_CM(&GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1], &Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Two_Points_Distance);

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
        Set_Points_To_Navigation(&Stored_Coordinates_Home_Point[0], &Stored_Coordinates_Home_Point[1], &GPS_Coordinates_Vector[0], &GPS_Coordinates_Vector[1]);
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
      SetAltitudeHold(ALTITUDE.EstimateAltitude);
      Time_To_Start_The_Land = AVRTIME.SchedulerMillis() + 100;
      NavigationMode = Do_Land_Settle;
      break;

    case Do_Land_Settle:
      if (AVRTIME.SchedulerMillis() >= Time_To_Start_The_Land)
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
      COMMAND_ARM_DISARM = false;
      Do_GPS_Altitude = false;
      BEEPER.Play(BEEPER_ACTION_SUCCESS);
      GPS_Reset_Navigation();
      break;
    }
  }
}

void GPS_Adjust_Heading()
{
  HeadingHoldTarget = WRap_180(Target_Bearing) / 100;
}

void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput)
{
  ScaleDownOfLongitude = cos(LatitudeVectorInput * 1.0e-7f * 0.01745329251f);
}

void Set_Points_To_Navigation(int32_t *Latitude_Destiny, int32_t *Longitude_Destiny,
                              int32_t *Latitude_Actual, int32_t *Longitude_Actual)
{
  Coordinates_To_Navigation[0] = *Latitude_Destiny;
  Coordinates_To_Navigation[1] = *Longitude_Destiny;
  Coordinates_From_Navigation[0] = *Latitude_Actual;
  Coordinates_From_Navigation[1] = *Longitude_Actual;
  GPS_Calcule_Longitude_Scaling(*Latitude_Destiny);
  Cruise_Mode_Update();
  GPS_Calcule_Bearing(&Coordinates_From_Navigation[0], &Coordinates_From_Navigation[1], &Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Target_Bearing);
  GPS_Calcule_Distance_In_CM(&Coordinates_From_Navigation[0], &Coordinates_From_Navigation[1], &Coordinates_To_Navigation[0], &Coordinates_To_Navigation[1], &Two_Points_Distance);
  INSPositionToHold[0] = (Coordinates_To_Navigation[0] - Stored_Coordinates_Home_Point[0]) * 1.11318845f;
  INSPositionToHold[1] = (Coordinates_To_Navigation[1] - Stored_Coordinates_Home_Point[1]) * 1.11318845f * ScaleDownOfLongitude;
  Coordinates_Navigation_Speed = 100;
  Original_Target_Bearing = Target_Bearing;
}

bool Point_Reached(void)
{
  int32_t TargetCalculed;
  TargetCalculed = Target_Bearing - Original_Target_Bearing;
  TargetCalculed = WRap_180(TargetCalculed);
  return (ABS_32BITS(TargetCalculed) > 10000);
}

void GPS_Calcule_Bearing(int32_t *Latitude_One, int32_t *Longitude_One, int32_t *Latitude_Two, int32_t *Longitude_Two, int32_t *bearing)
{
  int32_t off_x = *Longitude_Two - *Longitude_One;
  int32_t off_y = (*Latitude_Two - *Latitude_One) / ScaleDownOfLongitude;
  *bearing = 9000 + atan2(-off_y, off_x) * 5729.57795f;
  if (*bearing < 0)
    *bearing += 36000;
}

void GPS_Calcule_Distance_In_CM(int32_t *Latitude_One, int32_t *Longitude_One, int32_t *Latitude_Two, int32_t *Longitude_Two, uint32_t *CalculateDistance)
{
  float DistanceOfLatitude = (float)(*Latitude_One - *Latitude_Two);
  float DistanceOfLongitude = (float)(*Longitude_One - *Longitude_Two) * ScaleDownOfLongitude;
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
    if (DeltaTimeGPSNavigation >= (0.1f - 0.03f) && DeltaTimeGPSNavigation <= (0.1f + 0.03f))
    {
      DeltaTimeStored = 0.1f;
    }
    else if (DeltaTimeGPSNavigation >= (0.2f - 0.03f) && DeltaTimeGPSNavigation <= (0.2f + 0.03f))
    {
      DeltaTimeStored = 0.2f;
    }
    else
    {
      DeltaTimeStored = DeltaTimeGPSNavigation;
    }
    DeltaTimeStored = 1.0 / DeltaTimeStored;
    GPSActualSpeed[1] = (float)(GPS_Coordinates_Vector[1] - Last_CoordinatesOfGPS[1]) * ScaleDownOfLongitude * DeltaTimeStored;
    GPSActualSpeed[0] = (float)(GPS_Coordinates_Vector[0] - Last_CoordinatesOfGPS[0]) * DeltaTimeStored;
    GPSActualSpeed[1] = (GPSActualSpeed[1] + Previous_Velocity[1]) / 2;
    GPSActualSpeed[0] = (GPSActualSpeed[0] + Previous_Velocity[0]) / 2;
    Previous_Velocity[1] = GPSActualSpeed[1];
    Previous_Velocity[0] = GPSActualSpeed[0];
  }
  IgnoreFirstPeak = true;
  Last_CoordinatesOfGPS[1] = GPS_Coordinates_Vector[1];
  Last_CoordinatesOfGPS[0] = GPS_Coordinates_Vector[0];
}

void SetThisPointToPositionHold()
{
  INSPositionToHold[0] = INS.Position_EarthFrame[0] + INS.Velocity_EarthFrame[0] * PositionHoldPID.kI;
  INSPositionToHold[1] = INS.Position_EarthFrame[1] + INS.Velocity_EarthFrame[1] * PositionHoldPID.kI;
  GPS_CoordinatesToHold[0] = Stored_Coordinates_Home_Point[0] + INSPositionToHold[0] / 1.11318845f;
  GPS_CoordinatesToHold[1] = Stored_Coordinates_Home_Point[1] + INSPositionToHold[1] / (1.11318845f * ScaleDownOfLongitude);
}

static void ApplyINSPositionHoldPIDControl(float *DeltaTime)
{
  uint8_t axis;
  for (axis = 0; axis < 2; axis++)
  {
    int32_t positionError = INSPositionToHold[axis] - INS.Position_EarthFrame[axis];
    int32_t targetSpeed = GPSGetProportional(positionError, &PositionHoldPID);
    targetSpeed = Constrain_32Bits(targetSpeed, -1000, 1000);
    int32_t rateError = targetSpeed - INS.Velocity_EarthFrame[axis];
    rateError = Constrain_32Bits(rateError, -1000, 1000);
    GPS_Navigation_Array[axis] = GPSGetProportional(rateError, &PositionHoldRatePID) + GPSGetIntegral(rateError, DeltaTime, &PositionHoldRatePIDArray[axis], &PositionHoldRatePID);
    GPS_Navigation_Array[axis] -= Constrain_16Bits((INS.AccelerationEarthFrame_Filtered[axis] * PositionHoldRatePID.kD), -2000, 2000);
    GPS_Navigation_Array[axis] = Constrain_16Bits(GPS_Navigation_Array[axis], -3000, 3000);
    NavigationPIDArray[axis].Integrator = PositionHoldRatePIDArray[axis].Integrator;
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
    GPSResetPID(&PositionHoldRatePIDArray[0]);
    GPS_Navigation_Array[1] = 0;
    GPSResetPID(&PositionHoldRatePIDArray[1]);
  }
}

bool NavStateForPosHold()
{
  return NavigationMode == Do_Land_Init || NavigationMode == Do_Land_Settle || NavigationMode == Do_LandInProgress || NavigationMode == Do_PositionHold || NavigationMode == Do_Start_RTH;
}

void GPSCalculateNavigationRate(uint16_t Maximum_Velocity)
{
  uint8_t axis;
  float Trigonometry[2];
  float NavCompensation;
  int32_t Target_Speed[2];
  GPS_Update_CrossTrackError();
  int16_t Cross_Speed = Crosstrack_Error * ((.4 * 100) / 100.0);
  Cross_Speed = Constrain_16Bits(Cross_Speed, -200, 200);
  Cross_Speed = -Cross_Speed;
  float TargetCalculed = (9000L - Target_Bearing) * 0.000174532925f;
  Trigonometry[1] = cos(TargetCalculed);
  Trigonometry[0] = sin(TargetCalculed);
  Target_Speed[1] = Maximum_Velocity * Trigonometry[1] - Cross_Speed * Trigonometry[0];
  Target_Speed[0] = Cross_Speed * Trigonometry[1] + Maximum_Velocity * Trigonometry[0];
  for (axis = 0; axis < 2; axis++)
  {
    GPS_Rate_Error[axis] = Target_Speed[axis] - GPSActualSpeed[axis];
    GPS_Rate_Error[axis] = Constrain_16Bits(GPS_Rate_Error[axis], -1000, 1000);
    GPS_Navigation_Array[axis] = GPSGetProportional(GPS_Rate_Error[axis], &NavigationPID) +
                                 GPSGetIntegral(GPS_Rate_Error[axis], &DeltaTimeGPSNavigation, &NavigationPIDArray[axis], &NavigationPID) +
                                 GPSGetDerivative(GPS_Rate_Error[axis], &DeltaTimeGPSNavigation, &NavigationPIDArray[axis], &NavigationPID);
    if (NavTiltCompensation != 0)
    {
      NavCompensation = Target_Speed[axis] * Target_Speed[axis] * ((float)NavTiltCompensation * 0.0001f);
      if (Target_Speed[axis] < 0)
        NavCompensation = -NavCompensation;
    }
    else
      NavCompensation = 0;
    GPS_Navigation_Array[axis] = Constrain_16Bits(GPS_Navigation_Array[axis] + NavCompensation, -3000, 3000);
    PositionHoldRatePIDArray[axis].Integrator = NavigationPIDArray[axis].Integrator;
  }
}

static void GPS_Update_CrossTrackError(void)
{
  float TargetCalculed = (Target_Bearing - Original_Target_Bearing) * 0.000174532925f;
  Crosstrack_Error = sin(TargetCalculed) * Two_Points_Distance;
}

uint16_t Calculate_Navigation_Speed(uint16_t Maximum_Velocity)
{
  Maximum_Velocity = MIN_U16BITS(Maximum_Velocity, Two_Points_Distance);
  Maximum_Velocity = MAX_U16BITS(Maximum_Velocity, 100);
  if (Maximum_Velocity > Coordinates_Navigation_Speed)
  {
    Coordinates_Navigation_Speed += (int16_t)(100.0 * DeltaTimeGPSNavigation);
    Maximum_Velocity = Coordinates_Navigation_Speed;
  }
  return Maximum_Velocity;
}

void Do_Mode_RTH_Now()
{
  GPS_Flight_Mode = GPS_MODE_RTH;
  Do_GPS_Altitude = true;
  if (ALTITUDE.EstimateAltitude < RTH_Altitude * 100)
  {
    SetAltitudeHold(RTH_Altitude * 100);
  }
  else
  {
    SetAltitudeHold(ALTITUDE.EstimateAltitude);
  }
  SetThisPointToPositionHold();
  NavigationMode = Do_Start_RTH;
}

void Reset_Home_Point(void)
{
  if (GPS_3DFIX && GPS_NumberOfSatellites >= 5)
  {
    Stored_Coordinates_Home_Point[0] = GPS_Coordinates_Vector[0];
    Stored_Coordinates_Home_Point[1] = GPS_Coordinates_Vector[1];
    GPS_Calcule_Longitude_Scaling(GPS_Coordinates_Vector[0]);
    Navigation_Bearing_RTH = ATTITUDE.CalculedHeading;
    GPS_Altitude_For_Plane = GPS_Altitude;
    Home_Point = true;
  }
}

void GPS_Reset_Navigation(void)
{
  GPS_Navigation_Array[0] = 0;
  GPSResetPID(&PositionHoldPIDArray[0]);
  GPSResetPID(&PositionHoldRatePIDArray[0]);
  GPSResetPID(&NavigationPIDArray[0]);
  GPS_Navigation_Array[1] = 0;
  GPSResetPID(&PositionHoldPIDArray[1]);
  GPSResetPID(&PositionHoldRatePIDArray[1]);
  GPSResetPID(&NavigationPIDArray[1]);
  NavigationMode = Do_None;
  if (GetFrameStateOfAirPlane())
    PlaneResetNavigation();
}

void LoadGPSParameters(void)
{
  PositionHoldPID.kP = (float)PID[PIDGPSPOSITION].ProportionalVector / 100.0;
  PositionHoldPID.kI = (float)PID[PIDGPSPOSITION].IntegratorVector / 100.0;
  PositionHoldPID.IntegratorMax = 20 * 100;
  PositionHoldRatePID.kP = (float)PID[PIDGPSPOSITIONRATE].ProportionalVector / 10.0;
  PositionHoldRatePID.kI = (float)PID[PIDGPSPOSITIONRATE].IntegratorVector / 100.0;
  PositionHoldRatePID.kD = (float)PID[PIDGPSPOSITIONRATE].DerivativeVector / 100.0;
  PositionHoldRatePID.IntegratorMax = 20 * 100;
  NavigationPID.kP = (float)PID[PIDGPSNAVIGATIONRATE].ProportionalVector / 10.0;
  NavigationPID.kI = (float)PID[PIDGPSNAVIGATIONRATE].IntegratorVector / 100.0;
  NavigationPID.kD = (float)PID[PIDGPSNAVIGATIONRATE].DerivativeVector / 1000.0;
  NavigationPID.IntegratorMax = 20 * 100;
}

int32_t WRap_180(int32_t AngleInput)
{
  if (AngleInput > 18000)
    AngleInput -= 36000;
  if (AngleInput < -18000)
    AngleInput += 36000;
  return AngleInput;
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
