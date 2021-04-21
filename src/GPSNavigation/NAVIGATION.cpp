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
#include "NAVIGATIONGEO.h"
#include "INSNAVIGATION.h"

GPS_Parameters_Struct GPSParameters;

#define TIME_TO_INIT_LAND 100 //MS

void LoadGPSParameters(void)
{
  if (GPSParameters.Home.Altitude != STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR))
  {
    GPSParameters.Home.Altitude = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
  }

  PositionHoldPID.kP = (float)GET_SET[PID_GPS_POSITION].kP / 100.0f;
  PositionHoldPID.kI = (float)GET_SET[PID_GPS_POSITION].kI / 100.0f;
  PositionHoldPID.GPSFilter.IntegralMax = 20 * 100;

  PositionHoldRatePID.kP = (float)GET_SET[PID_GPS_POSITION_RATE].kP / 10.0f;
  PositionHoldRatePID.kI = (float)GET_SET[PID_GPS_POSITION_RATE].kI / 100.0f;
  PositionHoldRatePID.kD = (float)GET_SET[PID_GPS_POSITION_RATE].kD / 100.0f;
  PositionHoldRatePID.GPSFilter.IntegralMax = 20 * 100;

  NavigationPID.kP = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kP / 10.0f;
  NavigationPID.kI = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kI / 100.0f;
  NavigationPID.kD = (float)GET_SET[PID_GPS_NAVIGATION_RATE].kD / 1000.0f;
  NavigationPID.GPSFilter.IntegralMax = 20 * 100;
}

void GPS_Reset_Navigation(void)
{
  GPSParameters.Mode.Navigation = DO_NONE;
  GPSParameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
  GPSParameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
  ResetAllPIDOfGPS();
  if (GetFrameStateOfAirPlane())
  {
    PlaneResetNavigation();
  }
}

void GPS_Process_FlightModes(float DeltaTime)
{
  uint32_t CalculateDistance;
  int32_t CalculateDirection;
  //SAIA DA FUNÇÃO SE O GPS ESTIVER RUIM
  if (Get_GPS_In_Bad_Condition())
  {
    GPSParameters.Navigation.Misc.Velocity.NEDStatus = false;
    return;
  }
  //SAFE PARA RESETAR O HOME-POINT
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    GPSParameters.Home.Marked = false;
  }
  else //RESETA O HOME-POINT AO ARMAR
  {
    if (!GPSParameters.Home.Marked)
    {
      GPSParameters.Home.Coordinates[COORD_LATITUDE] = GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE];
      GPSParameters.Home.Coordinates[COORD_LONGITUDE] = GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
      GPS_Calcule_Longitude_Scaling(GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE]);
      GPSParameters.Navigation.Bearing.InitalTarget = Attitude.EulerAngles.Yaw;
      Altitude_For_Plane = GPSParameters.Navigation.Misc.Get.Altitude;
      GPSParameters.Home.Marked = true;
    }
  }
  //OBTÉM A DECLINAÇÃO MAGNETICA AUTOMATICAMENTE
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !GPSParameters.Declination.Pushed)
  {
    AUTODECLINATION.Set_Initial_Location(GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE]);
    GPSParameters.Declination.PushedCount++;
  }
  //SALVA O VALOR DA DECLINAÇÃO MAGNETICA NA EEPROM
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) &&      //CHECA SE ESTÁ DESARMADO
      AUTODECLINATION.GetDeclination() != 0 &&     //CHECA SE O VALOR É DIFERENTE DE ZERO
      !GPSParameters.Declination.Pushed &&         //CHECA SE A DECLINAÇÃO NÃO FOI PUXADA
      GPSParameters.Declination.PushedCount > 250) //UTILIZA 250 CICLOS DE MAQUINA PARA CALCULAR O VALOR
  {
    STORAGEMANAGER.Write_Float(MAG_DECLINATION_ADDR, AUTODECLINATION.GetDeclination());
    GPSParameters.Declination.Pushed = true;
  }
  GPSParameters.DeltaTime.Navigation = DeltaTime;
  GPSParameters.DeltaTime.Navigation = MIN(GPSParameters.DeltaTime.Navigation, 1.0f);
  GPS_Calcule_Bearing(GPSParameters.Home.Coordinates[COORD_LATITUDE], GPSParameters.Home.Coordinates[COORD_LONGITUDE], &CalculateDirection);
  GPS_Calcule_Distance_To_Home(&CalculateDistance);
  if (!GPSParameters.Home.Marked)
  {
    GPSParameters.Home.Distance = 0;
    GPSParameters.Home.Direction = 0;
  }
  else
  {
    GPSParameters.Home.Direction = CalculateDirection / 100; //FUTURAMENTE PARA O GCS E OSD
    GPSParameters.Home.Distance = CalculateDistance / 100;
  }
  GPS_Calcule_Velocity();
  if (Get_GPS_Only_Flight_Modes_In_Use())
  {
    GPS_Calcule_Bearing(GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPSParameters.Navigation.Bearing.ActualTarget);
    GPS_Calcule_Distance_In_CM(GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPSParameters.Navigation.Coordinates.Distance);

    int16_t CalculateNavigationSpeed = 0;

    if (GetFrameStateOfAirPlane())
    {
      GPSParameters.Mode.Navigation = DO_RTH_ENROUTE;
    }

    switch (GPSParameters.Mode.Navigation)
    {

    case DO_NONE:
      break;

    case DO_POSITION_HOLD:
      break;

    case DO_START_RTH:
      if (GPSParameters.Home.Distance <= JCF_Param.GPS_RTH_Land)
      {
        GPSParameters.Mode.Navigation = DO_LAND_INIT;
        HeadingHoldTarget = GPSParameters.Navigation.Bearing.InitalTarget;
      }
      else if (GetAltitudeReached())
      {
        Set_Next_Point_To_Navigation(GPSParameters.Home.Coordinates[COORD_LATITUDE], GPSParameters.Home.Coordinates[COORD_LONGITUDE]);
        GPSParameters.Mode.Navigation = DO_RTH_ENROUTE;
      }
      break;

    case DO_RTH_ENROUTE:
      CalculateNavigationSpeed = Calculate_Navigation_Speed(JCF_Param.Navigation_Vel);
      GPSCalculateNavigationRate(CalculateNavigationSpeed);
      GPS_Adjust_Heading();
      if ((GPSParameters.Navigation.Coordinates.Distance <= ConvertCMToMeters(JCF_Param.GPS_WP_Radius)) || Point_Reached())
      {
        if (GetFrameStateOfMultirotor())
        {
          GPSParameters.Mode.Navigation = DO_LAND_INIT;
        }
        HeadingHoldTarget = GPSParameters.Navigation.Bearing.InitalTarget;
      }
      break;

    case DO_LAND_INIT:
      Do_RTH_Or_Land_Call_Alt_Hold = true;
      SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
      GPSParameters.DeltaTime.InitLand = SCHEDULERTIME.GetMillis() + TIME_TO_INIT_LAND;
      GPSParameters.Mode.Navigation = DO_LAND_SETTLE;
      break;

    case DO_LAND_SETTLE:
      if (SCHEDULERTIME.GetMillis() >= GPSParameters.DeltaTime.InitLand)
      {
        GPSParameters.Mode.Navigation = DO_LAND_IN_PROGRESS;
      }
      break;

    case DO_LAND_IN_PROGRESS:
      if (GetLanded())
      {
        GPSParameters.Mode.Navigation = DO_LANDED;
      }
      else if (GetGroundDetected())
      {
        GPSParameters.Mode.Navigation = DO_LAND_DETECTED;
      }
      break;

    case DO_LAND_DETECTED:
      if (GetLanded())
      {
        GPSParameters.Mode.Navigation = DO_LANDED;
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
  if (Barometer.INS.Altitude.Estimated < ConvertCMToMeters(GPSParameters.Home.Altitude))
  {
    SetNewAltitudeToHold(ConvertCMToMeters(GPSParameters.Home.Altitude));
  }
  else
  {
    SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
  }
  SetThisPointToPositionHold();
}

void Set_Next_Point_To_Navigation(int32_t Latitude_Destiny, int32_t Longitude_Destiny)
{
  GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE] = Latitude_Destiny;
  GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE] = Longitude_Destiny;
  GPS_Calcule_Longitude_Scaling(Latitude_Destiny);
  Circle_Mode_Update();
  GPS_Calcule_Bearing(GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPSParameters.Navigation.Bearing.ActualTarget);
  GPS_Calcule_Distance_In_CM(GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE], GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE], &GPSParameters.Navigation.Coordinates.Distance);
  INS.Position.Hold[COORD_LATITUDE] = (GPSParameters.Navigation.Coordinates.Destiny[COORD_LATITUDE] - GPSParameters.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
  INS.Position.Hold[COORD_LONGITUDE] = (GPSParameters.Navigation.Coordinates.Destiny[COORD_LONGITUDE] - GPSParameters.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPSParameters.ScaleDownOfLongitude;
  GPSParameters.Navigation.Bearing.TargetPrev = GPSParameters.Navigation.Bearing.ActualTarget;
}

bool Get_Safe_State_To_Apply_Position_Hold(void)
{
  return GPSParameters.Mode.Navigation == DO_LAND_INIT ||
         GPSParameters.Mode.Navigation == DO_LAND_SETTLE ||
         GPSParameters.Mode.Navigation == DO_LAND_IN_PROGRESS ||
         GPSParameters.Mode.Navigation == DO_POSITION_HOLD ||
         GPSParameters.Mode.Navigation == DO_START_RTH;
}
