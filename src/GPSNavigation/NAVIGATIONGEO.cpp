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

#include "NAVIGATIONGEO.h"
#include "NAVIGATION.h"
#include "Math/MATHSUPPORT.h"
#include "Yaw/HEADINGHOLD.h"
#include "Param/PARAM.h"
#include "PID/GPSPID.h"
#include "PID/PIDPARAMS.h"

#ifdef __AVR_ATmega2560__
#define NAVTILTCOMPENSATION 20 //RETIRADO DA ARDUPILOT
#else
#define NAVTILTCOMPENSATION JCF_Param.GPS_TiltCompensation
#endif

#define MIN_NAVIGATION_SPEED 100 //100CM/S ~ 3.6KM/M -> VELOCIDADE MINIMA SUPORTADA
#define CROSSTRACK_ERROR 0.4     //TESTAR COM 1 FUTURAMENTE

void GPS_Adjust_Heading()
{
    HeadingHoldTarget = WRap_18000(GPSParameters.Navigation.Bearing.ActualTarget) / 100;
}

void GPS_Calcule_Bearing(int32_t InputLatitude, int32_t InputLongitude, int32_t *Bearing)
{
    int32_t Adjust_OffSet_Lat = (InputLatitude - GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE]) / GPSParameters.ScaleDownOfLongitude;
    int32_t Adjust_OffSet_Long = InputLongitude - GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
    *Bearing = 9000 + Fast_Atan2(-Adjust_OffSet_Lat, Adjust_OffSet_Long) * 5729.57795f;
    if (*Bearing < 0)
    {
        *Bearing += 36000;
    }
}

void GPS_Calcule_Distance_In_CM(int32_t InputLatitude, int32_t InputLongitude, int32_t *CalculateDistance)
{
    float DistanceOfLatitude = (float)(GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - InputLatitude);
    float DistanceOfLongitude = (float)(GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - InputLongitude) * GPSParameters.ScaleDownOfLongitude;
    *CalculateDistance = SquareRootU32Bits(SquareFloat(DistanceOfLatitude) + SquareFloat(DistanceOfLongitude)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
}

void GPS_Calcule_Distance_To_Home(uint32_t *CalculateDistance)
{
    GPSParameters.Home.INS.Distance[COORD_LATITUDE] = (GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - GPSParameters.Home.Coordinates[COORD_LATITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    GPSParameters.Home.INS.Distance[COORD_LONGITUDE] = (GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - GPSParameters.Home.Coordinates[COORD_LONGITUDE]) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * GPSParameters.ScaleDownOfLongitude;
    *CalculateDistance = SquareRootU32Bits(Square32Bits(GPSParameters.Home.INS.Distance[COORD_LATITUDE]) + Square32Bits(GPSParameters.Home.INS.Distance[COORD_LONGITUDE]));
}

int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity)
{
    static int16_t Coordinates_Navigation_Speed = MIN_NAVIGATION_SPEED;
    Maximum_Velocity = MIN(Maximum_Velocity, GPSParameters.Navigation.Coordinates.Distance);
    Maximum_Velocity = MAX(Maximum_Velocity, MIN_NAVIGATION_SPEED);
    if (Maximum_Velocity > Coordinates_Navigation_Speed)
    {
        Coordinates_Navigation_Speed += (int16_t)(100.0f * GPSParameters.DeltaTime.Navigation);
        Maximum_Velocity = Coordinates_Navigation_Speed;
    }
    return Maximum_Velocity;
}

void GPS_Calcule_Velocity(void)
{
    static int16_t Previous_Velocity[2] = {0, 0};
    static int32_t Last_CoordinatesOfGPS[2] = {0, 0};
    static bool IgnoreFirstPeak = false;
    if (IgnoreFirstPeak)
    {
        float DeltaTimeStored;
        if (GPSParameters.DeltaTime.Navigation >= 0.07f && GPSParameters.DeltaTime.Navigation <= 0.13f)
        {
            DeltaTimeStored = 0.1f;
        }
        else if (GPSParameters.DeltaTime.Navigation >= 0.17f && GPSParameters.DeltaTime.Navigation <= 0.23f)
        {
            DeltaTimeStored = 0.2f;
        }
        else
        {
            DeltaTimeStored = GPSParameters.DeltaTime.Navigation;
        }
        DeltaTimeStored = 1.0f / DeltaTimeStored;
        GPSParameters.Navigation.Speed[COORD_LATITUDE] = (float)(GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE] - Last_CoordinatesOfGPS[COORD_LATITUDE]) * DeltaTimeStored;
        GPSParameters.Navigation.Speed[COORD_LONGITUDE] = (float)(GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE] - Last_CoordinatesOfGPS[COORD_LONGITUDE]) * GPSParameters.ScaleDownOfLongitude * DeltaTimeStored;
        GPSParameters.Navigation.Speed[COORD_LATITUDE] = (GPSParameters.Navigation.Speed[COORD_LATITUDE] + Previous_Velocity[COORD_LATITUDE]) / 2;
        GPSParameters.Navigation.Speed[COORD_LONGITUDE] = (GPSParameters.Navigation.Speed[COORD_LONGITUDE] + Previous_Velocity[COORD_LONGITUDE]) / 2;
        Previous_Velocity[COORD_LATITUDE] = GPSParameters.Navigation.Speed[COORD_LATITUDE];
        Previous_Velocity[COORD_LONGITUDE] = GPSParameters.Navigation.Speed[COORD_LONGITUDE];
    }
    IgnoreFirstPeak = true;
    Last_CoordinatesOfGPS[COORD_LATITUDE] = GPSParameters.Navigation.Coordinates.Actual[COORD_LATITUDE];
    Last_CoordinatesOfGPS[COORD_LONGITUDE] = GPSParameters.Navigation.Coordinates.Actual[COORD_LONGITUDE];
}

bool Point_Reached(void)
{
    int32_t TargetCalculed;
    TargetCalculed = GPSParameters.Navigation.Bearing.ActualTarget - GPSParameters.Navigation.Bearing.TargetPrev;
    TargetCalculed = WRap_18000(TargetCalculed);
    return (ABS(TargetCalculed) > 10000);
}

void GPS_Calcule_Longitude_Scaling(int32_t LatitudeVectorInput)
{
    GPSParameters.ScaleDownOfLongitude = Constrain_Float(Fast_Cosine(ConvertToRadians((ConvertCoordinateToFloatingPoint(LatitudeVectorInput)))), 0.01f, 1.0f);
}

int16_t GPS_Update_CrossTrackError(void)
{
    float TargetCalculed = (GPSParameters.Navigation.Bearing.ActualTarget - GPSParameters.Navigation.Bearing.TargetPrev) * 0.000174532925f;
    return Fast_Sine(TargetCalculed) * GPSParameters.Navigation.Coordinates.Distance;
}

void GPSCalculateNavigationRate(uint16_t Maximum_Velocity)
{
    float Trigonometry[2];
    float NavCompensation = 0.0f;
    int32_t Target_Speed[2];
    int16_t Cross_Speed = GPS_Update_CrossTrackError() * CROSSTRACK_ERROR;
    Cross_Speed = Constrain_16Bits(Cross_Speed, -200, 200);
    Cross_Speed = -Cross_Speed;
    float TargetCalculed = (9000L - GPSParameters.Navigation.Bearing.ActualTarget) * 0.000174532925f;
    Trigonometry[COORD_LATITUDE] = Fast_Sine(TargetCalculed);
    Trigonometry[COORD_LONGITUDE] = Fast_Cosine(TargetCalculed);
    Target_Speed[COORD_LATITUDE] = Cross_Speed * Trigonometry[COORD_LONGITUDE] + Maximum_Velocity * Trigonometry[COORD_LATITUDE];
    Target_Speed[COORD_LONGITUDE] = Maximum_Velocity * Trigonometry[COORD_LONGITUDE] - Cross_Speed * Trigonometry[COORD_LATITUDE];
    for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
    {
        GPSParameters.Navigation.RateError[IndexCount] = Target_Speed[IndexCount] - GPSParameters.Navigation.Speed[IndexCount];
        GPSParameters.Navigation.RateError[IndexCount] = Constrain_16Bits(GPSParameters.Navigation.RateError[IndexCount], -1000, 1000);
        GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(GPSParameters.Navigation.RateError[IndexCount], &NavigationPID) +
                                                                    GPSGetIntegral(GPSParameters.Navigation.RateError[IndexCount], GPSParameters.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID) +
                                                                    GPSGetDerivative(GPSParameters.Navigation.RateError[IndexCount], GPSParameters.DeltaTime.Navigation, &NavigationPIDArray[IndexCount], &NavigationPID);

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
        GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] + NavCompensation, -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue));
        PositionHoldRatePIDArray[IndexCount].GPSFilter.IntegralSum = NavigationPIDArray[IndexCount].GPSFilter.IntegralSum;
    }
}