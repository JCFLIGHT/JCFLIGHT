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

#include "WINDESTIMATOR.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSUBLOX.h"
#include "GPS/GPSSTATES.h"
#include "Common/STRUCTS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AirSpeed/AIRSPEED.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "Barometer/BAROFRONTEND.h"
#include "GPSNavigation/NAVIGATION.h"

//O PDF ESTÁ NA PASTA 'DOCS' COM O NOME 'WindEstimation.pdf'

WindEstimatorClass WINDESTIMATOR;
WindEstimator_Struct WindEstimator;

bool EstimatedWindSpeedValid(void)
{
    return WindEstimator.ValidWindEstimated;
}

float GetEstimatedWindSpeed(uint8_t ArrayCount)
{
    return WindEstimator.EarthFrame.EstimatedWindVelocity[ArrayCount];
}

float GetEstimatedHorizontalWindSpeed(uint16_t *Angle)
{
    float EstimatedRollWindSpeed = GetEstimatedWindSpeed(ROLL);
    float EstimatedPitchWindSpeed = GetEstimatedWindSpeed(PITCH);
    if (Angle)
    {
        float EstimatedHorizontalWindAngle = Fast_Atan2(EstimatedPitchWindSpeed, EstimatedRollWindSpeed);
        if (EstimatedHorizontalWindAngle < 0)
        {
            EstimatedHorizontalWindAngle += 6.283185482f;
        }
        *Angle = ConvertRadiansToDeciDegrees(EstimatedHorizontalWindAngle);
    }
    return Fast_SquareRoot(SquareFloat(EstimatedRollWindSpeed) + SquareFloat(EstimatedPitchWindSpeed));
}

void UpdateWindEstimator() //50Hz
{
    WindEstimator.Time.Now = SCHEDULERTIME.GetMillis();

    if (!GetFrameStateOfAirPlane() || !Get_GPS_Heading_Is_Valid() || !GPSParameters.Navigation.Misc.Velocity.NEDStatus)
    {
        return;
    }

    //OBTÉM A VELOCIDADE 3D DO GPS EM CM/S
    WindEstimator.Ground.Velocity[ROLL] = GPSParameters.Navigation.Misc.Velocity.Get[ROLL];
    WindEstimator.Ground.Velocity[PITCH] = GPSParameters.Navigation.Misc.Velocity.Get[PITCH];
    WindEstimator.Ground.Velocity[YAW] = GPSParameters.Navigation.Misc.Velocity.Get[YAW];

    //OBTÉM A DIREÇÃO DA FUSELAGEM NO EARTH-FRAME (SACADO DO AHRS)
    WindEstimator.Fuselage.Direction[ROLL] = Rotation.Matrix3x3[0][0];
    WindEstimator.Fuselage.Direction[PITCH] = Rotation.Matrix3x3[1][0];
    WindEstimator.Fuselage.Direction[YAW] = Rotation.Matrix3x3[2][0];

    //RENOVA TODOS OS VALORES SE A MUDANÇA DE DIREÇÃO ESTIVER DEMORANDO MUITO
    if (WindEstimator.Time.LastUpdate == 0 || (WindEstimator.Time.Now - WindEstimator.Time.LastUpdate) > 10000) //0.1Hz
    {
        WindEstimator.Time.LastUpdate = WindEstimator.Time.Now;
        memcpy(WindEstimator.Fuselage.LastDirection, WindEstimator.Fuselage.Direction, sizeof(WindEstimator.Fuselage.LastDirection));
        memcpy(WindEstimator.Ground.LastVelocity, WindEstimator.Ground.Velocity, sizeof(WindEstimator.Ground.LastVelocity));
        return;
    }

    WindEstimator.Fuselage.DirectionDifference[ROLL] = WindEstimator.Fuselage.Direction[ROLL] - WindEstimator.Fuselage.LastDirection[ROLL];
    WindEstimator.Fuselage.DirectionDifference[PITCH] = WindEstimator.Fuselage.Direction[PITCH] - WindEstimator.Fuselage.LastDirection[PITCH];
    WindEstimator.Fuselage.DirectionDifference[YAW] = WindEstimator.Fuselage.Direction[YAW] - WindEstimator.Fuselage.LastDirection[YAW];

    float DifferenceLenght = SquareFloat(WindEstimator.Fuselage.DirectionDifference[ROLL]) +
                             SquareFloat(WindEstimator.Fuselage.DirectionDifference[PITCH]) +
                             SquareFloat(WindEstimator.Fuselage.DirectionDifference[YAW]);

    if (DifferenceLenght > SquareFloat(0.2f)) //A FUSELAGEM ESTÁ VIRANDO?SIM...
    {
        WindEstimator.Ground.VelocityDifference[ROLL] = WindEstimator.Ground.Velocity[ROLL] - WindEstimator.Ground.LastVelocity[ROLL];
        WindEstimator.Ground.VelocityDifference[PITCH] = WindEstimator.Ground.Velocity[PITCH] - WindEstimator.Ground.LastVelocity[PITCH];
        WindEstimator.Ground.VelocityDifference[YAW] = WindEstimator.Ground.Velocity[ROLL] - WindEstimator.Ground.LastVelocity[YAW];

        //VELOCIDADE ESTIMADA DO AR
        float EstimatedAirSpeed = (Fast_SquareRoot(SquareFloat(WindEstimator.Ground.VelocityDifference[ROLL]) +
                                                   SquareFloat(WindEstimator.Ground.VelocityDifference[PITCH]) +
                                                   SquareFloat(WindEstimator.Ground.VelocityDifference[YAW]))) /
                                  Fast_SquareRoot(DifferenceLenght);

        WindEstimator.Fuselage.DirectionSum[ROLL] = WindEstimator.Fuselage.Direction[ROLL] + WindEstimator.Fuselage.LastDirection[ROLL];
        WindEstimator.Fuselage.DirectionSum[PITCH] = WindEstimator.Fuselage.Direction[PITCH] + WindEstimator.Fuselage.LastDirection[PITCH];
        WindEstimator.Fuselage.DirectionSum[YAW] = WindEstimator.Fuselage.Direction[YAW] + WindEstimator.Fuselage.LastDirection[YAW];

        WindEstimator.Ground.VelocitySum[ROLL] = WindEstimator.Ground.Velocity[ROLL] + WindEstimator.Ground.LastVelocity[ROLL];
        WindEstimator.Ground.VelocitySum[PITCH] = WindEstimator.Ground.Velocity[PITCH] + WindEstimator.Ground.LastVelocity[PITCH];
        WindEstimator.Ground.VelocitySum[YAW] = WindEstimator.Ground.Velocity[YAW] + WindEstimator.Ground.LastVelocity[YAW];

        memcpy(WindEstimator.Fuselage.LastDirection, WindEstimator.Fuselage.Direction, sizeof(WindEstimator.Fuselage.LastDirection));
        memcpy(WindEstimator.Ground.LastVelocity, WindEstimator.Ground.Velocity, sizeof(WindEstimator.Ground.LastVelocity));

        float Theta = Fast_Atan2(WindEstimator.Ground.VelocityDifference[PITCH],
                                 WindEstimator.Ground.VelocityDifference[ROLL]) -
                      Fast_Atan2(WindEstimator.Fuselage.DirectionDifference[PITCH],
                                 WindEstimator.Fuselage.DirectionDifference[ROLL]); //EQUAÇÃO 9

        float SinTheta = Fast_Sine(Theta);
        float CosTheta = Fast_Cosine(Theta);

        float Wind[3];
        Wind[ROLL] = (WindEstimator.Ground.VelocitySum[ROLL] - EstimatedAirSpeed * (CosTheta * WindEstimator.Fuselage.DirectionSum[ROLL] - SinTheta * WindEstimator.Fuselage.DirectionSum[PITCH])) * 0.5f;   //EQUAÇÃO 10
        Wind[PITCH] = (WindEstimator.Ground.VelocitySum[PITCH] - EstimatedAirSpeed * (SinTheta * WindEstimator.Fuselage.DirectionSum[ROLL] + CosTheta * WindEstimator.Fuselage.DirectionSum[PITCH])) * 0.5f; //EQUAÇÃO 11
        Wind[YAW] = (WindEstimator.Ground.VelocitySum[YAW] - EstimatedAirSpeed * WindEstimator.Fuselage.DirectionSum[YAW]) * 0.5f;                                                                           //EQUAÇÃO 12

        float PreviousWindLength = Fast_SquareRoot(SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[ROLL]) +
                                                   SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[PITCH]) +
                                                   SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[YAW]));

        float WindLength = Fast_SquareRoot(SquareFloat(Wind[ROLL]) +
                                           SquareFloat(Wind[PITCH]) +
                                           SquareFloat(Wind[YAW]));

        if (WindLength < PreviousWindLength + 2000)
        {
            //FILTRO PARA REDUÇÃO DE NOISE
            WindEstimator.EarthFrame.EstimatedWindVelocity[ROLL] = WindEstimator.EarthFrame.EstimatedWindVelocity[ROLL] * 0.95f + Wind[ROLL] * 0.05f;
            WindEstimator.EarthFrame.EstimatedWindVelocity[PITCH] = WindEstimator.EarthFrame.EstimatedWindVelocity[PITCH] * 0.95f + Wind[PITCH] * 0.05f;
            WindEstimator.EarthFrame.EstimatedWindVelocity[YAW] = WindEstimator.EarthFrame.EstimatedWindVelocity[YAW] * 0.95f + Wind[YAW] * 0.05f;
        }
        WindEstimator.Time.LastUpdate = WindEstimator.Time.Now;
        WindEstimator.ValidWindEstimated = true;
    }
    else if (WindEstimator.Time.Now - WindEstimator.Time.LastUpdate > 2000 && //0.5HZ
             Get_AirSpeed_Enabled())                                          //TUBO DE PITOT ATIVADO?SIM...
    {
        //AO VOAR DIRETO,USE A VELOCIDADE DO TUDO DE PITOT PARA OBTER A ESTIMATIVA DO VENTO
        float AirSpeedVector[3];
        AirSpeedVector[ROLL] = WindEstimator.Fuselage.Direction[ROLL] * AirSpeed.Raw.IASPressure;
        AirSpeedVector[PITCH] = WindEstimator.Fuselage.Direction[PITCH] * AirSpeed.Raw.IASPressure;
        AirSpeedVector[YAW] = WindEstimator.Fuselage.Direction[YAW] * AirSpeed.Raw.IASPressure;

        float Wind[3];
        int32_t EAS2TAS = Get_EAS2TAS();
        Wind[ROLL] = WindEstimator.Ground.Velocity[ROLL] - (AirSpeedVector[ROLL] * EAS2TAS);
        Wind[PITCH] = WindEstimator.Ground.Velocity[PITCH] - (AirSpeedVector[PITCH] * EAS2TAS);
        Wind[YAW] = WindEstimator.Ground.Velocity[YAW] - (AirSpeedVector[YAW] * EAS2TAS);

        //FILTRO PARA REDUÇÃO DE NOISE
        WindEstimator.EarthFrame.EstimatedWindVelocity[ROLL] = WindEstimator.EarthFrame.EstimatedWindVelocity[ROLL] * 0.92f + Wind[ROLL] * 0.08f;
        WindEstimator.EarthFrame.EstimatedWindVelocity[PITCH] = WindEstimator.EarthFrame.EstimatedWindVelocity[PITCH] * 0.92f + Wind[PITCH] * 0.08f;
        WindEstimator.EarthFrame.EstimatedWindVelocity[YAW] = WindEstimator.EarthFrame.EstimatedWindVelocity[YAW] * 0.92f + Wind[YAW] * 0.08f;
    }
}