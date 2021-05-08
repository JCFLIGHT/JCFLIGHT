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
#include "GPSNavigation/NAVIGATION.h"
#include "Build/BOARDDEFS.h"

//O PDF ESTÁ NA PASTA "DOCS" COM O NOME "WindEstimation.pdf"

WindEstimatorClass WINDESTIMATOR;
WindEstimator_Struct WindEstimator;

void WindEstimatorClass::Update(void)
{
#ifdef USE_WIND_ESTIMATOR

    WindEstimator.Time.Now = SCHEDULERTIME.GetMillis();

    if (!GetAirPlaneEnabled() || !Get_GPS_Heading_Is_Valid() || !GPS_Resources.Navigation.Misc.Velocity.NEDStatus)
    {
        return;
    }

    //OBTÉM A VELOCIDADE 3D DO GPS EM CM/S
    WindEstimator.Ground.Velocity[NORTH] = GPS_Resources.Navigation.Misc.Velocity.Get[NORTH];
    WindEstimator.Ground.Velocity[EAST] = GPS_Resources.Navigation.Misc.Velocity.Get[EAST];
    WindEstimator.Ground.Velocity[DOWN] = GPS_Resources.Navigation.Misc.Velocity.Get[DOWN];

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
        WindEstimator.Ground.VelocityDifference[NORTH] = WindEstimator.Ground.Velocity[NORTH] - WindEstimator.Ground.LastVelocity[NORTH];
        WindEstimator.Ground.VelocityDifference[EAST] = WindEstimator.Ground.Velocity[EAST] - WindEstimator.Ground.LastVelocity[EAST];
        WindEstimator.Ground.VelocityDifference[DOWN] = WindEstimator.Ground.Velocity[DOWN] - WindEstimator.Ground.LastVelocity[DOWN];

        //VELOCIDADE ESTIMADA DO AR
        float EstimatedAirSpeed = (Fast_SquareRoot(SquareFloat(WindEstimator.Ground.VelocityDifference[NORTH]) +
                                                   SquareFloat(WindEstimator.Ground.VelocityDifference[EAST]) +
                                                   SquareFloat(WindEstimator.Ground.VelocityDifference[DOWN]))) /
                                  Fast_SquareRoot(DifferenceLenght);

        WindEstimator.Fuselage.DirectionSum[ROLL] = WindEstimator.Fuselage.Direction[ROLL] + WindEstimator.Fuselage.LastDirection[ROLL];
        WindEstimator.Fuselage.DirectionSum[PITCH] = WindEstimator.Fuselage.Direction[PITCH] + WindEstimator.Fuselage.LastDirection[PITCH];
        WindEstimator.Fuselage.DirectionSum[YAW] = WindEstimator.Fuselage.Direction[YAW] + WindEstimator.Fuselage.LastDirection[YAW];

        WindEstimator.Ground.VelocitySum[NORTH] = WindEstimator.Ground.Velocity[NORTH] + WindEstimator.Ground.LastVelocity[NORTH];
        WindEstimator.Ground.VelocitySum[EAST] = WindEstimator.Ground.Velocity[EAST] + WindEstimator.Ground.LastVelocity[EAST];
        WindEstimator.Ground.VelocitySum[DOWN] = WindEstimator.Ground.Velocity[DOWN] + WindEstimator.Ground.LastVelocity[DOWN];

        memcpy(WindEstimator.Fuselage.LastDirection, WindEstimator.Fuselage.Direction, sizeof(WindEstimator.Fuselage.LastDirection));
        memcpy(WindEstimator.Ground.LastVelocity, WindEstimator.Ground.Velocity, sizeof(WindEstimator.Ground.LastVelocity));

        //EQUAÇÃO 9
        float Theta = Fast_Atan2(WindEstimator.Ground.VelocityDifference[EAST],
                                 WindEstimator.Ground.VelocityDifference[NORTH]) -
                      Fast_Atan2(WindEstimator.Fuselage.DirectionDifference[PITCH],
                                 WindEstimator.Fuselage.DirectionDifference[ROLL]);

        float SinTheta = Fast_Sine(Theta);
        float CosTheta = Fast_Cosine(Theta);

        float Wind[3];
        //EQUAÇÃO 10
        Wind[NORTH] = (WindEstimator.Ground.VelocitySum[NORTH] - EstimatedAirSpeed * (CosTheta * WindEstimator.Fuselage.DirectionSum[ROLL] - SinTheta * WindEstimator.Fuselage.DirectionSum[PITCH])) * 0.5f;
        //EQUAÇÃO 11
        Wind[EAST] = (WindEstimator.Ground.VelocitySum[EAST] - EstimatedAirSpeed * (SinTheta * WindEstimator.Fuselage.DirectionSum[ROLL] + CosTheta * WindEstimator.Fuselage.DirectionSum[PITCH])) * 0.5f;
        //EQUAÇÃO 12
        Wind[DOWN] = (WindEstimator.Ground.VelocitySum[DOWN] - EstimatedAirSpeed * WindEstimator.Fuselage.DirectionSum[YAW]) * 0.5f;

        float PreviousWindLength = Fast_SquareRoot(SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH]) +
                                                   SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[EAST]) +
                                                   SquareFloat(WindEstimator.EarthFrame.EstimatedWindVelocity[DOWN]));

        float WindLength = Fast_SquareRoot(SquareFloat(Wind[NORTH]) +
                                           SquareFloat(Wind[EAST]) +
                                           SquareFloat(Wind[DOWN]));

        if (WindLength < PreviousWindLength + 2000)
        {
            //FILTRO PARA REDUÇÃO DE NOISE
            WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH] = WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH] * 0.95f + Wind[NORTH] * 0.05f;
            WindEstimator.EarthFrame.EstimatedWindVelocity[EAST] = WindEstimator.EarthFrame.EstimatedWindVelocity[EAST] * 0.95f + Wind[EAST] * 0.05f;
            WindEstimator.EarthFrame.EstimatedWindVelocity[DOWN] = WindEstimator.EarthFrame.EstimatedWindVelocity[DOWN] * 0.95f + Wind[DOWN] * 0.05f;
        }

        WindEstimator.Time.LastUpdate = WindEstimator.Time.Now;
        WindEstimator.ValidWindEstimated = true;
    }
    /*
    else if (WindEstimator.Time.Now - WindEstimator.Time.LastUpdate > 2000 &&    //0.5HZ
             Get_AirSpeed_Enabled() && Get_AirSpeed_Type() != VIRTUAL_AIR_SPEED) //TUBO DE PITOT REAL ATIVADO?SIM...
    {
        //AO VOAR DIRETO,USE A VELOCIDADE DO TUDO DE PITOT PARA OBTER A ESTIMATIVA DO VENTO
        float AirSpeedVector[3];
        float TrueAirSpeed = AIRSPEED.Get_True_Value("In Meters");
        AirSpeedVector[NORTH] = WindEstimator.Fuselage.Direction[ROLL] * TrueAirSpeed;
        AirSpeedVector[EAST] = WindEstimator.Fuselage.Direction[PITCH] * TrueAirSpeed;
        AirSpeedVector[DOWN] = WindEstimator.Fuselage.Direction[YAW] * TrueAirSpeed;

        float Wind[3];
        Wind[NORTH] = WindEstimator.Ground.Velocity[NORTH] - AirSpeedVector[NORTH];
        Wind[EAST] = WindEstimator.Ground.Velocity[EAST] - AirSpeedVector[EAST];
        Wind[DOWN] = WindEstimator.Ground.Velocity[DOWN] - AirSpeedVector[DOWN];

        //FILTRO PARA REDUÇÃO DE NOISE
        WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH] = WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH] * 0.92f + Wind[NORTH] * 0.08f;
        WindEstimator.EarthFrame.EstimatedWindVelocity[EAST] = WindEstimator.EarthFrame.EstimatedWindVelocity[EAST] * 0.92f + Wind[EAST] * 0.08f;
        WindEstimator.EarthFrame.EstimatedWindVelocity[DOWN] = WindEstimator.EarthFrame.EstimatedWindVelocity[DOWN] * 0.92f + Wind[DOWN] * 0.08f;

        //NÃO SEI SE O CALCULO ESTÁ CORRETO,É NECESSARIO TESTAR,FOI BASEADO NA ARDUPILOT
        float AirSpeedX = WindEstimator.Ground.Velocity[NORTH] - WindEstimator.EarthFrame.EstimatedWindVelocity[NORTH];
        float AirSpeedY = WindEstimator.Ground.Velocity[EAST] - WindEstimator.EarthFrame.EstimatedWindVelocity[EAST];
        float AirSpeedFinal = Fast_SquareRoot(SquareFloat(AirSpeedX) + SquareFloat(AirSpeedY)); //VELOCIDADE TOTAL EM LINHA RETA FUNDINDO O GPS COM O AIR-SPEED
    }
*/
#endif
}

float WindEstimatorClass::GetEstimatedValueHorizontal(uint16_t *Angle)
{
    //O PONTEIRO "Angle" É O ANGULO DO VENTO EM CENTIDEGREES
    //O "return" É A VELOCIDADE TOTAL DO VENTO EM CM/S
    float EstimatedRollWindSpeed = WINDESTIMATOR.GetEstimatedInAxis(NORTH);
    float EstimatedPitchWindSpeed = WINDESTIMATOR.GetEstimatedInAxis(EAST);
    if (Angle)
    {
        float EstimatedHorizontalWindAngle = Fast_Atan2(EstimatedPitchWindSpeed, EstimatedRollWindSpeed);
        if (EstimatedHorizontalWindAngle < 0)
        {
            EstimatedHorizontalWindAngle += 6.283185482f;
        }
        *Angle = ConvertRadiansToCentiDegrees(EstimatedHorizontalWindAngle);
    }
    return Fast_SquareRoot(SquareFloat(EstimatedRollWindSpeed) + SquareFloat(EstimatedPitchWindSpeed));
}

float WindEstimatorClass::GetEstimatedInAxis(uint8_t ArrayCount)
{
    return WindEstimator.EarthFrame.EstimatedWindVelocity[ArrayCount];
}

bool WindEstimatorClass::EstimatedValid(void)
{
    return WindEstimator.ValidWindEstimated;
}