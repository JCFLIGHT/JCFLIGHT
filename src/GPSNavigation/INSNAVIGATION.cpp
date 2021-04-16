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

#include "INSNAVIGATION.h"
#include "PID/GPSPID.h"
#include "NAVIGATION.h"
#include "InertialNavigation/INS.h"
#include "Math/MATHSUPPORT.h"
#include "PID/PIDPARAMS.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Param/PARAM.h"
#include "PID/RCPID.h"

#define POS_HOLD_DEADBAND 20

void SetThisPointToPositionHold(void)
{
    INS.Position.Hold[COORD_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + INS.EarthFrame.Velocity[INS_LATITUDE] * PositionHoldPID.kI;
    INS.Position.Hold[COORD_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + INS.EarthFrame.Velocity[INS_LONGITUDE] * PositionHoldPID.kI;
}

static void ApplyINSPositionHoldPIDControl(float DeltaTime)
{
    static bool NewPointToPositionHold = false;

    //--------PARAMETRO DE CONFIG PARA O USUARIO--------//
    float MaxManualSpeed = 500; //CM/S
    //--------------------------------------------------//

    if (!IS_FLIGHT_MODE_ACTIVE(WAYPOINT_MODE) && (JCF_Param.AutoPilotMode == AUTOPILOT_MODE_CRUISE))
    {
        if (RCController[PITCH] || RCController[ROLL]) //CHECA SE OS STICKS DO RÁDIO FORAM MANIPULADOS
        {
            const float RadioControllVelocityRoll = RCController[PITCH] * MaxManualSpeed / (float)(500 - POS_HOLD_DEADBAND);
            const float RadioControllVelocityPitch = RCController[ROLL] * MaxManualSpeed / (float)(500 - POS_HOLD_DEADBAND);

            //ROTACIONA AS VELOCIDADES DO BODY-FRAME PARA EARTH-FRAME
            const float NEUVelocityRoll = RadioControllVelocityRoll * INS.Math.Cosine_Yaw - RadioControllVelocityPitch * INS.Math.Sine_Yaw;
            const float NEUVelocityPitch = RadioControllVelocityRoll * INS.Math.Sine_Yaw + RadioControllVelocityPitch * INS.Math.Cosine_Yaw;

            //CALCULA UMA NOVA POSIÇÃO
            INS.Position.Hold[INS_LATITUDE] = INS.EarthFrame.Position[INS_LATITUDE] + (NEUVelocityRoll * PositionHoldPID.kI);
            INS.Position.Hold[INS_LONGITUDE] = INS.EarthFrame.Position[INS_LONGITUDE] + (NEUVelocityPitch * PositionHoldPID.kI);

            NewPointToPositionHold = true;
        }
        else
        {
            if (NewPointToPositionHold)
            {
                SetThisPointToPositionHold();
                NewPointToPositionHold = false;
            }
        }
    }

    for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
    {
        int32_t PositionError = INS.Position.Hold[IndexCount] - INS.EarthFrame.Position[IndexCount];
        int32_t TargetPositionError = Constrain_32Bits(GPSGetProportional(PositionError, &PositionHoldPID), -1000, 1000);
        int32_t FinalPositionError = Constrain_32Bits(TargetPositionError - INS.EarthFrame.Velocity[IndexCount], -1000, 1000);
        GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] = GPSGetProportional(FinalPositionError, &PositionHoldRatePID) + GPSGetIntegral(FinalPositionError, DeltaTime, &PositionHoldRatePIDArray[IndexCount], &PositionHoldRatePID);
        GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] -= Constrain_16Bits((INS.AccelerationEarthFrame_Filtered[IndexCount] * PositionHoldRatePID.kD), -2000, 2000);
        GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount] = Constrain_16Bits(GPSParameters.Navigation.AutoPilot.INS.Angle[IndexCount], -ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue), ConvertDegreesToDecidegrees(GET_SET[GPS_BANK_MAX].MinMaxValue));
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
        GPSParameters.Navigation.AutoPilot.INS.Angle[COORD_LATITUDE] = 0;
        GPSParameters.Navigation.AutoPilot.INS.Angle[COORD_LONGITUDE] = 0;
        GPSResetPID(&PositionHoldRatePIDArray[COORD_LATITUDE]);
        GPSResetPID(&PositionHoldRatePIDArray[COORD_LONGITUDE]);
    }
}