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

//TOTAL ENERGY CONSERVATION SYSTEM - SISTEMA DE CONSERVAÇÃO TOTAL DE ENERGIA

#include "TECS.h"
#include "AirSpeed/AIRSPEED.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "PID/RCPID.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "PID/PIDPARAMS.h"
#include "GenericPI/GENERICPI.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "FastSerial/PRINTF.h"

TecsClass TECS;

//#define APPLY_NEW_PI

#ifdef APPLY_NEW_PI
GenericPIClass AutoThrottlePI;
#endif

int16_t CalculateIntegral = 0;
uint16_t PreviousValueOfAirSpeed = 0;

static void Apply_Auto_Throttle_Control(float DeltaTime)
{
    if (GetFrameStateOfMultirotor())
    {
        return;
    }

    if (IS_FLIGHT_MODE_ACTIVE_ONCE(AUTO_THROTTLE_MODE))
    {
        PreviousValueOfAirSpeed = ConvertCentimeterPerSecondsToKmPerHour(AirSpeed.Raw.IASPressureInCM);
    }

    if (IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE) && !GetActualThrottleStatus(THROTTLE_LOW))
    {
        int16_t CalculateError = PreviousValueOfAirSpeed - ConvertCentimeterPerSecondsToKmPerHour(AirSpeed.Raw.IASPressureInCM);

        int16_t CalculateProportional = (CalculateError * GET_SET[PID_ALTITUDE].kP >> 3);
        CalculateIntegral += (CalculateError * GET_SET[PID_ALTITUDE].kI >> 5);
        CalculateIntegral = Constrain_16Bits(CalculateIntegral, -24000, 24000);
        RCController[THROTTLE] = Constrain_16Bits(RCController[THROTTLE] + CalculateProportional + (CalculateIntegral >> 7), 1100, 1900);

#ifdef APPLY_NEW_PI
        ///////////////////////////////DENTRO DO SETUP////////////////////////////
        //FUTURAMENTE NÃO UTILIZAR MAIS KP E O KI DO AJUSTE DE ALTITUDE PARA SETAR VALORES,
        //E SIM USAR O KI E O KD PARA SETAR O KP E KI.DESSA FORMA O KP DO AJUSTE DE ALTITUDE
        //FICARÁ POR CONTA APENAS DO AJUSTE DO ALTITUDE-HOLD
        AutoThrottlePI.Set_kP(GET_SET[PID_ALTITUDE].kP);
        AutoThrottlePI.Set_kI(GET_SET[PID_ALTITUDE].Integral);
        AutoThrottlePI.Set_kP_Scale(8);
        AutoThrottlePI.Set_kI_Scale(32);
        AutoThrottlePI.Set_Integral_Max(24000);
        AutoThrottlePI.Set_Integral_Scale(128);
        AutoThrottlePI.Set_Output_Min(AttitudeThrottleMin);
        AutoThrottlePI.Set_Output_Max(AttitudeThrottleMax);
        /////////////////////////////////////////////////////////////////////////
        float GetPICalced = AutoThrottlePI.Get_PI_Calced(CalculateError, 1); //USANDO O DELTATIME O VALOR FICA INCORRETO - GERAR CORREÇÃO PRA ISSO
        RCController[THROTTLE] = AutoThrottlePI.GetPICalcedWithDataConstrained(GetPICalced, RCController[THROTTLE]);
#endif
    }
    else
    {
        CalculateIntegral = 0;
#ifdef APPLY_NEW_PI
        AutoThrottlePI.Reset_Integral();
#endif
    }
}

static bool AirPlaneNavigationIsControllingThrottle()
{
    return GetFrameStateOfAirPlane() &&
           !IS_FLIGHT_MODE_ACTIVE(LAUNCH_MODE) &&
           !IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE) &&
           !IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) &&
           !IS_FLIGHT_MODE_ACTIVE(RTH_MODE);
}

float TecsClass::AutoPitchDown(int16_t InCruise_Throttle, int16_t InMinThrottleDownPitchAngle)
{
    if (AirPlaneNavigationIsControllingThrottle())
    {
        return ScaleRange16Bits(MAX(0, InCruise_Throttle - RCController[THROTTLE]), 0, InCruise_Throttle - MIN_STICKS_PULSE, 0, InMinThrottleDownPitchAngle);
    }
    return 0;
}

void TecsClass::Update(float DeltaTime)
{
    Apply_Auto_Throttle_Control(DeltaTime);
}