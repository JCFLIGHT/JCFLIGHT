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

//TOTAL ENERGY CONSERVATION SYSTEM - SISTEMA DE CONSERVAÇÃO TOTAL DE ENERGIA

TecsClass TECS;

GenericPIClass AutoThrottlePI;

float AirSpeedTarget = 0;

void TecsClass::Initialization(void)
{
    AutoThrottlePI.Set_kP(GET_SET[PID_ALTITUDE_HOLD].kI);
    AutoThrottlePI.Set_kI(GET_SET[PID_ALTITUDE_HOLD].kD);
    AutoThrottlePI.Set_kP_Scale(8);
    AutoThrottlePI.Set_kI_Scale(32);
    AutoThrottlePI.Set_Integral_Max(24000);
    AutoThrottlePI.Set_Integral_Scale(128);
    AutoThrottlePI.Set_Output_Min(AttitudeThrottleMin);
    AutoThrottlePI.Set_Output_Max(AttitudeThrottleMax);
}

static void Apply_Auto_Throttle_Control(float DeltaTime)
{
    if (GetFrameStateOfMultirotor())
    {
        return;
    }

    float TrueAirSpeed = AIRSPEED.Get_True_Value("In Centimeters");

    if (IS_FLIGHT_MODE_ACTIVE_ONCE(AUTO_THROTTLE_MODE))
    {
        //TALVEZ SEJA MELHOR USAR UMA VELOCIDADE DE AUTO-THROTTLE POR DEFINIÇÃO DO USUARIO COMO REFERENCIA PARA CALCULAR O "TargetError" DO PI
        AirSpeedTarget = TrueAirSpeed;
    }

    if (IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE) && !GetActualThrottleStatus(THROTTLE_LOW))
    {
        float TargetError = ConvertCentimeterPerSecondsToKmPerHour(TrueAirSpeed - AirSpeedTarget);
        float GetPICalced = AutoThrottlePI.Get_PI_Calced(TargetError, DeltaTime);
        RCController[THROTTLE] = AutoThrottlePI.GetPICalcedWithDataConstrained(GetPICalced, RCController[THROTTLE]);
    }
    else
    {
        AutoThrottlePI.Reset_Integral();
    }
}

static bool GetNavigationInAutomaticThrottleMode()
{
    return GetFrameStateOfAirPlane() & (IS_FLIGHT_MODE_ACTIVE(LAUNCH_MODE) |
                                        IS_FLIGHT_MODE_ACTIVE(CRUISE_MODE) |
                                        IS_FLIGHT_MODE_ACTIVE(CIRCLE_MODE) |
                                        IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE) |
                                        IS_FLIGHT_MODE_ACTIVE(RTH_MODE));
}

float TecsClass::AutoPitchDown(int16_t InCruise_Throttle, int16_t InMinThrottleDownPitchAngle)
{
    if (!GetNavigationInAutomaticThrottleMode())
    {
        return ScaleRange16Bits(MAX(0, InCruise_Throttle - RCController[THROTTLE]), 0, InCruise_Throttle - MIN_STICKS_PULSE, 0, InMinThrottleDownPitchAngle);
    }
    return 0;
}

void TecsClass::Update(float DeltaTime)
{
    Apply_Auto_Throttle_Control(DeltaTime);
}