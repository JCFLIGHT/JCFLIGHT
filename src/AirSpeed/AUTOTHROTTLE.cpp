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

#include "AUTOTHROTTLE.h"
#include "AIRSPEED.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "PID/RCPID.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "Common/STRUCTS.h"
#include "GenericPI/GENERICPI.h"

//#define APPLY_NEW_PI

#ifdef APPLY_NEW_PI
GenericPIClass AutoThrottlePI;
#endif

int16_t CalculateIntegral = 0;
uint16_t PreviousValueOfAirSpeed = 0;

void AirSpeed_Apply_Auto_Throttle_Control()
{
    if (GetFrameStateOfMultirotor())
    {
        return;
    }

    if (IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE))
    {
        if (IS_FLIGHT_MODE_ACTIVE_ONCE(AUTO_THROTTLE_MODE))
        {
            PreviousValueOfAirSpeed = ConvertCentimeterPerSecondsToKmPerHour(AIRSPEED.CalcedInCM);
        }
    }

    if (IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE) && (RCController[THROTTLE] > 1200))
    {
        int16_t CalculateError = PreviousValueOfAirSpeed - ConvertCentimeterPerSecondsToKmPerHour(AIRSPEED.CalcedInCM);
        int16_t CalculateProportional = (CalculateError * GET_SET[PID_ALTITUDE].ProportionalVector >> 3);
        CalculateIntegral += (CalculateError * GET_SET[PID_ALTITUDE].IntegralVector >> 5);
        CalculateIntegral = Constrain_16Bits(CalculateIntegral, -24000, 24000);
        RCController[THROTTLE] = Constrain_16Bits(RCController[THROTTLE] + CalculateProportional + (CalculateIntegral >> 7), 1100, 1900);

#ifdef APPLY_NEW_PI
        ///////////////////////////////DENTRO DO SETUP////////////////////////////
        AutoThrottlePI.SetkP(GET_SET[PID_ALTITUDE].ProportionalVector / 8); //AINDA FALTA VERIFICAR SE ISSO ESTÁ CORRETO
        AutoThrottlePI.SetkI(GET_SET[PID_ALTITUDE].IntegralVector / 32);    //AINDA FALTA VERIFICAR SE ISSO ESTÁ CORRETO
        AutoThrottlePI.SetIntegratorMax(24000);
        AutoThrottlePI.SetOutputMin(MIN_PULSE);
        AutoThrottlePI.SetOutputMax(MAX_PULSE);
        /////////////////////////////////////////////////////////////////////////
        RCController[THROTTLE] = AutoThrottlePI.Get_PI_Calced(CalculateError, 1000); //AINDA FALTA PUXAR O DELTA TIME
#endif
    }
    else
    {
        CalculateIntegral = 0;
#ifdef APPLY_NEW_PI
        AutoThrottlePI.Reset_Integrator();
#endif
    }
}