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
        //FUTURAMENTE NÃO UTILIZAR MAIS KP E O KI DO AJUSTE DE ALTITUDE PARA SETAR VALORES,
        //E SIM USAR O KI E O KD PARA SETAR O KP E KI.DESSA FORMA O KP DO AJUSTE DE ALTITUDE
        //FICARÁ POR CONTA APENAS DO AJUSTE DO ALTITUDE-HOLD
        AutoThrottlePI.SetkP(GET_SET[PID_ALTITUDE].ProportionalVector / 8); //AINDA FALTA VERIFICAR SE ISSO ESTÁ CORRETO
        AutoThrottlePI.SetkI(GET_SET[PID_ALTITUDE].IntegralVector / 32);    //AINDA FALTA VERIFICAR SE ISSO ESTÁ CORRETO
        AutoThrottlePI.SetIntegratorMax(24000);
        AutoThrottlePI.SetOutputMin(MIN_PULSE);
        AutoThrottlePI.SetOutputMax(MAX_PULSE);
        /////////////////////////////////////////////////////////////////////////
        RCController[THROTTLE] = AutoThrottlePI.Get_PI_Calced(CalculateError, DeltaTime); //AINDA FALTA PUXAR O DELTA TIME
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

void TecsClass::Update(float DeltaTime)
{
    Apply_Auto_Throttle_Control(DeltaTime);
}