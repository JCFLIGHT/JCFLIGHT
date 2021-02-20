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
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"

int16_t CalculateIntegrator = 0;
uint16_t PreviousValueOfAirSpeed = 0;

void AirSpeed_Update_Auto_Throttle()
{
    if (GetFrameStateOfMultirotor())
    {
        return;
    }
    if (IS_FLIGHT_MODE_ACTIVE(AUTO_THROTTLE_MODE))
    {
        if (!Do_AutoThrottle_Mode)
        {
            Do_AutoThrottle_Mode = true;
            PreviousValueOfAirSpeed = AirSpeedCalcedInKM;
        }
    }
    else
    {
        Do_AutoThrottle_Mode = false;
    }
}

void AirSpeed_Apply_Auto_Throttle_Control()
{
    if (GetFrameStateOfMultirotor())
    {
        return;
    }
    if (Do_AutoThrottle_Mode && (RCController[THROTTLE] > 1200))
    {
        int16_t CalculateError = PreviousValueOfAirSpeed - AirSpeedCalcedInKM;
        int16_t CalculateProportional = (CalculateError * PID[PIDALTITUDE].ProportionalVector >> 3);
        CalculateIntegrator += (CalculateError * PID[PIDALTITUDE].IntegratorVector >> 5);
        CalculateIntegrator = Constrain_16Bits(CalculateIntegrator, -24000, 24000);
        RCController[THROTTLE] = Constrain_16Bits(RCController[THROTTLE] + CalculateProportional + (CalculateIntegrator >> 7), 1100, 1900);
    }
    else
    {
        CalculateIntegrator = 0;
    }
}