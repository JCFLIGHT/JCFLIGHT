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

#include "AIRSPEEDANALOG.h"
#include "Common/STRUCTS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AnalogDigitalConverter/ADC.h"
#include "Build/BOARDDEFS.h"
#include "FastSerial/PRINTF.h"

//AIR-SPEED MODELO MPXV7002DP (CONEXÃO DO TIPO ANALOGICA)

#define INITIAL_SAMPLES 50 //RECOLHE 50 AMOSTRAS INICIAIS DO SENSOR

float AirSpeed_Analog_Get_Calibration(void)
{
    LOG("AirSpeed Analogico em calib.Aguarde...");
    static float AirSpeedAnalogReturnValue = 0;
    for (uint8_t CountSamples = 0; CountSamples < INITIAL_SAMPLES; CountSamples++)
    {
        AirSpeedAnalogReturnValue = ANALOGDIGITALCONVERTER.Read(ADC_ANALOG_AIRSPEED);
        SCHEDULERTIME.Sleep(20);
    }
    LOG("Calib do AirSpeed Analogico finalizada!");
    LINE_SPACE;
    return AirSpeedAnalogReturnValue;
}

float AirSpeed_Analog_Get_Actual_Value(void)
{
    return ANALOGDIGITALCONVERTER.Read(ADC_ANALOG_AIRSPEED);
}