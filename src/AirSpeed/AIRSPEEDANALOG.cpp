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
#include "AnalogDigitalConverter/ADC.h"
#include "Build/BOARDDEFS.h"

//AIR-SPEED MODELO MPXV7002DP (CONEXÃO DO TIPO ANALOGICA)

#define PITOT_ADC_VOLTAGE_TO_PRESSURE 1000.0f

float AirSpeed_Analog_Get_Actual_Value(void)
{
  return (ANALOGSOURCE.Read_Voltage_Ratiometric(ADC_ANALOG_AIR_SPEED) * ADC_VOLTAGE_SCALER - ADC_VOLTAGE_ZERO) * PITOT_ADC_VOLTAGE_TO_PRESSURE;
}