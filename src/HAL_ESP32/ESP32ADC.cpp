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

#include "ESP32ADC.h"

#ifdef ESP32
#include "esp_adc_cal.h"

void ConfigureADCCaptureWidth(adc_bits_width_t Width)
{
	adc1_config_width(Width);
}

void ConfigureADCAttenuationInDB(adc1_channel_t Channel, adc_atten_t AttenuationDB)
{
	adc1_config_channel_atten(Channel, AttenuationDB);
}

int16_t GetADCValue(adc1_channel_t Channel)
{
	return adc1_get_raw(Channel);
}

int16_t GPIOAnalogRead(uint8_t AnalogPin)
{
	ConfigureADCCaptureWidth(ADC_WIDTH_BIT_12);
	adc1_channel_t ADC_Channel = ADC1_CHANNEL_0;
	switch (AnalogPin)
	{

	case 0:
		ADC_Channel = ADC1_CHANNEL_6; //GPIO34
		break;

	case 1:
		ADC_Channel = ADC1_CHANNEL_7; //GPIO35
		break;

	case 2:
		ADC_Channel = ADC1_CHANNEL_4; //GPIO32
		break;

	case 3:
		ADC_Channel = ADC1_CHANNEL_5; //GPIO33
		break;

		/*		
		//OUTROS PINOS ADC PARA O FUTURO,QUEM SABE...
		case 36:
		ADC_Channel = ADC1_CHANNEL_0; //GPIO36
		break;

	case 37:
		ADC_Channel = ADC1_CHANNEL_1; //GPIO37
		break;

	case 38:
		ADC_Channel = ADC1_CHANNEL_2; //GPIO38
		break;

	case 39:
		ADC_Channel = ADC1_CHANNEL_3; //GPIO39
		break;
*/
	}
	ConfigureADCAttenuationInDB(ADC_Channel, ADC_ATTEN_DB_11);
	return GetADCValue(ADC_Channel);
	return 0;
}

#endif