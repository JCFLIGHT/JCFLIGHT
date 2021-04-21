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

#pragma once

//STM32
#ifdef __arm__
#define THIS_LOOP_FREQUENCY 1000 //RATE PARA O FILTRO BIQUAD E TASK
#define INITIAL_ADDRESS_EEPROM_TO_CLEAR 0
#define FINAL_ADDRESS_EEPROM_TO_CLEAR 2000
#define SIZE_OF_EEPROM 2000
#define ADC_BATTERY_VOLTAGE ADC_NUM_0
#define ADC_BATTERY_CURRENT ADC_NUM_1
#define ADC_ANALOG_AIR_SPEED ADC_NUM_2
#define SAFETY_BUTTON_PININPUT
#define SAFETY_BUTTON_PIN_READ_STATE ADC_NUM_3
#define SAFETY_BUTTON_LED_PINOUT
#define SAFETY_BUTTON_LED_ON
#define SAFETY_BUTTON_LED_OFF
#define BEEP_PINOUT
#define BEEP_ON
#define BEEP_OFF
#define RED_LED_PINOUT
#define GREEN_LED_PINOUT
#define BLUE_LED_PINOUT
#define RED_LED_PWM_REGISTER
#define GREEN_LED_PWM_REGISTER
#define BLUE_LED_PWM_REGISTER
#endif