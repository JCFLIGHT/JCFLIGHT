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

//ESP32
#ifdef ESP32
#include "HAL_ESP32/ESP32PWM.h"
#define THIS_LOOP_RATE_IN_US SCHEDULER_SET_FREQUENCY(1000, "KHz")    //RATE PARA O FILTRO BIQUAD,PT1 E TASK
#define ADC_BATTERY_VOLTAGE ADC_NUM_0                                //GPIO34
#define ADC_BATTERY_CURRENT ADC_NUM_1                                //GPIO35
#define ADC_ANALOG_AIR_SPEED ADC_NUM_2                               //GPIO32
#define SAFETY_BUTTON_PIN_READ_STATE ADC_NUM_3                       //GPIO33
#define SAFETY_BUTTON_LED_PINOUT pinMode(GPIO_NUM_5, OUTPUT)         //GPIO5
#define SAFETY_BUTTON_LED_ON AnalogWriteApplyPulse(GPIO_NUM_5, 4095) //ATIVA O LED DO SAFE BUTTON
#define SAFETY_BUTTON_LED_OFF AnalogWriteApplyPulse(GPIO_NUM_5, 0)   //DESATIVA O LED DO SAFE BUTTON
#define BEEP_PINOUT pinMode(GPIO_NUM_18, OUTPUT)                     //GPIO18
#define BEEP_ON AnalogWriteApplyPulse(GPIO_NUM_18, 1000)             //ATIVA O BUZZER
#define BEEP_OFF AnalogWriteApplyPulse(GPIO_NUM_18, 0)               //DESATIVA O BUZZER
#define RED_LED_PINOUT pinMode(GPIO_NUM_4, OUTPUT)                   //GPIO4
#define GREEN_LED_PINOUT pinMode(GPIO_NUM_2, OUTPUT)                 //GPIO2
#define BLUE_LED_PINOUT pinMode(GPIO_NUM_15, OUTPUT)                 //GPIO15
#define RED_LED_PWM_REGISTER GPIO_NUM_4                              //GPIO4
#define GREEN_LED_PWM_REGISTER GPIO_NUM_2                            //GPIO2
#define BLUE_LED_PWM_REGISTER GPIO_NUM_15                            //GPIO15
#endif