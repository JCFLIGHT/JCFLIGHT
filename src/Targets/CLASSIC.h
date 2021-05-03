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

//AVR
#ifdef __AVR_ATmega2560__
#define THIS_LOOP_RATE_IN_US 4000                 //(ESSE VALOR É UM HACK PARA A VERSÃO CLASSIC) RATE PARA O FILTRO BIQUAD E PT1
#define ADC_BATTERY_VOLTAGE ADC_NUM_0             //PINO ANALOGICO CONECTADO O SENSOR DE TENSÃO
#define ADC_BATTERY_CURRENT ADC_NUM_1             //PINO ANALOGICO CONECTADO O SENSOR DE CORRENTE
#define ADC_ANALOG_AIR_SPEED ADC_NUM_2            //PINO ANALOGICO CONECTADO O SENSOR DE PRESSÃO (TUBO DE PITOT)
#define SAFETY_BUTTON_PIN_READ_STATE ADC_NUM_14   //PINO ADC 14
#define SAFETY_BUTTON_LED_PINOUT DDRA |= (1 << 3) //DEFINE A PORTA DIGITAL 25 COMO SAIDA
#define SAFETY_BUTTON_LED_ON PORTA &= ~(1 << 3)   //ATIVA O LED DO SAFE BUTTON
#define SAFETY_BUTTON_LED_OFF PORTA |= 1 << 3     //DESATIVA O LED DO SAFE BUTTON
#define BEEP_PINOUT DDRH |= (1 << 6)              //PORTA DIGITAL 9
#define BEEP_ON OCR2B = 1000 >> 3                 //REGISTRADOR PWM DO PINO 9
#define BEEP_OFF OCR2B = 0                        //REGISTRADOR PWM DO PINO 9
#define RED_LED_PINOUT DDRB |= (1 << 4)           //DEFINE A PORTA DIGITAL 10 COMO SAIDA
#define GREEN_LED_PINOUT DDRB |= (1 << 5)         //DEFINE A PORTA DIGITAL 11 COMO SAIDA
#define BLUE_LED_PINOUT DDRB |= (1 << 6)          //DEFINE A PORTA DIGITAL 12 COMO SAIDA
#define RED_LED_PWM_REGISTER OCR2A                //PINO DIGITAL 10
#define GREEN_LED_PWM_REGISTER OCR1A              //PINO DIGITAL 11
#define BLUE_LED_PWM_REGISTER OCR1B               //PINO DIGITAL 12
#endif