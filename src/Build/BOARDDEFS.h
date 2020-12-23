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
#include "Common/ENUM.h"

//AVR
#ifdef __AVR_ATmega2560__
#define MEGA2560
#define THIS_LOOP_FREQUENCY 500 //HZ
#define I2C_AND_SERIAL_500HZ    //NÃO SEI PRA QUE EU COLOQUEI ESSA MERDA,NÃO ADIANTA DE NADA,O CICLO DE MAQUINA DO AVR JÁ TÁ FODIDO
#define INITIAL_ADDRESS_EEPROM_TO_CLEAR 0
#define FINAL_ADDRESS_EEPROM_TO_CLEAR 4096
#define SIZE_OF_EEPROM 4096
#define ADC_BATTERY_VOLTAGE ADC_NUM_0
#define ADC_BATTERY_CURRENT ADC_NUM_1
#define ADC_ANALOG_AIRSPEED ADC_NUM_2
#define SAFETY_BUTTON_PININPUT DDRK &= ~(1 << 6) //DECLARA COMO ENTRADA (BOTÃO)
#define SAFETY_BUTTON_PIN_READ_STATE (*(volatile uint8_t *)(0x106)) & 64
#define SAFETY_BUTTON_LED_PINOUT DDRA |= (1 << 3) //DEFINE A PORTA DIGITAL 25 COMO SAIDA
#define SAFETY_BUTTON_LED_ON PORTA &= ~(1 << 3)   //ATIVA O LED DO SAFE BUTTON
#define SAFETY_BUTTON_LED_OFF PORTA |= 1 << 3     //DESATIVA O LED DO SAFE BUTTON
#define BEEP_PINOUT DDRH |= (1 << DDD6)           //PORTA DIGITAL 9
#define BEEP_ON OCR2B = 1000 >> 3
#define BEEP_OFF OCR2B = 0
#define RED_LED_PINOUT DDRB |= (1 << DDD4)   //DEFINE A PORTA DIGITAL 10 COMO SAIDA
#define GREEN_LED_PINOUT DDRB |= (1 << DDD5) //DEFINE A PORTA DIGITAL 11 COMO SAIDA
#define BLUE_LED_PINOUT DDRB |= (1 << DDD6)  //DEFINE A PORTA DIGITAL 12 COMO SAIDA
#define RED_LED_PWM_REGISTER OCR2A           //PINO DIGITAL 10
#define GREEN_LED_PWM_REGISTER OCR1A         //PINO DIGITAL 11
#define BLUE_LED_PWM_REGISTER OCR1B          //PINO DIGITAL 12
#define RED_LED_ON PORTB |= 1 << 4           //PINO DIGITAL 10
#define RED_LED_OFF PORTB &= ~(1 << 4)       //PINO DIGITAL 10
#define GREEN_LED_ON PORTB |= 1 << 5         //PINO DIGITAL 11
#define GREEN_LED_OFF PORTB &= ~(1 << 5)     //PINO DIGITAL 11
#define BLUE_LED_ON PORTB |= 1 << 6          //PINO DIGITAL 12
#define BLUE_LED_OFF PORTB &= ~(1 << 6)      //PINO DIGITAL 12
#endif

//STM32
#ifdef __arm__
#define THIS_LOOP_FREQUENCY 1000 //HZ - PARA OS FILTROS LPF E NOTCH DA IMU
#define LOOP_PERIOD 1000         //RATE DO LOOP PRINCIPAL
#define INITIAL_ADDRESS_EEPROM_TO_CLEAR
#define FINAL_ADDRESS_EEPROM_TO_CLEAR
#define SIZE_OF_EEPROM
#define ADC_BATTERY_VOLTAGE
#define ADC_BATTERY_CURRENT
#define ADC_ANALOG_AIRSPEED
#define SAFETY_BUTTON_PININPUT
#define SAFETY_BUTTON_PIN_READ_STATE
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
#define RED_LED_ON
#define RED_LED_OFF
#define GREEN_LED_ON
#define GREEN_LED_OFF
#define BLUE_LED_ON
#define BLUE_LED_OFF
#endif