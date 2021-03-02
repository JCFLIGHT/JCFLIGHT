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

#include "AVRADC.h"

#ifdef __AVR_ATmega2560__

int16_t GPIOAnalogRead(uint8_t AnalogPin)
{
    uint8_t LowByte;
    uint8_t HighByte;
    if (AnalogPin >= 54)
    {
        AnalogPin -= 54;
    }
    //CONFIGURA O MUXB PARA O MEGA2560 (PINOS ADC DE 8 - 15)
    (*(volatile uint8_t *)(0x7B)) = ((*(volatile uint8_t *)(0x7B)) & ~(1 << 3)) | (((AnalogPin >> 0x03) & 0x01) << 3);
    (*(volatile uint8_t *)(0x7C)) = 64 | (AnalogPin & 0x07);                       //ANALOG REFERENCE PADRÃO DE 5V
    (*(volatile uint8_t *)(((uint16_t) & ((*(volatile uint8_t *)(0x7A)))))) |= 64; //INICIA A CONVERSÃO (ADC)
    while (((*(volatile uint8_t *)(((uint16_t) & ((*(volatile uint8_t *)(0x7A)))))) & 64))
    {
        //ESPERA A CONVERSÃO TERMINAR
    }
    LowByte = (*(volatile uint8_t *)(0x78));  //REALIZA O LEITURA DO ADCL
    HighByte = (*(volatile uint8_t *)(0x79)); //REALIZA O LEITURA DO ADCH
    return (HighByte << 8) | LowByte;         //JUNTA O BIT MAIS SIGNIFICATIVO COM O MENOS SIGNIFICATIVO E OBTÉM O VALOR ADC EM 16 BITS
}

#endif