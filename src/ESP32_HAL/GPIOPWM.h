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

#ifndef _GPIOPWM_H_
#define _GPIOPWM_H_
#include "Arduino.h"
typedef struct analog_write_channel
{
  int8_t Pin;
  double Frequency;
  uint8_t Resolution;
} ESP32_Analog_Write_Struct;
void AnalogWriteSetFrequency(uint8_t Pin, double Frequency);
void AnalogWriteApplyDuty(uint8_t Pin, uint32_t DutyCycle);
#endif