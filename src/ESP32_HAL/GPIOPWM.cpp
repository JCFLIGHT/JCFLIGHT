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

#include "GPIOPWM.h"
#include "Math/MATHSUPPORT.h"

#define MIN_PULSE 205 //4095 / 20
#define MAX_PULSE 410 //4095 / 10

void AnalogWriteSetSettings(uint8_t Pin, double Frequency, uint8_t Resolution)
{
  if (Pin == GPIO_NUM_25)
  {
    ledcSetup(12, Frequency, Resolution);
    ledcAttachPin(Pin, 12);
  }
  else if (Pin == GPIO_NUM_26)
  {
    ledcSetup(1, Frequency, Resolution);
    ledcAttachPin(Pin, 1);
  }
  else if (Pin == GPIO_NUM_27)
  {
    ledcSetup(2, Frequency, Resolution);
    ledcAttachPin(Pin, 2);
  }
  else if (Pin == GPIO_NUM_14)
  {
    ledcSetup(3, Frequency, Resolution);
    ledcAttachPin(Pin, 3);
  }
  else if (Pin == GPIO_NUM_12)
  {
    ledcSetup(4, Frequency, Resolution);
    ledcAttachPin(Pin, 4);
  }
  else if (Pin == GPIO_NUM_13)
  {
    ledcSetup(5, Frequency, Resolution);
    ledcAttachPin(Pin, 5);
  }
  else if (Pin == GPIO_NUM_23)
  {
    ledcSetup(6, Frequency, Resolution);
    ledcAttachPin(Pin, 6);
  }
  else if (Pin == GPIO_NUM_19)
  {
    ledcSetup(7, Frequency, Resolution);
    ledcAttachPin(Pin, 7);
  }
  else if (Pin == GPIO_NUM_4)
  {
    ledcSetup(8, Frequency, Resolution);
    ledcAttachPin(Pin, 8);
  }
  else if (Pin == GPIO_NUM_2)
  {
    ledcSetup(9, Frequency, Resolution);
    ledcAttachPin(Pin, 9);
  }
  else if (Pin == GPIO_NUM_15)
  {
    ledcSetup(10, Frequency, Resolution);
    ledcAttachPin(Pin, 10);
  }
  else if (Pin == GPIO_NUM_18)
  {
    ledcSetup(11, Frequency, Resolution);
    ledcAttachPin(Pin, 11);
  }
}

void AnalogWriteApplyPulse(uint8_t Pin, int16_t Pulse)
{
  if (Pin == GPIO_NUM_25)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(12, Pulse);
  }
  else if (Pin == GPIO_NUM_26)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(1, Pulse);
  }
  else if (Pin == GPIO_NUM_27)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(2, Pulse);
  }
  else if (Pin == GPIO_NUM_14)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(3, Pulse);
  }
  else if (Pin == GPIO_NUM_12)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(4, Pulse);
  }
  else if (Pin == GPIO_NUM_13)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(5, Pulse);
  }
  else if (Pin == GPIO_NUM_23)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(6, Pulse);
  }
  else if (Pin == GPIO_NUM_19)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(7, Pulse);
  }
  else if (Pin == GPIO_NUM_4)
  {
    ledcWrite(8, Pulse);
  }
  else if (Pin == GPIO_NUM_2)
  {
    ledcWrite(9, Pulse);
  }
  else if (Pin == GPIO_NUM_15)
  {
    ledcWrite(10, Pulse);
  }
  else if (Pin == GPIO_NUM_18)
  {
    Pulse = Map_16Bits(Pulse, 1000, 2000, MIN_PULSE, MAX_PULSE);
    ledcWrite(11, Pulse);
  }
}