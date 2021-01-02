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

ESP32_Analog_Write_Struct ESP32_Analog_Write[16] = {
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13},
    {-1, 5000, 13}};

int AnalogWriteGetChannel(uint8_t Pin)
{
  int Channel = -1;
  for (uint8_t i = 0; i < 16; i++)
  {
    if (ESP32_Analog_Write[i].Pin == Pin)
    {
      Channel = i;
      break;
    }
  }
  if (Channel == -1)
  {
    for (uint8_t i = 0; i < 16; i++)
    {
      if (ESP32_Analog_Write[i].Pin == -1)
      {
        ESP32_Analog_Write[i].Pin = Pin;
        Channel = i;
        ledcSetup(Channel, ESP32_Analog_Write[i].Frequency, ESP32_Analog_Write[i].Resolution);
        ledcAttachPin(Pin, Channel);
        break;
      }
    }
  }

  return Channel;
}

void AnalogWriteSetFrequency(uint8_t Pin, double Frequency)
{
  int Channel = AnalogWriteGetChannel(Pin);
  if (Channel != -1 && Channel < 16)
  {
    ESP32_Analog_Write[Channel].Frequency = Frequency;
  }
}

void AnalogWriteApplyDuty(uint8_t Pin, uint32_t DutyCycle)
{
  int Channel = AnalogWriteGetChannel(Pin);
  if (Channel != -1 && Channel < 16)
  {
    uint8_t Resolution = ESP32_Analog_Write[Channel].Resolution;
    uint32_t Level = pow(2, Resolution);
    uint32_t CalcedDutyCycle = ((Level - 1) / 255) * DutyCycle;
    ledcWrite(Channel, CalcedDutyCycle);
  }
}