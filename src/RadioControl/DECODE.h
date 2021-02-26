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

#ifndef DECODE_H_
#define DECODE_H_
#include "Build/LIBDEPENDENCIES.h"
class DecodeClass
{
public:
  volatile uint16_t PPMReadChannels[12];
  uint8_t RcChannelMap[12];
  int16_t DirectRadioControllRead[12];
  int16_t RadioControllOutput[12];
  void Initialization();
  void Update();
  int16_t GetRxChannelOutput(uint8_t Channel);
  void SetRxChannelInput(uint8_t Channel, int16_t Value);
};
extern DecodeClass DECODE;
#endif
