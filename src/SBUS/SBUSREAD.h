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

#ifndef SBUSREAD_H_
#define SBUSREAD_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Build/BOARDDEFS.h"
extern uint16_t SBUSReadChannels[SBUS_MAX_CHANNELS];
class SBUS
{
public:
  void Update(void);
  bool FailSafe;

private:
  bool SerialParse(void);
  uint8_t ParserState;
  uint8_t PrevByte = 0x00;
  uint8_t ActualByte;
  uint8_t PayLoadArray[24];
  void Read(uint16_t *ChannelsRead, bool *FailSafe, bool *LostFrame);
};
extern SBUS SBUSRC;
#endif
