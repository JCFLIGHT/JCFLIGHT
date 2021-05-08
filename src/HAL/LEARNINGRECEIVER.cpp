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

#include "LEARNINGRECEIVER.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "RadioControl/DECODE.h"
#include "BAR/BAR.h"
#include "SBUS/SBUSREAD.h"
#include "IBUS/IBUSREAD.h"
#include "Common/ENUM.h"
#include "PID/RCPID.h"

uint16_t LearningChannelsOfReceiver(uint8_t Channels)
{
    uint16_t ReceiverData;
    if (RC_Resources.ReceiverTypeEnabled == SBUS_RECEIVER)
    {
        ReceiverData = SBUSReadChannels[DECODE.RadioControlChannelsMap[Channels]];
    }
    else if (RC_Resources.ReceiverTypeEnabled == IBUS_RECEIVER)
    {
        ReceiverData = IBUSReadChannels[DECODE.RadioControlChannelsMap[Channels]];
    }
    else
    {
#ifdef __AVR_ATmega2560__
        uint8_t oldSREG;
        oldSREG = SREG;
        __asm__ __volatile__("cli" ::
                                 : "memory");
#endif
        ReceiverData = DECODE.PPMReadChannels[DECODE.RadioControlChannelsMap[Channels]];
#ifdef __AVR_ATmega2560__
        SREG = oldSREG;
#endif
    }
    return ReceiverData;
}