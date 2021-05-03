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

#include "IBUSREAD.h"
#include "FastSerial/FASTSERIAL.h"
#include "Common/STRUCTS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

IBUSClass IBUSRC;

#define IBUS_SERIAL_RX_PACKET_LENGTH 32
#define IBUS_CHECKSUM_SIZE 2
#define IBUS_MAX_SLOTS 14

static uint8_t IBUSIndex = 0;
static uint8_t IBUS_Packet[32];
uint16_t IBUSReadChannels[IBUS_MAX_CHANNELS];

void IBUSClass::Update(void)
{
  if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != IBUS_RECEIVER)
  {
    return;
  }
  uint16_t CheckSum;
  uint16_t ReceiverSum;
  while (FASTSERIAL.Available(UART_NUMB_2))
  {
    uint8_t IBUS_SerialRead = FASTSERIAL.Read(UART_NUMB_2);
    if (IBUSIndex == 0 && IBUS_SerialRead != 0x20)
    {
      continue;
    }
    if (IBUSIndex == 1 && IBUS_SerialRead != 0x40)
    {
      IBUSIndex = 0;
      continue;
    }
    if (IBUSIndex < 32)
    {
      IBUS_Packet[IBUSIndex] = IBUS_SerialRead;
    }
    IBUSIndex++;
    if (IBUSIndex == 32)
    {
      IBUSIndex = 0;
      CheckSum = 0xFFFF;

      for (size_t IndexCount = 0; IndexCount < IBUS_SERIAL_RX_PACKET_LENGTH - IBUS_CHECKSUM_SIZE; IndexCount++)
      {
        CheckSum -= IBUS_Packet[IndexCount];
      }

      ReceiverSum = IBUS_Packet[30] + (IBUS_Packet[31] << 8);

      if (CheckSum == ReceiverSum)
      {
        uint8_t IndexCount;
        uint8_t OffSet;

        for (IndexCount = 0, OffSet = 3; IndexCount < IBUS_MAX_SLOTS; IndexCount++, OffSet += 2)
        {
          IBUSReadChannels[IndexCount] = IBUS_Packet[OffSet] + ((IBUS_Packet[OffSet + 1] & 0x0F) << 8);
        }
#ifdef USE_IBUS_EXTENDED

        for (IndexCount = IBUS_MAX_SLOTS, OffSet = 3; IndexCount < IBUS_MAX_CHANNELS; IndexCount++, OffSet += 6)
        {
          IBUSReadChannels[IndexCount] = ((IBUS_Packet[OffSet] & 0xF0) >> 4) | (IBUS_Packet[OffSet + 2] & 0xF0) | ((IBUS_Packet[OffSet + 4] & 0xF0) << 4);
        }

#endif
      }
    }
  }
}