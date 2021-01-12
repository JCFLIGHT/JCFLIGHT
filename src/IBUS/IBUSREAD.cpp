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

static uint8_t IBUSIndex = 0;
static uint8_t IBUS_Vector[32];
uint16_t IBUSReadChannels[12];

void IBUS_Update(void)
{
  if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != 2)
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
      IBUS_Vector[IBUSIndex] = IBUS_SerialRead;
    }
    IBUSIndex++;
    if (IBUSIndex == 32)
    {
      IBUSIndex = 0;
      CheckSum = 0xFFFF;
      CheckSum -= IBUS_Vector[0];
      CheckSum -= IBUS_Vector[1];
      CheckSum -= IBUS_Vector[2];
      CheckSum -= IBUS_Vector[3];
      CheckSum -= IBUS_Vector[4];
      CheckSum -= IBUS_Vector[5];
      CheckSum -= IBUS_Vector[6];
      CheckSum -= IBUS_Vector[7];
      CheckSum -= IBUS_Vector[8];
      CheckSum -= IBUS_Vector[9];
      CheckSum -= IBUS_Vector[10];
      CheckSum -= IBUS_Vector[11];
      CheckSum -= IBUS_Vector[12];
      CheckSum -= IBUS_Vector[13];
      CheckSum -= IBUS_Vector[14];
      CheckSum -= IBUS_Vector[15];
      CheckSum -= IBUS_Vector[16];
      CheckSum -= IBUS_Vector[17];
      CheckSum -= IBUS_Vector[18];
      CheckSum -= IBUS_Vector[19];
      CheckSum -= IBUS_Vector[20];
      CheckSum -= IBUS_Vector[21];
      CheckSum -= IBUS_Vector[22];
      CheckSum -= IBUS_Vector[23];
      CheckSum -= IBUS_Vector[24];
      CheckSum -= IBUS_Vector[25];
      CheckSum -= IBUS_Vector[26];
      CheckSum -= IBUS_Vector[27];
      CheckSum -= IBUS_Vector[28];
      CheckSum -= IBUS_Vector[29];
      ReceiverSum = IBUS_Vector[30] + (IBUS_Vector[31] << 8);
      if (CheckSum == ReceiverSum)
      {
        IBUSReadChannels[0] = (IBUS_Vector[3] << 8) + IBUS_Vector[2];
        IBUSReadChannels[1] = (IBUS_Vector[5] << 8) + IBUS_Vector[4];
        IBUSReadChannels[2] = (IBUS_Vector[7] << 8) + IBUS_Vector[6];
        IBUSReadChannels[3] = (IBUS_Vector[9] << 8) + IBUS_Vector[8];
        IBUSReadChannels[4] = (IBUS_Vector[11] << 8) + IBUS_Vector[10];
        IBUSReadChannels[5] = (IBUS_Vector[13] << 8) + IBUS_Vector[12];
        IBUSReadChannels[6] = (IBUS_Vector[15] << 8) + IBUS_Vector[14];
        IBUSReadChannels[7] = (IBUS_Vector[17] << 8) + IBUS_Vector[16];
        IBUSReadChannels[8] = (IBUS_Vector[19] << 8) + IBUS_Vector[18];
        IBUSReadChannels[9] = (IBUS_Vector[21] << 8) + IBUS_Vector[20];
        IBUSReadChannels[10] = (IBUS_Vector[23] << 8) + IBUS_Vector[22];
        IBUSReadChannels[11] = (IBUS_Vector[25] << 8) + IBUS_Vector[24];
      }
    }
  }
}