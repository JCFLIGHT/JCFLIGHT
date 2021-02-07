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

#include "SBUSREAD.h"
#include "FastSerial/FASTSERIAL.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FastSerial/PRINTF.h"
#include "BAR/BAR.h"
#include "Common/ENUM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

SBUS SBUSRC;

//#define DEBUG_MODE

bool LostFrame = false;
uint16_t SBUSReadChannels[12];

void SBUS_Update()
{
  if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != SBUS_RECEIVER)
  {
    return;
  }
  SBUSRC.Read(&SBUSReadChannels[0], &SBUSRC.FailSafe, &LostFrame);
#if defined(DEBUG_MODE)
  static uint32_t SBUS_Serial_Refresh;
  if (SCHEDULERTIME.GetMillis() - SBUS_Serial_Refresh >= 20)
  {
    FastSerialPrintln(PSTR("Thr:%d Yaw:%d Pitch:%d Roll:%d Aux1:%d Aux2:%d Aux3:%d Aux4:%d Aux5:%d Aux6:%d Aux7:%d Aux8:%d FailSafe:%d\n"),
                      SBUSReadChannels[0], SBUSReadChannels[1], SBUSReadChannels[2], SBUSReadChannels[3], SBUSReadChannels[4],
                      SBUSReadChannels[5], SBUSReadChannels[6], SBUSReadChannels[7], SBUSRC.FailSafe);
    SBUS_Serial_Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}

void SBUS::Read(uint16_t *ChannelsRead, bool *FailSafe, bool *LostFrame)
{
  if (SerialParse())
  {
    if (ChannelsRead)
    {
      ChannelsRead[0] = (uint16_t)((PayLoadArray[0] | PayLoadArray[1] << 8) & 0x07FF) / 2 + 988;
      ChannelsRead[1] = (uint16_t)((PayLoadArray[1] >> 3 | PayLoadArray[2] << 5) & 0x07FF) / 2 + 988;
      ChannelsRead[2] = (uint16_t)((PayLoadArray[2] >> 6 | PayLoadArray[3] << 2 | PayLoadArray[4] << 10) & 0x07FF) / 2 + 988;
      ChannelsRead[3] = (uint16_t)((PayLoadArray[4] >> 1 | PayLoadArray[5] << 7) & 0x07FF) / 2 + 988;
      ChannelsRead[4] = (uint16_t)((PayLoadArray[5] >> 4 | PayLoadArray[6] << 4) & 0x07FF) / 2 + 988;
      ChannelsRead[5] = (uint16_t)((PayLoadArray[6] >> 7 | PayLoadArray[7] << 1 | PayLoadArray[8] << 9) & 0x07FF) / 2 + 988;
      ChannelsRead[6] = (uint16_t)((PayLoadArray[8] >> 2 | PayLoadArray[9] << 6) & 0x07FF) / 2 + 988;
      ChannelsRead[7] = (uint16_t)((PayLoadArray[9] >> 5 | PayLoadArray[10] << 3) & 0x07FF) / 2 + 988;
      ChannelsRead[8] = (uint16_t)((PayLoadArray[11] | PayLoadArray[12] << 8) & 0x07FF) / 2 + 988;
      ChannelsRead[9] = (uint16_t)((PayLoadArray[12] >> 3 | PayLoadArray[13] << 5) & 0x07FF) / 2 + 988;
      ChannelsRead[10] = (uint16_t)((PayLoadArray[13] >> 6 | PayLoadArray[14] << 2 | PayLoadArray[15] << 10) & 0x07FF) / 2 + 988;
      ChannelsRead[11] = (uint16_t)((PayLoadArray[15] >> 1 | PayLoadArray[16] << 7) & 0x07FF) / 2 + 988;
      ChannelsRead[12] = (uint16_t)((PayLoadArray[16] >> 4 | PayLoadArray[17] << 4) & 0x07FF) / 2 + 988;
      ChannelsRead[13] = (uint16_t)((PayLoadArray[17] >> 7 | PayLoadArray[18] << 1 | PayLoadArray[19] << 9) & 0x07FF) / 2 + 988;
      ChannelsRead[14] = (uint16_t)((PayLoadArray[19] >> 2 | PayLoadArray[20] << 6) & 0x07FF) / 2 + 988;
      ChannelsRead[15] = (uint16_t)((PayLoadArray[20] >> 5 | PayLoadArray[21] << 3) & 0x07FF) / 2 + 988;
    }
    if (LostFrame)
    {
      if (PayLoadArray[22] & 0x04)
      {
        *LostFrame = true;
      }
      else
      {
        *LostFrame = false;
      }
    }
    if (FailSafe)
    {
      if (PayLoadArray[22] & 0x08)
      {
        *FailSafe = true;
      }
      else
      {
        *FailSafe = false;
        if (Fail_Safe_System > 20)
        {
          Fail_Safe_System -= 20;
        }
        else
        {
          Fail_Safe_System = 0;
        }
      }
    }
  }
}

bool SBUS::SerialParse()
{
  static uint32_t SBUS_Stored_Time = 0;
  if (SCHEDULERTIME.GetMillis() - SBUS_Stored_Time > SBUS_TIMEOUT_US)
  {
    ParserState = 0;
    SBUS_Stored_Time = SCHEDULERTIME.GetMillis();
  }
  while (FASTSERIAL.Available(UART_NUMB_2) > 0)
  {
    SBUS_Stored_Time = 0;
    ActualByte = FASTSERIAL.Read(UART_NUMB_2);
    if (ParserState == 0)
    {
      if ((ActualByte == 0x0F) && ((PrevByte == 0x00) || ((PrevByte & 0x0F) == 0x04)))
      {
        ParserState++;
      }
      else
      {
        ParserState = 0;
      }
    }
    else
    {
      if ((ParserState - 1) < 24)
      {
        PayLoadArray[ParserState - 1] = ActualByte;
        ParserState++;
      }
      if ((ParserState - 1) == 24)
      {
        if ((ActualByte == 0x00) || ((ActualByte & 0x0F) == 0x04))
        {
          ParserState = 0;
          return true;
        }
        else
        {
          ParserState = 0;
          return false;
        }
      }
    }
    PrevByte = ActualByte;
  }
  return false;
}
