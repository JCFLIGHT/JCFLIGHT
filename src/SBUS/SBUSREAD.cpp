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
#include "StorageManager/EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "Common/ENUM.h"
#include "FailSafe/FAILSAFE.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

SBUS SBUSRC;

//DEBUG
//#define PRINTLN_MODE

bool LostFrame = false;
uint16_t SBUSReadChannels[12];

void SBUS::Update()
{
  if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != SBUS_RECEIVER)
  {
    return;
  }
  SBUSRC.Read(&SBUSReadChannels[0], &SBUSRC.FailSafe, &LostFrame);
#if defined(PRINTLN_MODE)
  static uint32_t SBUS_Serial_Refresh;
  if (SCHEDULERTIME.GetMillis() - SBUS_Serial_Refresh >= 20)
  {
    PRINTF.SendToConsole(ProgmemString("Thr:%d Yaw:%d Pitch:%d Roll:%d Aux1:%d Aux2:%d Aux3:%d Aux4:%d Aux5:%d Aux6:%d Aux7:%d Aux8:%d FailSafe:%d\n"),
                         SBUSReadChannels[0], SBUSReadChannels[1], SBUSReadChannels[2], SBUSReadChannels[3], SBUSReadChannels[4],
                         SBUSReadChannels[5], SBUSReadChannels[6], SBUSReadChannels[7], SBUSRC.FailSafe);
    SBUS_Serial_Refresh = SCHEDULERTIME.GetMillis();
  }
#endif
}

void SBUS::Read(uint16_t *ChannelsRead, bool *FailSafe, bool *LostFrame)
{
  if (SBUSRC.SerialParse())
  {
    if (ChannelsRead)
    {
      ChannelsRead[0] = (uint16_t)((SBUSRC.PayLoadArray[0] | SBUSRC.PayLoadArray[1] << 8) & 0x07FF) / 2 + 988;
      ChannelsRead[1] = (uint16_t)((SBUSRC.PayLoadArray[1] >> 3 | SBUSRC.PayLoadArray[2] << 5) & 0x07FF) / 2 + 988;
      ChannelsRead[2] = (uint16_t)((SBUSRC.PayLoadArray[2] >> 6 | SBUSRC.PayLoadArray[3] << 2 | SBUSRC.PayLoadArray[4] << 10) & 0x07FF) / 2 + 988;
      ChannelsRead[3] = (uint16_t)((SBUSRC.PayLoadArray[4] >> 1 | SBUSRC.PayLoadArray[5] << 7) & 0x07FF) / 2 + 988;
      ChannelsRead[4] = (uint16_t)((SBUSRC.PayLoadArray[5] >> 4 | SBUSRC.PayLoadArray[6] << 4) & 0x07FF) / 2 + 988;
      ChannelsRead[5] = (uint16_t)((SBUSRC.PayLoadArray[6] >> 7 | SBUSRC.PayLoadArray[7] << 1 | SBUSRC.PayLoadArray[8] << 9) & 0x07FF) / 2 + 988;
      ChannelsRead[6] = (uint16_t)((SBUSRC.PayLoadArray[8] >> 2 | SBUSRC.PayLoadArray[9] << 6) & 0x07FF) / 2 + 988;
      ChannelsRead[7] = (uint16_t)((SBUSRC.PayLoadArray[9] >> 5 | SBUSRC.PayLoadArray[10] << 3) & 0x07FF) / 2 + 988;
      ChannelsRead[8] = (uint16_t)((SBUSRC.PayLoadArray[11] | SBUSRC.PayLoadArray[12] << 8) & 0x07FF) / 2 + 988;
      ChannelsRead[9] = (uint16_t)((SBUSRC.PayLoadArray[12] >> 3 | SBUSRC.PayLoadArray[13] << 5) & 0x07FF) / 2 + 988;
      ChannelsRead[10] = (uint16_t)((PayLoadArray[13] >> 6 | SBUSRC.PayLoadArray[14] << 2 | SBUSRC.PayLoadArray[15] << 10) & 0x07FF) / 2 + 988;
      ChannelsRead[11] = (uint16_t)((SBUSRC.PayLoadArray[15] >> 1 | SBUSRC.PayLoadArray[16] << 7) & 0x07FF) / 2 + 988;
      ChannelsRead[12] = (uint16_t)((SBUSRC.PayLoadArray[16] >> 4 | SBUSRC.PayLoadArray[17] << 4) & 0x07FF) / 2 + 988;
      ChannelsRead[13] = (uint16_t)((SBUSRC.PayLoadArray[17] >> 7 | SBUSRC.PayLoadArray[18] << 1 | SBUSRC.PayLoadArray[19] << 9) & 0x07FF) / 2 + 988;
      ChannelsRead[14] = (uint16_t)((SBUSRC.PayLoadArray[19] >> 2 | SBUSRC.PayLoadArray[20] << 6) & 0x07FF) / 2 + 988;
      ChannelsRead[15] = (uint16_t)((SBUSRC.PayLoadArray[20] >> 5 | SBUSRC.PayLoadArray[21] << 3) & 0x07FF) / 2 + 988;
    }
    if (LostFrame)
    {
      if (SBUSRC.PayLoadArray[22] & 0x04)
      {
        *LostFrame = true;
      }
      else
      {
        *LostFrame = false;
      }
    }
    if (SBUSRC.FailSafe)
    {
      if (SBUSRC.PayLoadArray[22] & 0x08)
      {
        *FailSafe = true;
      }
      else
      {
        *FailSafe = false;
        if (Fail_Safe_System_Count > 20)
        {
          Fail_Safe_System_Count -= 20;
        }
        else
        {
          Fail_Safe_System_Count = 0;
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
    SBUSRC.ParserState = 0;
    SBUS_Stored_Time = SCHEDULERTIME.GetMillis();
  }
  while (FASTSERIAL.Available(UART_NUMB_2) > 0)
  {
    SBUS_Stored_Time = 0;
    SBUSRC.ActualByte = FASTSERIAL.Read(UART_NUMB_2);
    if (SBUSRC.ParserState == 0)
    {
      if ((SBUSRC.ActualByte == 0x0F) && ((SBUSRC.PrevByte == 0x00) || ((SBUSRC.PrevByte & 0x0F) == 0x04)))
      {
        SBUSRC.ParserState++;
      }
      else
      {
        SBUSRC.ParserState = 0;
      }
    }
    else
    {
      if ((SBUSRC.ParserState - 1) < 24)
      {
        PayLoadArray[SBUSRC.ParserState - 1] = SBUSRC.ActualByte;
        SBUSRC.ParserState++;
      }
      if ((SBUSRC.ParserState - 1) == 24)
      {
        if ((SBUSRC.ActualByte == 0x00) || ((SBUSRC.ActualByte & 0x0F) == 0x04))
        {
          SBUSRC.ParserState = 0;
          return true;
        }
        else
        {
          SBUSRC.ParserState = 0;
          return false;
        }
      }
    }
    SBUSRC.PrevByte = SBUSRC.ActualByte;
  }
  return false;
}
