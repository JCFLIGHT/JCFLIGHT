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

#include "DECODE.h"
#include "Common/STRUCTS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "SBUS/SBUSREAD.h"
#include "IBUS/IBUSREAD.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "HAL/LEARNINGRECEIVER.h"
#include "HAL/HALPPM.h"
#include "ParamsToGCS/CHECKSUM.h"

DecodeClass DECODE;

void DecodeClass::Initialization()
{
  if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != SBUS_RECEIVER) ||
      (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != IBUS_RECEIVER))
  {
    HALPPM.Initialization();
  }
  //FlySky FS-i6, FlySky FS-i6s, FlySky FS-i6x, FlySky FS-iA10B, TGY-I6(OU TGY-I6 OU FS-i6 ATUALIZADO PARA 10 CANAIS)
  if (ReceiverModel <= 7)
  {
    DECODE.RcChannelMap[0] = ROLL;
    DECODE.RcChannelMap[1] = PITCH;
    DECODE.RcChannelMap[2] = THROTTLE;
    DECODE.RcChannelMap[3] = YAW;
  }
  else
  { //FUTABA OU D4R-II
    DECODE.RcChannelMap[0] = PITCH;
    DECODE.RcChannelMap[1] = ROLL;
    DECODE.RcChannelMap[2] = THROTTLE;
    DECODE.RcChannelMap[3] = YAW;
  }
  DECODE.RcChannelMap[4] = AUX1;
  DECODE.RcChannelMap[5] = AUX2;
  DECODE.RcChannelMap[6] = AUX3;
  DECODE.RcChannelMap[7] = AUX4;
  DECODE.RcChannelMap[8] = AUX5;
  DECODE.RcChannelMap[9] = AUX6;
  DECODE.RcChannelMap[10] = AUX7;
  DECODE.RcChannelMap[11] = AUX8;
}

void DecodeClass::Update()
{
  bool CheckFailSafeState = true;
  static uint8_t TYPRIndex = 0;
  static uint16_t RadioControllOutputTYPR[12][3];
  uint16_t RadioControllOutputMeasured;
  uint16_t RadioControllOutputDecoded;

  TYPRIndex++;
  if (TYPRIndex == 3)
  {
    TYPRIndex = 0;
  }

  for (uint8_t Channels = 0; Channels < 12; Channels++)
  {
    RadioControllOutputDecoded = LearningChannelsOfReceiver(Channels);
    if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER)
    {
      CheckFailSafeState = SBUSRC.FailSafe || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    }
    else
    {
      CheckFailSafeState = RadioControllOutputDecoded > CHECKSUM.GetFailSafeValue || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    }
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER) ||
        (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == IBUS_RECEIVER))
    {
      if (CheckFailSafeState)
      {
        DECODE.DirectRadioControllRead[Channels] = RadioControllOutputDecoded;
      }
    }
    else
    {
      if (CheckFailSafeState)
      {
        RadioControllOutputMeasured = RadioControllOutputDecoded;
        for (uint8_t TYPR = 0; TYPR < 3; TYPR++)
        {
          RadioControllOutputMeasured += RadioControllOutputTYPR[Channels][TYPR];
        }
        RadioControllOutputMeasured = (RadioControllOutputMeasured + 2) / 4;
        if (RadioControllOutputMeasured < (uint16_t)DECODE.DirectRadioControllRead[Channels] - 3)
        {
          DECODE.DirectRadioControllRead[Channels] = RadioControllOutputMeasured + 2;
        }
        if (RadioControllOutputMeasured > (uint16_t)DECODE.DirectRadioControllRead[Channels] + 3)
        {
          DECODE.DirectRadioControllRead[Channels] = RadioControllOutputMeasured - 2;
        }
        RadioControllOutputTYPR[Channels][TYPRIndex] = RadioControllOutputDecoded;
      }
    }
  }
}

/*
void DecodeClass::Update()
{
  bool CheckFailSafeState = true;
  static int16_t RadioControllReadAverage[12][4];
  static int16_t RCAverageIndex = 0;
  uint16_t RadioControllOutputDecoded;

  for (uint8_t Channels = 0; Channels < 12; Channels++)
  {
    RadioControllOutputDecoded = LearningChannelsOfReceiver(Channels);
    if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER)
    {
      CheckFailSafeState = SBUSRC.FailSafe || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    }
    else
    {
      CheckFailSafeState = RadioControllOutputDecoded > CHECKSUM.GetFailSafeValue || !IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    }
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER) ||
        (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == IBUS_RECEIVER))
    {
      if (CheckFailSafeState)
      {
        DECODE.DirectRadioControllRead[Channels] = RadioControllOutputDecoded;
      }
    }
    else
    {
      if (CheckFailSafeState)
      {
        RadioControllReadAverage[Channels][RCAverageIndex % 4] = RadioControllOutputDecoded;
        DECODE.DirectRadioControllRead[Channels] = 0;
        for (uint8_t TYPR = 0; TYPR < 4; TYPR++)
        {
          DECODE.DirectRadioControllRead[Channels] = DECODE.DirectRadioControllRead[Channels] + RadioControllReadAverage[Channels][TYPR];
        }
        DECODE.DirectRadioControllRead[Channels] = DECODE.DirectRadioControllRead[Channels] / 4;
      }
    }
  }
  RCAverageIndex++;
}
*/
int16_t DecodeClass::GetRxChannelOutput(uint8_t Channel)
{
  return DECODE.RadioControllOutput[Channel];
}

void DecodeClass::SetRxChannelInput(uint8_t Channel, int16_t Value)
{
  DECODE.RadioControllOutput[Channel] = Value;
}