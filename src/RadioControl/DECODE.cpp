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
#include "FastSerial/FASTSERIAL.h"
#include "Common/VARIABLES.h"
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
    RcChannelMap[0] = ROLL;
    RcChannelMap[1] = PITCH;
    RcChannelMap[2] = THROTTLE;
    RcChannelMap[3] = YAW;
  }
  else
  { //FUTABA OU D4R-II
    RcChannelMap[0] = PITCH;
    RcChannelMap[1] = ROLL;
    RcChannelMap[2] = THROTTLE;
    RcChannelMap[3] = YAW;
  }
  RcChannelMap[4] = AUX1;
  RcChannelMap[5] = AUX2;
  RcChannelMap[6] = AUX3;
  RcChannelMap[7] = AUX4;
  RcChannelMap[8] = AUX5;
  RcChannelMap[9] = AUX6;
  RcChannelMap[10] = AUX7;
  RcChannelMap[11] = AUX8;
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
      CheckFailSafeState = SBUSRC.FailSafe || !COMMAND_ARM_DISARM;
    }
    else
    {
      CheckFailSafeState = RadioControllOutputDecoded > CHECKSUM.GetFailSafeValue || !COMMAND_ARM_DISARM;
    }
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER) ||
        (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == IBUS_RECEIVER))
    {
      if (CheckFailSafeState)
      {
        DirectRadioControllRead[Channels] = RadioControllOutputDecoded;
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
        if (RadioControllOutputMeasured < (uint16_t)DirectRadioControllRead[Channels] - 3)
        {
          DirectRadioControllRead[Channels] = RadioControllOutputMeasured + 2;
        }
        if (RadioControllOutputMeasured > (uint16_t)DirectRadioControllRead[Channels] + 3)
        {
          DirectRadioControllRead[Channels] = RadioControllOutputMeasured - 2;
        }
        RadioControllOutputTYPR[Channels][TYPRIndex] = RadioControllOutputDecoded;
      }
    }
  }
}