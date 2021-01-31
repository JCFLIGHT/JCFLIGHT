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

#include "PPM.h"
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

#define FAILSAFE_DETECT_TRESHOLD 975 //US

volatile uint16_t PPMReadChannels[12];
uint8_t PPMChannelMap[12];

void ConfigurePPMRegisters()
{
  if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != 1) || (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) != 2))
  {
    HALPPM.Initialization();
  }
  //FlySky FS-i6, FlySky FS-i6s, FlySky FS-i6x, FlySky FS-iA10B, TGY-I6(OU TGY-I6 OU FS-i6 ATUALIZADO PARA 10 CANAIS)
  if (ReceiverModel <= 7)
  {
    PPMChannelMap[0] = ROLL;
    PPMChannelMap[1] = PITCH;
    PPMChannelMap[2] = THROTTLE;
    PPMChannelMap[3] = YAW;
  }
  else
  { //FUTABA OU D4R-II
    PPMChannelMap[0] = PITCH;
    PPMChannelMap[1] = ROLL;
    PPMChannelMap[2] = THROTTLE;
    PPMChannelMap[3] = YAW;
  }
  PPMChannelMap[4] = AUX1;
  PPMChannelMap[5] = AUX2;
  PPMChannelMap[6] = AUX3;
  PPMChannelMap[7] = AUX4;
  PPMChannelMap[8] = AUX5;
  PPMChannelMap[9] = AUX6;
  PPMChannelMap[10] = AUX7;
  PPMChannelMap[11] = AUX8;
}

void DecodeAllReceiverChannels()
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
    if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 1)
    {
      CheckFailSafeState = SBUSRC.FailSafe || !COMMAND_ARM_DISARM;
    }
    else
    {
      CheckFailSafeState = RadioControllOutputDecoded > FAILSAFE_DETECT_TRESHOLD || !COMMAND_ARM_DISARM;
    }
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 1) || (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 2))
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