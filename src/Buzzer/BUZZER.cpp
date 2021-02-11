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

#include "BUZZER.h"
#include "Common/VARIABLES.h"
#include "RadioControl/STICKS.h"
#include "EscCalibration/CALIBESC.h"
#include "BatteryMonitor/BATTERY.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/BOARDDEFS.h"
#include "Common/STRUCTS.h"

BEEPERCLASS BEEPER;

#define BEEPER_COMMAND_STOP 0xFF

bool BuzzerInit = false;
bool SafeToOthersBeeps = false;

static const uint8_t AlgorithmInit_Beep[] = {
    19, 5,
    9, 5,
    9, 5,
    19, 5,
    9, 5,
    0, 39,
    19, 5,
    19, 5, BEEPER_COMMAND_STOP};

static const uint8_t Arm_Beep[] = {
    245, BEEPER_COMMAND_STOP};

static const uint8_t Disarm_Beep[] = {
    15, 5, 15, 5, BEEPER_COMMAND_STOP};

static const uint8_t LowBattery_Beep[] = {
    50, 2, BEEPER_COMMAND_STOP};

static const uint8_t Success_Beep[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP};

static const uint8_t Fail_Beep[] = {
    20, 15, 35, 5, BEEPER_COMMAND_STOP};

static const uint8_t Calibration_Beep[] = {
    18, 8, 18, 8, 18, 8, BEEPER_COMMAND_STOP};

static const uint8_t AutoLaunch_Beep[] = {
    5, 5, 5, 100, BEEPER_COMMAND_STOP};

static const uint8_t Launched_Beep[] = {
    245, BEEPER_COMMAND_STOP};

static const uint8_t FMU_Init_Beep[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP};

static const uint8_t FMU_Safe_Beep[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP};

static const uint8_t Fail_Safe_Beep[] = {
    50, 50, BEEPER_COMMAND_STOP};

static uint8_t BeeperState = 0;
static uint16_t BeeperPositionArray = 0;
static uint32_t BeeperNextNote = 0;

const Struct_BeeperEntry BeeperTable[] = {
    {BEEPER_CALIBRATION_DONE, 0, Calibration_Beep},
    {BEEPER_DISARMING, 1, Disarm_Beep},
    {BEEPER_BAT_CRIT_LOW, 2, LowBattery_Beep},
    {BEEPER_ACTION_SUCCESS, 3, Success_Beep},
    {BEEPER_ACTION_FAIL, 4, Fail_Beep},
    {BEEPER_ARM, 5, Arm_Beep},
    {BEEPER_ALGORITHM_INIT, 6, AlgorithmInit_Beep},
    {BEEPER_AUTOLAUNCH, 7, AutoLaunch_Beep},
    {BEEPER_LAUNCHED, 8, Launched_Beep},
    {BEEPER_FMU_INIT, 9, FMU_Init_Beep},
    {BEEPER_FMU_SAFE_TO_ARM, 10, FMU_Safe_Beep},
    {BEEPER_FAIL_SAFE, 11, Fail_Safe_Beep}};

static const Struct_BeeperEntry *BeeperEntry = NULL;

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(BeeperTable) / sizeof(Struct_BeeperEntry))

void BEEPERCLASS::Play(Beeper_Mode Mode)
{
  if (STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == OFF_ALL_DISP ||
      STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == ONLY_SWITCH)
  {
    return;
  }
  const Struct_BeeperEntry *SelectedSong = NULL;
  for (uint8_t i = 0; i < BEEPER_TABLE_ENTRY_COUNT; i++)
  {
    const Struct_BeeperEntry *SelectedSongTable = &BeeperTable[i];
    if (SelectedSongTable->Mode != Mode)
      continue;
    if (!BeeperEntry)
    {
      SelectedSong = SelectedSongTable;
      break;
    }
    if (SelectedSongTable->Priority < BeeperEntry->Priority)
      SelectedSong = SelectedSongTable;
    break;
  }
  if (!SelectedSong)
    return;
  BeeperEntry = SelectedSong;
  BeeperPositionArray = 0;
  BeeperNextNote = 0;
}

void BEEPERCLASS::Silence()
{
  BEEP_OFF;
  BeeperState = 0;
  BeeperNextNote = 0;
  BeeperPositionArray = 0;
  BeeperEntry = NULL;
  SafeToOthersBeeps = true;
}

void BEEPERCLASS::Update()
{
  if (BeeperEntry == NULL)
  {
    return;
  }
  if (BeeperNextNote > SCHEDULERTIME.GetMicros() / 1000)
  {
    return;
  }
  if (!BeeperState)
  {
    BeeperState = 1;
    if (BeeperEntry->Sequence[BeeperPositionArray] != 0)
      BEEP_ON;
  }
  else
  {
    BeeperState = 0;
    if (BeeperEntry->Sequence[BeeperPositionArray] != 0)
      BEEP_OFF;
  }
  ProcessCommand();
}

void BEEPERCLASS::ProcessCommand()
{
  if (BeeperEntry->Sequence[BeeperPositionArray] == BEEPER_COMMAND_STOP)
  {
    Silence();
  }
  else
  {
    BeeperNextNote = SCHEDULERTIME.GetMillis() + 10 * BeeperEntry->Sequence[BeeperPositionArray];
    BeeperPositionArray++;
  }
}

void BEEPERCLASS::Run()
{
  if (STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == OFF_ALL_DISP ||
      STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == ONLY_SWITCH)
  {
    Silence();
    return;
  }
  if (!BuzzerInit)
  {
    BEEP_PINOUT;
#if defined ESP32
    AnalogWriteSetSettings(GPIO_NUM_18, 490, 12);
#endif
  }
  if (ESC.BeeperMode == ESC_CALIBRATION_MODE)
  {
    Play(BEEPER_CALIBRATION_DONE);
  }
  else if (ESC.BeeperMode == NORMAL_OPERATION_MODE)
  {
    if (!BuzzerInit)
    {
      Play(BEEPER_ALGORITHM_INIT);
    }
  }
  //NÃO FAÇA A MUDANÇA DO SOM TÃO RAPIDO PARA NÃO EMBOLAR COM O BEEP DA BATERIA SE A MESMA ESTIVER COM BAIXA TENSÃO
  if (SafeToOthersBeeps && SafeToOthersBeepsCounter < 254)
  {
    SafeToOthersBeepsCounter++;
  }
  Update();
  if (!BuzzerInit)
  {
    BuzzerInit = true;
  }
}
