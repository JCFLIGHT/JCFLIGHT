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

#include "BATTERY.h"
#include "AnalogDigitalConverter/ADC.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "Build/BOARDDEFS.h"
#include "Math/MATHSUPPORT.h"
#include "FailSafe/FAILSAFE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "FastSerial/PRINTF.h"

BatteryClass BATTERY;
Battery_Struct Battery;

PT1_Filter_Struct BattVoltage_Smooth;
PT1_Filter_Struct BattCurrent_Smooth;

//DEBUG
//#define PRINTLN_BATT

#define THIS_LOOP_RATE 50            //HZ
#define TIMER_TO_AUTO_DETECT_BATT 15 //TEMPO EM SEGUNDOS PARA AUTO DETECTAR O NÚMERO DE CELULAS DA BATERIA
#define LOW_BATT_DETECT_EXHAUSTED 10 //VALIDA QUE REALMENTE A BATERIA ESTÁ ABAIXO DA CAPACIDADE MINIMA POR 'N' SEGUNDOS
#define VOLTAGE_CUTOFF 1             //FREQUENCIA DE CORTE DO FILTRO LPF DA LEITURA DA TENSÃO EM HZ
#define CURRENT_CUTOFF 1             //FREQUENCIA DE CORTE DO FILTRO LPF DA LEITURA DA CORRENTE EM HZ

//PARAMETROS PARA AS CELULAS DA BATERIA
#define BATT_LOW_CELLS Battery.Param.Min_Cell_Volt
#define BATT_HIGH_CELLS Battery.Param.Max_Cell_Volt

//PARAMETROS DE TENSÃO PARA BATERIA DE 3 CELULAS
#define BATT_3S_LOW_VOLTAGE 3 * BATT_LOW_CELLS
#define BATT_3S_HIGH_VOLTAGE 3 * BATT_HIGH_CELLS
#define BATT_3S_SAFE_LOW_VOLTAGE 10.0f
#define BATT_3S_SAFE_HIGH_VOLTAGE 13.0f

//PARAMETROS DE TENSÃO PARA BATERIA DE 4 CELULAS
#define BATT_4S_LOW_VOLTAGE 4 * BATT_LOW_CELLS
#define BATT_4S_HIGH_VOLTAGE 4 * BATT_HIGH_CELLS
#define BATT_4S_SAFE_LOW_VOLTAGE 14.0f
#define BATT_4S_SAFE_HIGH_VOLTAGE 17.5f

//PARAMETROS DE TENSÃO PARA BATERIA DE 6 CELULAS
#define BATT_6S_LOW_VOLTAGE 6 * BATT_LOW_CELLS
#define BATT_6S_HIGH_VOLTAGE 6 * BATT_HIGH_CELLS
#define BATT_6S_SAFE_LOW_VOLTAGE 20.0f
#define BATT_6S_SAFE_HIGH_VOLTAGE 26.0f

void BatteryClass::Initialization(void)
{
  Battery.Param.Voltage_Factor = STORAGEMANAGER.Read_16Bits(BATT_VOLTAGE_FACTOR_ADDR) / 100.0f;
  Battery.Param.Amps_Per_Volt = STORAGEMANAGER.Read_16Bits(BATT_AMPS_VOLT_ADDR) / 100.0f;
  Battery.Param.Amps_OffSet = STORAGEMANAGER.Read_16Bits(BATT_AMPS_OFFSET_ADDR) / 100.0f;
  Battery.Param.Min_Cell_Volt = STORAGEMANAGER.Read_16Bits(BATT_MIN_VOLTAGE_ADDR) / 100.0f;
  Battery.Param.Max_Cell_Volt = STORAGEMANAGER.Read_16Bits(BATT_MAX_VOLTAGE_ADDR) / 100.0f;
  Battery.Param.NumberOfCells = STORAGEMANAGER.Read_8Bits(BATT_NUMBER_OF_CELLS_ADDR);
  Battery.Param.CriticPercent = STORAGEMANAGER.Read_8Bits(BATT_CRIT_PERCENT_ADDR);
  Battery.Param.Do_RTH = STORAGEMANAGER.Read_8Bits(BATT_RTH_LOW_BATT_ADDR);
  PT1FilterInit(&BattVoltage_Smooth, VOLTAGE_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
  PT1FilterInit(&BattCurrent_Smooth, CURRENT_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
}

void BatteryClass::Update_Voltage(void)
{
  Battery.Calced.Voltage = PT1FilterApply3(&BattVoltage_Smooth, (float)(ANALOGSOURCE.Read_Voltage_Ratiometric(ADC_BATTERY_VOLTAGE) * Battery.Param.Voltage_Factor));
  BATTERY.Update_Exhausted();
  if (Battery.Param.Do_RTH)
  {
    FailSafe_Do_RTH_With_Low_Batt(Battery.Exhausted.LowPercentPreventArm);
  }
}

uint8_t BatteryClass::CalculatePercentage(float BattVoltage, float BattMinVolt, float BattMaxVolt)
{
  Battery.Calced.Percentage = (BattVoltage - BattMinVolt) / (BattMaxVolt - BattMinVolt);

  if (Battery.Calced.Percentage < 0)
  {
    Battery.Calced.Percentage = 0;
  }
  else if (Battery.Calced.Percentage > 1)
  {
    Battery.Calced.Percentage = 1;
  }

#ifdef PRINTLN_BATT

  DEBUG("Volts:%.2f Min:%.2f Max:%.2f MinCount:%u MaxCount:%u",
        BattVoltage,
        BattMinVolt,
        BattMaxVolt,
        Battery.Auto.MinCount,
        Battery.Auto.MaxCount);

#endif

  return 100 * Battery.Calced.Percentage;
}

float BatteryClass::AutoBatteryMin(float BattVoltage)
{
  if (BattVoltage > 6.0f && Battery.Param.NumberOfCells == AUTO_BATTERY)
  {
    if (Battery.Auto.MinVoltageType == BATTERY_3S)
    {
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (Battery.Auto.MinVoltageType == BATTERY_4S)
    {
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (Battery.Auto.MinVoltageType == BATTERY_6S)
    {
      return BATT_6S_LOW_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (3.6 x 3 = 10.8v)
    {
      if (Battery.Auto.MinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        Battery.Auto.MinVoltageType = BATTERY_3S;
      }
      else
      {
        Battery.Auto.MinCount++;
      }
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (3.6 x 4 = 14.4v)
    {
      if (Battery.Auto.MinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        Battery.Auto.MinVoltageType = BATTERY_4S;
      }
      else
      {
        Battery.Auto.MinCount++;
      }
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (3.6 x 6 = 21.6v)
    {
      if (Battery.Auto.MinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        Battery.Auto.MinVoltageType = BATTERY_6S;
      }
      else
      {
        Battery.Auto.MinCount++;
      }
      return BATT_6S_LOW_VOLTAGE;
    }
  }
  else
  {
    if (Battery.Param.NumberOfCells == BATTERY_3S)
    {
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (Battery.Param.NumberOfCells == BATTERY_4S)
    {
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (Battery.Param.NumberOfCells == BATTERY_6S)
    {
      return BATT_6S_LOW_VOLTAGE;
    }
  }
  Battery.Auto.MinCount = 0;
  Battery.Auto.MinVoltageType = 0;
  return NONE_BATTERY;
}

float BatteryClass::AutoBatteryMax(float BattVoltage)
{
  if (BattVoltage > 6.0f && Battery.Param.NumberOfCells == AUTO_BATTERY)
  {
    if (Battery.Auto.MaxVoltageType == BATTERY_3S)
    {
      return BATT_3S_HIGH_VOLTAGE;
    }
    else if (Battery.Auto.MaxVoltageType == BATTERY_4S)
    {
      return BATT_4S_HIGH_VOLTAGE;
    }
    else if (Battery.Auto.MaxVoltageType == BATTERY_6S)
    {
      return BATT_6S_HIGH_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (4.2 x 3 = 12.6v)
    {
      if (Battery.Auto.MaxCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        Battery.Auto.MaxVoltageType = BATTERY_3S;
      }
      else
      {
        Battery.Auto.MaxCount++;
      }
      return BATT_3S_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (4.2 x 4 = 16.8v)
    {
      if (Battery.Auto.MaxCount >= (THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT))
      {
        Battery.Auto.MaxVoltageType = BATTERY_4S;
      }
      else
      {
        Battery.Auto.MaxCount++;
      }
      return BATT_4S_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (4.2 x 6 = 25.2v)
    {
      if (Battery.Auto.MaxCount >= (THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT))
      {
        Battery.Auto.MaxVoltageType = BATTERY_6S;
      }
      else
      {
        Battery.Auto.MaxCount++;
      }
      return BATT_6S_HIGH_VOLTAGE;
    }
  }
  else
  {
    if (Battery.Param.NumberOfCells == BATTERY_3S)
    {
      return BATT_3S_HIGH_VOLTAGE;
    }
    else if (Battery.Param.NumberOfCells == BATTERY_4S)
    {
      return BATT_4S_HIGH_VOLTAGE;
    }
    else if (Battery.Param.NumberOfCells == BATTERY_6S)
    {
      return BATT_6S_HIGH_VOLTAGE;
    }
  }
  Battery.Auto.MaxCount = 0;
  Battery.Auto.MaxVoltageType = 0;
  return NONE_BATTERY;
}

float BatteryClass::Get_Max_Voltage_Calced(void)
{
  return BATTERY.AutoBatteryMax(Battery.Calced.Voltage);
}

uint8_t BatteryClass::GetPercentage(void)
{
  return BATTERY.CalculatePercentage(Battery.Calced.Voltage,
                                     BATTERY.AutoBatteryMin(Battery.Calced.Voltage),
                                     BATTERY.AutoBatteryMax(Battery.Calced.Voltage));
}

void BatteryClass::Update_Current(void)
{
  //FAZ A LEITURA DO SENSOR DE CORRENTE
  Battery.Calced.Current = PT1FilterApply3(&BattCurrent_Smooth, (ANALOGSOURCE.Read(ADC_BATTERY_CURRENT) - Battery.Param.Amps_OffSet) * Battery.Param.Amps_Per_Volt);
  Battery.Calced.Current = MAX(Battery.Calced.Current, 0);
}

void BatteryClass::Calculate_Total_Current_In_Mah(void)
{
  uint32_t TimeNow = SCHEDULERTIME.GetMicros();
  static uint32_t Last_Time_Stored;
  float Delta_Time = TimeNow - Last_Time_Stored;
  if (Last_Time_Stored != 0 && Delta_Time < 2000000.0f)
  {
    //0.0002778 É 1/3600 (CONVERSÃO PRA HORAS)
    Battery.Calced.CurrentInMah += Battery.Calced.Current * Delta_Time * 0.0000002778f;
  }
  Last_Time_Stored = TimeNow;
}

float BatteryClass::Get_Actual_Voltage(void)
{
  return Battery.Calced.Voltage;
}

float BatteryClass::Get_Actual_Current(void)
{
  return Battery.Calced.Current;
}

float BatteryClass::Get_Current_In_Mah(void)
{
  return (Battery.Calced.CurrentInMah / 1000);
}

uint32_t BatteryClass::GetWatts(void)
{
  //NO GCS,O RESULTADO DESTA OPERAÇÃO É DIVIDO POR 1000,AFIM DE OBTER OS VALORES DECIMAIS,
  //POR ESSE FATO ESSA FUNÇÃO ESTÁ EM 32 BITS
  return (Battery.Calced.Current * Battery.Calced.Voltage);
}

//VALIDA QUE REALMENTE A BATERIA ESTÁ ABAIXO DA CAPACIDADE MINIMA POR 'N' SEGUNDOS
void BatteryClass::Update_Exhausted(void)
{
  if (Battery.Calced.Voltage > 6.0f && Battery.Param.CriticPercent != 0) //TENSÃO DA BATERIA ACIMA DE 6V?SIM...
  {
    if (BATTERY.GetPercentage() <= Battery.Param.CriticPercent) //MENOR OU IGUAL QUE 'N%' DA CAPACIDADE TOTAL DA BATERIA?SIM...
    {
      if (Battery.Exhausted.LowBatteryCount >= (THIS_LOOP_RATE * LOW_BATT_DETECT_EXHAUSTED)) //CHECA SE NÃO É APENAS UM PICO POR 'N' SEGUNDOS
      {
        Battery.Exhausted.LowPercentPreventArm = true;
        BEEPER.Play(BEEPER_BATT_CRIT_LOW);
      }
      else
      {
        Battery.Exhausted.LowBatteryCount++;
      }
    }
    else
    {
      Battery.Exhausted.LowPercentPreventArm = false;
      Battery.Exhausted.LowBatteryCount = 0;
    }
  }
  else
  {
    Battery.Exhausted.LowPercentPreventArm = false;
  }
}

bool BatteryClass::GetExhausted(void)
{
  return Battery.Exhausted.LowPercentPreventArm;
}