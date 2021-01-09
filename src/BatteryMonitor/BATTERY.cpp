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
#include "Common/VARIABLES.h"
#include "Filters/AVERAGEFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "BATTLEVELS.h"
#include "Build/BOARDDEFS.h"

BATT BATTERY;

AverageFilterFloat_Size12 Voltage_Filter; //ISTANCIA DO FILTRO AVERAGE PARA A TENSÃO,TAMANHO = 12 ITERAÇÕES
AverageFilterFloat_Size12 Current_Filter; //ISTANCIA DO FILTRO AVERAGE PARA A CORRENTE,TAMANHO = 12 ITERAÇÕES

#define THIS_LOOP_RATE 100          //HZ
#define TIMER_TO_AUTO_DETECT_BATT 3 //SEGUNDOS
#define PREVENT_ARM_LOW_BATT 20     //PREVINE A CONTROLADORA DE ARMAR SE A BATERIA ESTIVER ABAIXO DE 20% DA CAPACIDADE

//VALORES DE CALIBRAÇÃO PARA O MODULO DA 3DR
float BattVoltageFactor = 237.489f; //VALOR DE CALIBRAÇÃO PARA O DIVISOR RESISTIVO COM R1 DE 13.7K E R2 DE 1.5K
float Amps_Per_Volt = 17.0f;        //FATOR DE DIVISÃO DA TENSÃO DO PINO ANALOGICO PARA CONVERTER EM CORRENTE (AMPERES)
float Amps_OffSet = 0.00f;          //TENSÃO DE OFFSET (AJUSTE FINO DA CORRENTE)

void BATT::Read_Voltage(void)
{
  //FILTRO COMPLEMENTAR PARA REDUÇÃO DE NOISE NA LEITURA DA TENSÃO (10 BITS ADC É TERRIVEL)
  Voltage = Voltage_Filter.Apply(Voltage * 0.92f + (float)(ADCPIN.Read(ADC_BATTERY_VOLTAGE) / BattVoltageFactor));
  //TENSÃO DA BATERIA ACIMA DE 5V?SIM...
  if (Voltage > 5)
  {
    if (BATTERY.GetPercentage() < PREVENT_ARM_LOW_BATT) //MENOR QUE 20%
    {
      LowBattPreventArm = true;
      if (BEEPER.SafeToOthersBeepsCounter > 200)
      {
        BEEPER.Play(BEEPER_BAT_CRIT_LOW);
      }
    }
    else
    {
      LowBattPreventArm = false;
    }
  }
  else
  {
    LowBattPreventArm = false;
  }
}

uint8_t BATT::CalculatePercentage(float BattVoltage, float BattMinVolt, float BattMaxVolt)
{
  BATTERY.Percentage = (BattVoltage - BattMinVolt) / (BattMaxVolt - BattMinVolt);
  if (BATTERY.Percentage < 0)
  {
    BATTERY.Percentage = 0;
  }
  else if (BATTERY.Percentage > 1)
  {
    BATTERY.Percentage = 1;
  }
  return 100 * BATTERY.Percentage;
}

float BATT::AutoBatteryMin(float BattVoltage)
{
  if (BattVoltage > 5)
  {
    if (BattMinVoltageSelect == BATTERY_3S)
    {
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (BattMinVoltageSelect == BATTERY_4S)
    {
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (BattMinVoltageSelect == BATTERY_6S)
    {
      return BATT_6S_LOW_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (3.6 x 3 = 10.8v)
    {
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMinVoltageSelect = BATTERY_3S;
      }
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (3.6 x 4 = 14.4v)
    {
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMinVoltageSelect = BATTERY_4S;
      }
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (3.6 x 6 = 21.6v)
    {
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMinVoltageSelect = BATTERY_6S;
      }
      return BATT_6S_LOW_VOLTAGE;
    }
  }
  BattMinCount = 0;
  BattMinVoltageSelect = 0;
  return NONE_BATTERY;
}

float BATT::AutoBatteryMax(float BattVoltage)
{
  if (BattVoltage > 5)
  {
    if (BattMaxVoltageSelect == BATTERY_3S)
    {
      return BATT_3S_SAFE_HIGH_VOLTAGE;
    }
    else if (BattMaxVoltageSelect == BATTERY_4S)
    {
      return BATT_4S_SAFE_HIGH_VOLTAGE;
    }
    else if (BattMaxVoltageSelect == BATTERY_6S)
    {
      return BATT_6S_SAFE_HIGH_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (4.2 x 3 = 12.6v)
    {
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMaxVoltageSelect = BATTERY_3S;
      }
      return BATT_3S_SAFE_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (4.2 x 4 = 16.8v)
    {
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMaxVoltageSelect = BATTERY_4S;
      }
      return BATT_4S_SAFE_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (4.2 x 6 = 25.2v)
    {
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BattMaxVoltageSelect = BATTERY_6S;
      }
      return BATT_6S_SAFE_HIGH_VOLTAGE;
    }
  }
  BattMaxCount = 0;
  BattMaxVoltageSelect = 0;
  return NONE_BATTERY;
}

uint8_t BATT::GetPercentage()
{
  return BATTERY.CalculatePercentage(Voltage, BATTERY.AutoBatteryMin(Voltage), BATTERY.AutoBatteryMax(Voltage));
}

void BATT::Do_RTH_With_Low_Batt(bool FailSafeBatt)
{
  //ENTRA EM MODO RTH OU LAND (SE ESTIVER PROXIMO DO HOME-POINT)
  //SE A BATERIA ESTIVER COM A TENSÃO ABAIXO DE 20% DA CARGA TOTAL
  static bool FailSafeBattDetect = false;
  if (FailSafeBatt)
  {
    //EVITA COM QUE UMA TROCA RAPIDA DE TRUE PARA FALSE OCORRA
    //A FIM DE NÃO INTERFERIR NO FUNCIONAMENTO DESSA FUNÇÃO
    FailSafeBattDetect = true;
  }
  if (!COMMAND_ARM_DISARM)
  {
    FailSafeBattDetect = false;
    return;
  }
  if (!FailSafeBattDetect)
  {
    return;
  }
}

void BATT::Read_Current(void)
{
  //FAZ A LEITURA DO SENSOR DE CORRENTE
  Total_Current = ((ADCPIN.Read(ADC_BATTERY_CURRENT)) - Amps_OffSet) * Amps_Per_Volt;
}

void BATT::Calculate_Total_Mah(void)
{
  uint32_t TimeNow = SCHEDULER.GetMicros();
  static uint32_t Last_Time_Stored;
  float Delta_Time = TimeNow - Last_Time_Stored;
  if (Last_Time_Stored != 0 && Delta_Time < 2000000.0f)
  {
    //0.0002778 É 1/3600 (CONVERSÃO PRA HORAS)
    TotalCurrentInMah += Total_Current * Delta_Time * 0.0000002778f;
  }
  Last_Time_Stored = TimeNow;
}

float BATT::Get_Current_In_Mah()
{
  return (TotalCurrentInMah / 1000);
}

uint32_t BATT::GetWatts()
{
  //NO GCS,O RESULTADO DESTA OPERAÇÃO É DIVIDO POR 1000,AFIM DE OBTER OS VALORES DECIMAIS,
  //POR ESSE FATO ESSA FUNÇÃO ESTÁ EM 32 BITS
  return (Total_Current * Voltage);
}