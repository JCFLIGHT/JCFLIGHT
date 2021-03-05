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
#include "Filters/AVERAGEFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "BATTLEVELS.h"
#include "Build/BOARDDEFS.h"
#include "Math/MATHSUPPORT.h"
#include "FailSafe/FAILSAFE.h"
#include "FastSerial/PRINTF.h"

BATT BATTERY;

AverageFilterFloat_Size20 Voltage_Filter; //INSTANCIA DO FILTRO AVERAGE PARA A TENSÃO,TAMANHO = 20 ITERAÇÕES
AverageFilterFloat_Size20 Current_Filter; //INSTANCIA DO FILTRO AVERAGE PARA A CORRENTE,TAMANHO = 20 ITERAÇÕES

//DEBUG
//#define PRINTLN_BATT

#define THIS_LOOP_RATE 50            //HZ
#define TIMER_TO_AUTO_DETECT_BATT 15 //SEGUNDOS
#define LOW_BATT_DETECT_OVERFLOW 4   //SEGUNDOS
#define PREVENT_ARM_LOW_BATT 20      //PREVINE A CONTROLADORA DE ARMAR SE A BATERIA ESTIVER ABAIXO DE 20% DA CAPACIDADE

//VALORES DE CALIBRAÇÃO PARA O MODULO DA 3DR
float BattVoltageFactor = 259.489f; //VALOR DE CALIBRAÇÃO PARA O DIVISOR RESISTIVO COM R1 DE 13.7K E R2 DE 1.5K
float Amps_Per_Volt = 62.0f;        //FATOR DE MULTIPLICAÇÃO DA TENSÃO DO PINO ANALOGICO PARA CONVERTER EM CORRENTE (AMPERES)
float Amps_OffSet = 0.00f;          //TENSÃO DE OFFSET (AJUSTE FINO DA CORRENTE)

void BATT::Update_Voltage(void)
{
  //FILTRO COMPLEMENTAR PARA REDUÇÃO DE NOISE NA LEITURA DA TENSÃO (10 BITS ADC É TERRIVEL)
  BATTERY.Voltage = Voltage_Filter.Apply(BATTERY.Voltage * 0.92f + (float)(ANALOGSOURCE.Read(ADC_BATTERY_VOLTAGE) / BattVoltageFactor));
  //TENSÃO DA BATERIA ACIMA DE 6V?SIM...
  if (BATTERY.Voltage > 6)
  {
    if (BATTERY.GetPercentage() <= PREVENT_ARM_LOW_BATT) //MENOR OU IGUAL QUE 20% DA CAPACIDADE TOTAL DA BATERIA?SIM...
    {
      if (LowBatteryCount >= (THIS_LOOP_RATE * LOW_BATT_DETECT_OVERFLOW))
      {
        BATTERY.LowBattPreventArm = true;
        if (BEEPER.GetSafeStateToOthersBeeps())
        {
          BEEPER.Play(BEEPER_BATT_CRIT_LOW);
        }
      }
      else
      {
        LowBatteryCount++;
      }
    }
    else
    {
      BATTERY.LowBattPreventArm = false;
      LowBatteryCount = 0;
    }
  }
  else
  {
    BATTERY.LowBattPreventArm = false;
  }
  FailSafe_Do_RTH_With_Low_Batt(BATTERY.LowBattPreventArm);
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

#ifdef PRINTLN_BATT
  PRINTF.SendToConsole(PSTR("volt:%0.2f min:%0.2f max:%0.2f mincount:%d maxcount:%d\n"),
                       BattVoltage,
                       BattMinVolt,
                       BattMaxVolt,
                       BATTERY.BattMinCount,
                       BATTERY.BattMaxCount);
#endif

  return 100 * BATTERY.Percentage;
}

float BATT::AutoBatteryMin(float BattVoltage)
{
  if (BattVoltage > 6)
  {
    if (BATTERY.BattMinVoltageSelect == BATTERY_3S)
    {
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (BATTERY.BattMinVoltageSelect == BATTERY_4S)
    {
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (BATTERY.BattMinVoltageSelect == BATTERY_6S)
    {
      return BATT_6S_LOW_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (3.6 x 3 = 10.8v)
    {
      if (BATTERY.BattMinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BATTERY.BattMinVoltageSelect = BATTERY_3S;
      }
      else
      {
        BATTERY.BattMinCount++;
      }
      return BATT_3S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (3.6 x 4 = 14.4v)
    {
      if (BATTERY.BattMinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BATTERY.BattMinVoltageSelect = BATTERY_4S;
      }
      else
      {
        BATTERY.BattMinCount++;
      }
      return BATT_4S_LOW_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (3.6 x 6 = 21.6v)
    {
      if (BATTERY.BattMinCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BATTERY.BattMinVoltageSelect = BATTERY_6S;
      }
      else
      {
        BATTERY.BattMinCount++;
      }
      return BATT_6S_LOW_VOLTAGE;
    }
  }
  BATTERY.BattMinCount = 0;
  BATTERY.BattMinVoltageSelect = 0;
  return NONE_BATTERY;
}

float BATT::AutoBatteryMax(float BattVoltage)
{
  if (BattVoltage > 6)
  {
    if (BATTERY.BattMaxVoltageSelect == BATTERY_3S)
    {
      return BATT_3S_HIGH_VOLTAGE;
    }
    else if (BATTERY.BattMaxVoltageSelect == BATTERY_4S)
    {
      return BATT_4S_HIGH_VOLTAGE;
    }
    else if (BATTERY.BattMaxVoltageSelect == BATTERY_6S)
    {
      return BATT_6S_HIGH_VOLTAGE;
    }
    if (BattVoltage > BATT_3S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_3S_SAFE_HIGH_VOLTAGE) //BATERIA 3S (4.2 x 3 = 12.6v)
    {
      if (BattMaxCount >= THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT)
      {
        BATTERY.BattMaxVoltageSelect = BATTERY_3S;
      }
      else
      {
        BattMaxCount++;
      }
      return BATT_3S_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_4S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_4S_SAFE_HIGH_VOLTAGE) //BATERIA 4S (4.2 x 4 = 16.8v)
    {
      if (BattMaxCount >= (THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT))
      {
        BATTERY.BattMaxVoltageSelect = BATTERY_4S;
      }
      else
      {
        BattMaxCount++;
      }
      return BATT_4S_HIGH_VOLTAGE;
    }
    else if (BattVoltage > BATT_6S_SAFE_LOW_VOLTAGE && BattVoltage < BATT_6S_SAFE_HIGH_VOLTAGE) //BATERIA 6S (4.2 x 6 = 25.2v)
    {
      if (BattMaxCount >= (THIS_LOOP_RATE * TIMER_TO_AUTO_DETECT_BATT))
      {
        BATTERY.BattMaxVoltageSelect = BATTERY_6S;
      }
      else
      {
        BattMaxCount++;
      }
      return BATT_6S_HIGH_VOLTAGE;
    }
  }
  BattMaxCount = 0;
  BATTERY.BattMaxVoltageSelect = 0;
  return NONE_BATTERY;
}

float BATT::Get_Max_Voltage_Calced()
{
  return BATTERY.AutoBatteryMax(BATTERY.Voltage);
}

uint8_t BATT::GetPercentage()
{
  return BATTERY.CalculatePercentage(BATTERY.Voltage, BATTERY.AutoBatteryMin(BATTERY.Voltage), BATTERY.AutoBatteryMax(BATTERY.Voltage));
}

void BATT::Update_Current(void)
{
  //FAZ A LEITURA DO SENSOR DE CORRENTE
  BATTERY.Total_Current = Current_Filter.Apply(((ANALOGSOURCE.Read(ADC_BATTERY_CURRENT)) - Amps_OffSet) * Amps_Per_Volt);
  BATTERY.Total_Current = MAX(0, BATTERY.Total_Current);
}

void BATT::Calculate_Total_Mah(void)
{
  uint32_t TimeNow = SCHEDULERTIME.GetMicros();
  static uint32_t Last_Time_Stored;
  float Delta_Time = TimeNow - Last_Time_Stored;
  if (Last_Time_Stored != 0 && Delta_Time < 2000000.0f)
  {
    //0.0002778 É 1/3600 (CONVERSÃO PRA HORAS)
    BATTERY.TotalCurrentInMah += BATTERY.Total_Current * Delta_Time * 0.0000002778f;
  }
  Last_Time_Stored = TimeNow;
}

float BATT::Get_Current_In_Mah()
{
  return (BATTERY.TotalCurrentInMah / 1000);
}

uint32_t BATT::GetWatts()
{
  //NO GCS,O RESULTADO DESTA OPERAÇÃO É DIVIDO POR 1000,AFIM DE OBTER OS VALORES DECIMAIS,
  //POR ESSE FATO ESSA FUNÇÃO ESTÁ EM 32 BITS
  return (BATTERY.Total_Current * BATTERY.Voltage);
}