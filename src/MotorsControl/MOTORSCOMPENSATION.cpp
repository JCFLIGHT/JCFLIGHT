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

#include "MOTORSCOMPENSATION.h"
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "BatteryMonitor/BATTERY.h"
#include "Math/MATHSUPPORT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/PT1.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_SAGGING

#define IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH 10 //NÚMERO MINIMO DE AMOSTRAS PARA CONSIDERAR QUE A IMPEDANCIA FOI CALCULADA

uint8_t ImpedanceSampleCount = 0;

float Throttle_Compensation_Weight = 1.0f; //GANHO DA COMPENSAÇÃO DEFINIDA PELO USUARIO
float ActualBatteryVoltage = 12.60f;       //SIMULANDO O VALOR ATUAL DA BATERIA
float ActualBatteryCurrent = 0.0f;         //SIMULANDO O CONSUMO DA BATERIA
float BatteryFullVoltage = 12.60f;         //SIMULANDO 3S
float PreviousBatteryVoltage;
float PreviousAmperage;

static uint16_t SAGCompensatedVBat = 0;   //TENSÃO DA BATERIA SEM CARGA CALCULADA
static uint16_t PowerSupplyImpedance = 0; //IMPEDANCIA DA BATERIA CALCULADA EM MILLIOHM

uint32_t PreviousTime = 0;

float PT1FilterApply2(PT1_Filter_Struct *Filter, float Input, float DeltaTime)
{
  Filter->DeltaTime = DeltaTime;
  Filter->State = Filter->State + DeltaTime / (Filter->RC + DeltaTime) * (Input - Filter->State);
  return Filter->State;
}

void SaggingCompensatedUpdate()
{

  float DeltaTime = 1000 * 1e-6f;

  static PT1_Filter_Struct ImpedanceFilterState;
  static PT1_Filter_Struct SaggingCompVBatFilterState;

  ActualBatteryVoltage = BATTERY.Voltage;
  ActualBatteryCurrent = BATTERY.Total_Current;
  BatteryFullVoltage = BATTERY.Get_Max_Voltage_Calced();

  if (BatteryFullVoltage == 0)
  {
    SaggingCompVBatFilterState.State = ActualBatteryVoltage * 100;
    ImpedanceFilterState.State = 0;
    SAGCompensatedVBat = ActualBatteryVoltage * 100;
    return;
  }

  if ((SCHEDULERTIME.GetMicros() - PreviousTime) > SCHEDULER_SET_FREQUENCY(2, "Hz"))
  {
    PreviousTime = 0;
  }

  if (!PreviousTime)
  {
    PreviousAmperage = ActualBatteryCurrent;
    PreviousBatteryVoltage = ActualBatteryVoltage;
    PreviousTime = SCHEDULERTIME.GetMicros();
  }
  else if ((ActualBatteryCurrent - PreviousAmperage >= 2) &&
           (PreviousBatteryVoltage - ActualBatteryVoltage >= 0.4f)) //2A DE DIF & 0.4V DE DIF
  {

    uint16_t impedanceSample = (int32_t)((PreviousBatteryVoltage - ActualBatteryVoltage) * 100) * 1000 / ((ActualBatteryCurrent - PreviousAmperage) * 100);

    if (ImpedanceSampleCount <= IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH)
    {
      ImpedanceSampleCount += 1;
    }

    if (ImpedanceFilterState.State)
    {
      ImpedanceFilterState.RC = ImpedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH ? 1.2 : 0.5;
      PT1FilterApply2(&ImpedanceFilterState, impedanceSample, DeltaTime);
    }
    else
    {
      ImpedanceFilterState.State = impedanceSample;
    }

    if (ImpedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH)
    {
      PowerSupplyImpedance = lrintf(ImpedanceFilterState.State);
    }
  }

  uint16_t SaggingCompensatedSample = MIN(BatteryFullVoltage * 100, (ActualBatteryVoltage * 100) + (int32_t)PowerSupplyImpedance * (ActualBatteryCurrent * 100) / 1000);
  SaggingCompVBatFilterState.RC = SaggingCompensatedSample < SaggingCompVBatFilterState.State ? 40 : 500;
  SAGCompensatedVBat = lrintf(PT1FilterApply2(&SaggingCompVBatFilterState, SaggingCompensatedSample, DeltaTime));

#ifdef PRINTLN_SAGGING

  DEBUG("PowerSupplyImpedance:%u SAGCompensatedVBat:%u ActualBatteryVoltage:%.2f ActualBatteryCurrent:%.2f",
        PowerSupplyImpedance,
        SAGCompensatedVBat,
        ActualBatteryVoltage,
        ActualBatteryCurrent);

#endif
}

float CalculateThrottleCompensationFactor(void)
{
  SaggingCompensatedUpdate();
  return 1.0f + ((float)(BatteryFullVoltage * 100) / SAGCompensatedVBat - 1.0f) * Throttle_Compensation_Weight;
}