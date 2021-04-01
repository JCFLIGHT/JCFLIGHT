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

#include "THRCOMPENSATION.h"
#include "Math/MATHSUPPORT.h"
#include "BatteryMonitor/BATTERY.h"
#include "Math/MATHSUPPORT.h"
#include "Scheduler/SCHEDULER.h"
#include "Filters/PT1.h"
#include "FastSerial/PRINTF.h"

//DEBUG
//#define PRINTLN_SAGGING

#define IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH 10 //NÚMERO MINIMO DE AMOSTRAS PARA CONSIDERAR QUE A IMPEDANCIA FOI CALCULADA

uint8_t ImpedanceSampleCount = 0;

float Throttle_Compensation_Weight = 1.0f; //GANHO DA COMPENSAÇÃO DEFINIDA PELO USUARIO
float BatteryFullVoltage;
float ActualBatteryVoltage;
float ActualBatteryCurrent;
float PreviousBatteryVoltage;
float PreviousAmperage;

static uint16_t SaggingCompensatedVBat = 0; //TENSÃO DA BATERIA SEM CARGA CALCULADA (SEM A INTERFERENCIA DO CONSUMO DOS MOTORES)
static uint16_t PowerSupplyImpedance = 0;   //IMPEDANCIA DA BATERIA CALCULADA EM MILLIOHM

void SaggingCompensatedUpdate(float DeltaTime)
{
  static PT1_Filter_Struct ImpedanceFilterState;
  static PT1_Filter_Struct SaggingCompVBatFilterState;

  ActualBatteryVoltage = BATTERY.Get_Actual_Voltage();
  ActualBatteryCurrent = BATTERY.Get_Actual_Current();
  BatteryFullVoltage = BATTERY.Get_Max_Voltage_Calced();

  if (BatteryFullVoltage == 0)
  {
    SaggingCompVBatFilterState.State = ActualBatteryVoltage * 100;
    SaggingCompensatedVBat = ActualBatteryVoltage * 100;
    ImpedanceFilterState.State = 0;
    return;
  }

  static Scheduler_Struct SaggingCompensatedTimer;
  if (Scheduler(&SaggingCompensatedTimer, SCHEDULER_SET_FREQUENCY(2, "Hz"))) //OBTÉM NOVAS AMOSTRAS A CADA MEIO SEGUNDO
  {
    PreviousAmperage = ActualBatteryCurrent;
    PreviousBatteryVoltage = ActualBatteryVoltage;
  }
  else if ((ActualBatteryCurrent - PreviousAmperage >= 2) &&
           (PreviousBatteryVoltage - ActualBatteryVoltage >= 0.4f)) //2A DE DIF & 0.4V DE DIF
  {

    uint16_t ImpedanceSample = (int32_t)((PreviousBatteryVoltage - ActualBatteryVoltage) * 100) * 1000 / ((ActualBatteryCurrent - PreviousAmperage) * 100);

    if (ImpedanceSampleCount <= IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH)
    {
      ImpedanceSampleCount += 1;
    }

    if (ImpedanceFilterState.State)
    {
      ImpedanceFilterState.RC = ImpedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH ? 1.2 : 0.5;
      PT1FilterApply2(&ImpedanceFilterState, ImpedanceSample, DeltaTime);
    }
    else
    {
      ImpedanceFilterState.State = ImpedanceSample;
    }

    if (ImpedanceSampleCount > IMPEDANCE_STABLE_SAMPLE_COUNT_THRESH)
    {
      PowerSupplyImpedance = lrintf(ImpedanceFilterState.State);
    }
  }

  uint16_t SaggingCompensatedSample = MIN(BatteryFullVoltage * 100, (ActualBatteryVoltage * 100) + (int32_t)PowerSupplyImpedance * (ActualBatteryCurrent * 100) / 1000);
  SaggingCompVBatFilterState.RC = SaggingCompensatedSample < SaggingCompVBatFilterState.State ? 40 : 500;
  SaggingCompensatedVBat = lrintf(PT1FilterApply2(&SaggingCompVBatFilterState, SaggingCompensatedSample, DeltaTime));

#ifdef PRINTLN_SAGGING

  DEBUG("PowerSupplyImpedance:%u SaggingCompensatedVBat:%u ActualBatteryVoltage:%.2f ActualBatteryCurrent:%.2f",
        PowerSupplyImpedance,
        SaggingCompensatedVBat,
        ActualBatteryVoltage,
        ActualBatteryCurrent);

#endif
}

float CalculateThrottleCompensationFactor(float DeltaTime)
{
  SaggingCompensatedUpdate(DeltaTime);
  return 1.0f + ((float)(BatteryFullVoltage * 100) / SaggingCompensatedVBat - 1.0f) * Throttle_Compensation_Weight;
}