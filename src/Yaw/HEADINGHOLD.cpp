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

#include "HEADINGHOLD.h"
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "FastSerial/PRINTF.h"

//#define DEBUG
#define HEADING_HOLD_ERROR_LPF_FREQ 2

PT1_Filter_Struct HeadingHoldRateFilter;

void UpdateStateOfHeadingHold(void)
{
  if (!COMMAND_ARM_DISARM)
  {
    HeadingHoldRateFilter.State = 0.0f;         //RESETA O FILTRO
    HeadingHoldTarget = ATTITUDE.AngleOut[YAW]; //OBTÉM UM NOVO VALOR INICIAL PARA HEADING HOLD TARGET
  }
}

bool GetSafeStateOfHeadingHold()
{
  if (!Do_HeadingHold_Mode) //NÃO APLICA A CORREÇÃO DO YAW SE O MODO NÃO ESTIVER ATIVO
  {
    return false;
  }

  if (AHRS.CheckAnglesInclination(25)) //NÃO APLICA A CORREÇÃO DO YAW SE OS ANGULOS FOREM MAIOR QUE 25 GRAUS
  {
    return false;
  }

  if (GPS_Flight_Mode == Do_None) //NÃO APLICA A CORREÇÃO DO YAW SE NENHUM MODO DE VOO USANDO O GPS ESTIVER ATIVO
  {
    return false;
  }

  if (ABS_16BITS(RCController[YAW]) != 0) //NÃO APLICA A CORREÇÃO DO YAW SE O USUARIO MANIPULAR O STICK YAW DO RADIO
  {
    return false;
  }

  return true; //TUDO ESTÁ OK
}

float GetHeadingHoldValue()
{
  uint8_t Heading_Hold_Rate_Limit = 90;
  float HeadingHoldRate;

  int16_t YawError = ATTITUDE.AngleOut[YAW] - HeadingHoldTarget; //CALCULA O ERRO / DIFERENÇA

  //CALCULA O VALOR RELATIVO DO ERRO/DIFERENÇA E CONVERTE PARA UM VALOR ACEITAVEL POR HEADING
  if (YawError <= -180)
  {
    YawError += 360;
  }

  if (YawError >= +180)
  {
    YawError -= 360;
  }

  //CALCULA O VALOR DO RATE
  HeadingHoldRate = YawError * PID[PIDYAWVELOCITY].ProportionalVector / 30.0f;
  //APLICA LIMITES MIN E MAX NO RATE
  HeadingHoldRate = Constrain_Float(HeadingHoldRate, -Heading_Hold_Rate_Limit, Heading_Hold_Rate_Limit);

#ifdef DEBUG
  float HeadingHoldRateNF = HeadingHoldRate;
#endif

//REALIZA FILTRAGEM DO RATE COM O PT1
#ifndef __AVR_ATmega2560__
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRateFilter, HeadingHoldRate, HEADING_HOLD_ERROR_LPF_FREQ, Loop_Integral_Time * 1e-6f);
#else
  //DEVIDO O CICLO DE MAQUINA EM 100HZ,NÃO É POSSIVEL FILTRAR A 2HZ USANDO O VALOR MEDIDO DO PROPRIO CICLO DE MAQUINA
  //É NECESSARIO "ENGANAR" O ALGORITIMO,E FINGIR QUE ELE ESTÁ TRABALHANDO A 1KHZ PARA FREQUENCIA DE CORTE DE 2HZ SER APLICADA CORRETAMENTE
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRateFilter, HeadingHoldRate, HEADING_HOLD_ERROR_LPF_FREQ, 1.0f / 1000);
#endif

#ifdef DEBUG
  PRINTF.SendToConsole(PSTR("HHR:%.2f HHRF:%.2f\n"),
                       HeadingHoldRateNF,
                       HeadingHoldRate);
#endif

  //APLICA O CONTROLE DO YAW
  return HeadingHoldRate;
}