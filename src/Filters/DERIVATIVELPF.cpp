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

#include "DERIVATIVELPF.h"
#include "Scheduler/SCHEDULERTIME.h"

bool Reset_I = false;

int16_t DiscreteLPF_To_Derivative_PID(int16_t DerivativeInput, int16_t CutOff)
{
  //FILTRO PASSA BAIXA DISCRETO
  //FILTRA O NOISE DA ALTA FREQUENCIA,ASSIM EVITANDO O DERIVATIVO DO CONTROLADOR PID DE FICAR LOUCO
  //http://en.wikipedia.org/wiki/Low-pass_filter.
  uint8_t Frequency_CutOff = CutOff;
  int16_t Derivative_To_LPF = DerivativeInput;
  static float Store_Previous_Derivative;
  static uint32_t Store_Previous_Time;
  uint32_t LPF_StoreTime = SCHEDULER.GetMillis();
  uint32_t LPF_DeltaTime = LPF_StoreTime - Store_Previous_Time;
  float LPF_Delta_Time;
  Store_Previous_Time = LPF_StoreTime;
  LPF_Delta_Time = (float)LPF_DeltaTime / 1000.0f;
  if (Store_Previous_Time == 0 || LPF_DeltaTime > 1000)
  {
    LPF_DeltaTime = 0;
    Reset_I = true; //RESETA O INTEGRAL DO CONTROLADOR PID
  }
  else
    Reset_I = false;
  float CutOffEquation = 1 / (2 * PI * Frequency_CutOff);
  Derivative_To_LPF = Store_Previous_Derivative + ((LPF_Delta_Time / (CutOffEquation + LPF_Delta_Time)) *
                                                   (Derivative_To_LPF - Store_Previous_Derivative));
  Store_Previous_Derivative = Derivative_To_LPF;
  return Derivative_To_LPF;
}
