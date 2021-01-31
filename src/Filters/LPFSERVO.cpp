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

#include "LPFSERVO.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

void LowPassFilterCoefficient(int16_t CutOffFreq, Struct_LowPassFilter *Filter, int16_t InputBias)
{
  float FixedScaler;
  //COEFICINTENTES PARA O FILTRO PASSA BAIXA DE SEGUNDA ORDEM
  float CutOffFreqFloat = (float)CutOffFreq * 0.001f;
  float Omega = tanf((float)3.14159265358979323846f * CutOffFreqFloat / 2.0f);
  float Scaling = 1.0f / (Omega * Omega + 1.4142136f * Omega + 1.0f);

  Filter->BFilter[0] = Scaling * Omega * Omega;
  Filter->BFilter[1] = 2.0f * Filter->BFilter[0];
  Filter->BFilter[2] = Filter->BFilter[0];

  Filter->AFilter[0] = 1.0f;
  Filter->AFilter[1] = Scaling * (2.0f * Omega * Omega - 2.0f);
  Filter->AFilter[2] = Scaling * (Omega * Omega - 1.4142136f * Omega + 1.0f);

  //ESCALA PARA PONTO FIXO
  Filter->Input_Bias = InputBias;
  Filter->Input_Bit_Shift = 16;
  Filter->Coefficient_Bit_Shift = 24;
  FixedScaler = (float)(1ULL << Filter->Coefficient_Bit_Shift);
  for (uint8_t i = 0; i < 3; i++)
  {
    Filter->A_64_Bits[i] = Filter->AFilter[i] * FixedScaler < 0 ? (Filter->AFilter[i] * FixedScaler - 0.5f) : (Filter->AFilter[i] * FixedScaler + 0.5f);
    Filter->B_64_Bits[i] = Filter->BFilter[i] * FixedScaler < 0 ? (Filter->BFilter[i] * FixedScaler - 0.5f) : (Filter->BFilter[i] * FixedScaler + 0.5f);
  }
  Filter->CutOffFreq = CutOffFreq;
}

int32_t LowPassFilter(Struct_LowPassFilter *Filter, int32_t Input_Device, int16_t CutOffFreq, int16_t InputBiasSet)
{
  int16_t CofficientIndex;
  int64_t Filter_Out;
  int32_t Input_Scale;

  //CHECA SE A FREQUENCIA DE CORTE NÃO FOI ALTERADA AO LONGO DO CICLO DE MAQUINA
  if (CutOffFreq != Filter->CutOffFreq)
  {
    Filter->Init_Filter = false;
  }

  //INICIALIZA O FILTRO SE A FREQUENCIA DE CORTE FOR DIFERENTE DA ANTERIOR
  if (!Filter->Init_Filter)
  {
    LowPassFilterCoefficient(CutOffFreq, Filter, InputBiasSet);
    for (CofficientIndex = 0; CofficientIndex < 3; CofficientIndex++)
    {
      Filter->X_32Bits[CofficientIndex] = (Input_Device - Filter->Input_Bias) << Filter->Input_Bit_Shift;
      Filter->Y_32Bits[CofficientIndex] = (Input_Device - Filter->Input_Bias) << Filter->Input_Bit_Shift;
    }
    Filter->Init_Filter = true;
  }

  //ENTRADA DO AJUSTE DE BIAS E ESCALA DE ENTRADA
  Input_Scale = (Input_Device - Filter->Input_Bias) << Filter->Input_Bit_Shift;

  //TEMPO DE ESPERA DO FILTRO
  for (CofficientIndex = 2; CofficientIndex > 0; CofficientIndex--)
  {
    Filter->X_32Bits[CofficientIndex] = Filter->X_32Bits[CofficientIndex - 1];
    Filter->Y_32Bits[CofficientIndex] = Filter->Y_32Bits[CofficientIndex - 1];
  }
  Filter->X_32Bits[0] = Input_Scale;

  //ACUMULA O RESULTADO
  Filter_Out = Filter->X_32Bits[0] * Filter->B_64_Bits[0];
  for (CofficientIndex = 1; CofficientIndex < 3; CofficientIndex++)
  {
    Filter_Out -= Filter->Y_32Bits[CofficientIndex] * Filter->A_64_Bits[CofficientIndex];
    Filter_Out += Filter->X_32Bits[CofficientIndex] * Filter->B_64_Bits[CofficientIndex];
  }

  //ESCALA DE SAIDA DADA PELO COEFFCIENT SHIFT
  Filter_Out >>= Filter->Coefficient_Bit_Shift;
  Filter->Y_32Bits[0] = (int32_t)Filter_Out;

  //ESCALA DE SAIDA DADA PELO BIT SHIFT INPUT
  Filter_Out = (Filter_Out + (1 << (Filter->Input_Bit_Shift - 1))) >> Filter->Input_Bit_Shift;

  //REAPLICA O AJUSTE DE BIAS
  Filter_Out += Filter->Input_Bias;

  //RETORNA O VALOR FILTRADO
  return (int32_t)Filter_Out;
}
