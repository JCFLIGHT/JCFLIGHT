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

#include "BIQUADFILTER.h"
#include "Math/MATHSUPPORT.h"

void BiQuadFilter::Settings(int16_t CutOffFreq, int16_t SampleFreq, uint8_t FilterType)
{
  //CALCULA A FREQUENCIA DE CORTE
  //A FREQUENCIA DE CORTE TEM QUE SER METADE DO RATE DA FUNÇÃO APLICADA
  float COF = MIN_FLOAT(float(CutOffFreq), float((SampleFreq / 2) - 1));
  //GERA OS COEFICIENTES DO FILTRO BIQUADRATICO
  float Omega = tan(3.1415927f * COF / SampleFreq);
  float Normal = 1.0f / (1.0f + Omega / 0.7071f + Omega * Omega);
  switch (FilterType)
  {

  case LPF:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO LOW PASS
    Coeff1 = int16_t(Omega * Omega * Normal * 16384.0f);
    Coeff2 = 2 * Coeff1;
    Coeff3 = Coeff1;
    Coeff4 = int16_t(2.0f * (Omega * Omega - 1.0f) * Normal * 16384.0f);
    Coeff5 = int16_t((1.0f - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f);
    break;

  case HPF:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO HIGH PASS
    Coeff1 = 1 * Normal * 16384.0f;
    Coeff2 = -2 * Coeff1;
    Coeff3 = Coeff1;
    Coeff4 = 2 * (Omega * Omega - 1) * Normal * 16384.0f;
    Coeff5 = (1 - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f;
    break;

  case NOTCH:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO NOTCH
    Coeff1 = (1 + Omega * Omega) * Normal * 16384.0f;
    Coeff2 = 2 * (Omega * Omega - 1) * Normal * 16384.0f;
    Coeff3 = Coeff1;
    Coeff4 = Coeff2;
    Coeff5 = (1 - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f;
    break;
  }
}

int16_t BiQuadFilter::FilterOutput(int16_t DeviveToFilter)
{
  //ENTRADA DO DISPOSITIVO PARA O FILTRO
  Result.LongValue = DeviveToFilter * Coeff1 + GuardInput1 * Coeff2 + GuardInput2 * Coeff3 - GuardOutput1 * Coeff4 - GuardOutput2 * Coeff5;
  Result.LongValue = Result.LongValue << 2;
  GuardInput2 = GuardInput1;
  GuardInput1 = DeviveToFilter;
  GuardOutput2 = GuardOutput1;
  GuardOutput1 = Result.ShortValue[1];
  return Result.ShortValue[1];
}
