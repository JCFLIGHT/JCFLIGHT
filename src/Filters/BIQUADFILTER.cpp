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
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

BiQuadFilter BIQUADFILTER;

float FilterGetLPF_Quality()
{
  return 1.0f / sqrtf(2.0f);
}

float FilterGetNotch_Quality(float CenterFrequencyHz, float CutOffFrequencyHz)
{
  return CenterFrequencyHz * CutOffFrequencyHz / (CenterFrequencyHz * CenterFrequencyHz - CutOffFrequencyHz * CutOffFrequencyHz);
}

void BiQuadFilter::Settings(BiquadFilter_Struct *Filter, int16_t FilterFreq, int16_t CutOffFreq, int16_t SampleInterval, uint8_t FilterType)
{

  float Q_Quality = 0;

  if (FilterType == LPF)
  {
    Q_Quality = FilterGetLPF_Quality();
  }
  else //NOTCH
  {
    Q_Quality = FilterGetNotch_Quality(FilterFreq, CutOffFreq);
  }

  float Beta0, Beta1, Beta2;
  const float SampleRate = 1.0f / ((float)SampleInterval * 0.000001f);
  const float Omega = 2.0f * 3.14159265358979323846f * ((float)FilterFreq) / SampleRate;
  const float CalcedSine = Fast_Sine(Omega);
  const float CalcedCosine = Fast_Cosine(Omega);
  const float Alpha = CalcedSine / (2 * Q_Quality);

  switch (FilterType)
  {
  case LPF:
    Beta0 = (1 - CalcedCosine) / 2;
    Beta1 = 1 - CalcedCosine;
    Beta2 = (1 - CalcedCosine) / 2;
    break;

  case NOTCH:
    Beta0 = 1;
    Beta1 = -2 * CalcedCosine;
    Beta2 = 1;
    break;
  }
  const float Alpha0 = 1 + Alpha;
  const float Alpha1 = -2 * CalcedCosine;
  const float Alpha2 = 1 - Alpha;
  Filter->Beta0 = Beta0 / Alpha0;
  Filter->Beta1 = Beta1 / Alpha0;
  Filter->Beta2 = Beta2 / Alpha0;
  Filter->Alpha1 = Alpha1 / Alpha0;
  Filter->Alpha2 = Alpha2 / Alpha0;
  Filter->SampleX1 = Filter->SampleX2 = 0;
  Filter->SampleY1 = Filter->SampleY2 = 0;
}

float BiQuadFilter::FilterApplyAndGet(BiquadFilter_Struct *Filter, float DeviceToFilter)
{
  const float Result = Filter->Beta0 * DeviceToFilter + Filter->SampleX1;
  Filter->SampleX1 = Filter->Beta1 * DeviceToFilter - Filter->Alpha1 * Result + Filter->SampleX2;
  Filter->SampleX2 = Filter->Beta2 * DeviceToFilter - Filter->Alpha2 * Result;
  return Result;
}