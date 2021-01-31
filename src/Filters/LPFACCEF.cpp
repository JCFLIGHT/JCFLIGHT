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

#include "LPFACCEF.h"
#include "Math/MATHSUPPORT.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

void LowPassFilter::Apply(const float Sample, float CutOff_Freq, float DeltaTime)
{
  float RecalculateCutOff = 1.0f / (6.283185307179586476925286766559f * CutOff_Freq);
  Alpha = Constrain_Float(DeltaTime / (DeltaTime + RecalculateCutOff), 0.0f, 1.0f);
  OutputFiltered += (Sample - OutputFiltered) * Alpha;
}

float LowPassFilter::GetOutputFiltered()
{
  return OutputFiltered;
}
