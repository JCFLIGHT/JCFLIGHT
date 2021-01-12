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

#include "BAROREAD.h"
#include "Common/VARIABLES.h"
#include "Filters/AVERAGEFILTER.h"

float BarometerTemperatureScale;
float BarometerPressureSum;

int32_t BaroPressureRaw;
int16_t BaroTemperatureRaw;
int32_t BaroPressureToLogarithm;

AverageFilterInt32_Size5 Baro_Filter; //ISTANCIA DO FILTRO AVERAGE PARA O BARO,TAMANHO = 5 ITERAÇÕES

void Baro_Calibration()
{
  static int32_t BaroVector[21];
  static uint8_t BaroIndex;
  uint8_t IndexFilter = (BaroIndex + 1);
  if (IndexFilter == 21)
  {
    IndexFilter = 0;
  }
  BaroVector[BaroIndex] = BaroPressureRaw;
  BaroPressureToLogarithm += BaroVector[BaroIndex];
  BaroPressureToLogarithm -= BaroVector[IndexFilter];
  BaroIndex = IndexFilter;
}

void CalculateBaroAltitude()
{
  if (!COMMAND_ARM_DISARM)
  {
    BarometerPressureSum = log(BaroPressureToLogarithm);
    BarometerTemperatureScale = ((int32_t)BaroTemperatureRaw + 27315) * 29.271267f;
    Baro_Filter.Reset(); //RESETA O FILTRO AVERAGE PARA EVITAR ALTOS DROPS DE VALORES
  }
  else
  {
    ALTITUDE.RealBaroAltitude = Baro_Filter.Apply((BarometerPressureSum - log(BaroPressureToLogarithm)) * BarometerTemperatureScale);
  }
}

int32_t GetAltitudeForGCS()
{
  static uint8_t InitialSamples = 0xB4;
  static float BarometerTemperatureScaleTwo;
  static float BarometerPressureSumTwo;

  if (COMMAND_ARM_DISARM)
  {
    return ALTITUDE.RealBaroAltitude;
  }

  if (InitialSamples > 0)
  {
    BarometerPressureSumTwo = log(BaroPressureToLogarithm);
    BarometerTemperatureScaleTwo = ((int32_t)BaroTemperatureRaw + 27315) * 29.271267f;
    InitialSamples--;
    return 0;
  }
  else
  {
    return ((BarometerPressureSumTwo - log(BaroPressureToLogarithm)) * BarometerTemperatureScaleTwo);
  }
}