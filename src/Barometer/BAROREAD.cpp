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

float BarometerGroundPressureForFlight;
float BarometerTemperatureScale;

int16_t BaroTemperatureRaw;
int32_t BaroPressureRaw;
int32_t BaroPressureFiltered;

AverageFilterInt32_Size5 Baro_Filter; //ISTANCIA DO FILTRO AVERAGE PARA O BARO,TAMANHO = 5 ITERAÇÕES

void Baro_AverageFilter()
{
  static int32_t BaroVector[0x15];
  static uint8_t BaroIndex;
  uint8_t IndexFilter = (BaroIndex + 1);
  if (IndexFilter == 0x15)
  {
    IndexFilter = 0;
  }
  BaroVector[BaroIndex] = BaroPressureRaw;
  BaroPressureFiltered += BaroVector[BaroIndex];
  BaroPressureFiltered -= BaroVector[IndexFilter];
  BaroIndex = IndexFilter;
}

void CalculateBaroAltitudeForFlight()
{
  if (!COMMAND_ARM_DISARM)
  {
    DoBaroCalibrationForFlight();
    Baro_Filter.Reset(); //RESETA O FILTRO AVERAGE PARA EVITAR ALTOS DROPS DE VALORES
  }
  else
  {
    ALTITUDE.RealBaroAltitude = Baro_Filter.Apply(Get_Altitude_Difference(BarometerGroundPressureForFlight, BaroPressureFiltered, BarometerTemperatureScale));
  }
}

void DoBaroCalibrationForFlight()
{
  BarometerGroundPressureForFlight = BaroPressureFiltered;
  BarometerTemperatureScale = BaroTemperatureRaw;
}

float Get_Altitude_Difference(float Base_Pressure, int32_t Pressure, int16_t BaroTemperature)
{
  float Result;
#if __AVR_ATmega2560__
  //EM CPU MAIS LENTA USE UM CÁLCULO MENOS EXATO,PORÉM MAIS RÁPIDO
  float Scaling = Base_Pressure / Pressure;
  int16_t CalcedTemperature = BaroTemperature + 27315;
  Result = logf(Scaling) * CalcedTemperature * 29.271267f;
#else
  //EM CPUs MAIS RÁPIDAS USE UM CÁLCULO MAIS EXATO
  float Scaling = Pressure / Base_Pressure;
  int16_t CalcedTemperature = BaroTemperature + 27315;
  //ESTE É UM CÁLCULO EXATO QUE ESTÁ DENTRO DE +/- 2.5M DAS TABELAS DE ATMOSFERA PADRÃO NA TROPOSFERA (ATÉ 11.000M AMSL)
  Result = 153.8462f * CalcedTemperature * (1.0f - expf(0.190259f * logf(Scaling)));
#endif
  return Result;
}

int32_t GetAltitudeForGCS()
{
  static uint16_t InitialSamples = 0x1F4;
  static int16_t BarometerTemperatureScaleForGCS;
  static int32_t BarometerGroundPressure;

  if (COMMAND_ARM_DISARM)
  {
    return ALTITUDE.RealBaroAltitude;
  }

  if (InitialSamples > 0)
  {
    BarometerGroundPressure = BaroPressureFiltered;
    BarometerTemperatureScaleForGCS = BaroTemperatureRaw;
    InitialSamples--;
    return 0;
  }

  return Get_Altitude_Difference(BarometerGroundPressure, BaroPressureFiltered, BarometerTemperatureScaleForGCS);
}