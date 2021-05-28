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
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "BAROBACKEND.h"
#include "Math/MATHSUPPORT.h"
#include "Build/BOARDDEFS.h"

#define BARO_SPIKES_SIZE 0x15

#ifndef USE_BARO_PRECISE_MATH

void Remove_Barometer_Spikes(void)
{
  static int32_t PressureVector[BARO_SPIKES_SIZE];
  static uint8_t PressureIndex;
  uint8_t PressureIndexCount = PressureIndex + 1;

  if (PressureIndexCount >= BARO_SPIKES_SIZE)
  {
    PressureIndexCount = 0;
  }

  PressureVector[PressureIndex] = Barometer.Raw.Pressure;
  Barometer.Raw.PressureFiltered += PressureVector[PressureIndex];
  Barometer.Raw.PressureFiltered -= PressureVector[PressureIndexCount];
  PressureIndex = PressureIndexCount;
}

#else

#define BARO_SAMPLE_COUNT_MAX 48
#define PRESSURE_SAMPLES_MEDIAN 3

//FILTRO COM 3 MEDIAS
int32_t ApplyQuickMedianFilterSuperSoft(int32_t *VectorBuffer)
{
  int32_t Points[3];

  for (int32_t IndexCount = 0; IndexCount < 3; IndexCount++)
  {
    Points[IndexCount] = VectorBuffer[IndexCount];
  }

  if (Points[0] > Points[1])
  {
    int32_t StoredPoint = Points[0];
    Points[0] = Points[1];
    Points[1] = StoredPoint;
  }

  if (Points[1] > Points[2])
  {
    int32_t StoredPoint = Points[1];
    Points[1] = Points[2];
    Points[2] = StoredPoint;
  }

  if (Points[0] > Points[1])
  {
    int32_t StoredPoint = Points[0];
    Points[0] = Points[1];
    Points[1] = StoredPoint;
  }

  return Points[1];
}

static int32_t ApplyBarometerMedianFilter(int32_t newPressureReading)
{
  static int32_t BarometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
  static int CurrentFilterSampleIndex = 0;
  static bool MedianFilterReady = false;
  int NextSampleIndex;

  NextSampleIndex = (CurrentFilterSampleIndex + 1);

  if (NextSampleIndex == PRESSURE_SAMPLES_MEDIAN)
  {
    NextSampleIndex = 0;
    MedianFilterReady = true;
  }

  BarometerFilterSamples[CurrentFilterSampleIndex] = newPressureReading;
  CurrentFilterSampleIndex = NextSampleIndex;

  if (MedianFilterReady)
  {
    return ApplyQuickMedianFilterSuperSoft(BarometerFilterSamples);
  }
  return newPressureReading;
}

void Remove_Barometer_Spikes(void)
{
  static int32_t BarometerSamples[BARO_SAMPLE_COUNT_MAX + 1];
  static int CurrentSampleIndex = 0;
  int16_t NextSampleIndex;

  if (CurrentSampleIndex >= BARO_SPIKES_SIZE)
  {
    NextSampleIndex = 0;
  }
  else
  {
    NextSampleIndex = (CurrentSampleIndex + 1);
  }

  BarometerSamples[CurrentSampleIndex] = ApplyBarometerMedianFilter(Barometer.Raw.Pressure);

  Barometer.Raw.PressureFiltered += BarometerSamples[CurrentSampleIndex];
  Barometer.Raw.PressureFiltered -= BarometerSamples[NextSampleIndex];

  CurrentSampleIndex = NextSampleIndex;
}

#endif

float Get_Altitude_Difference(float Base_Pressure, float Pressure, float BaroTemperature)
{
  float Result;

#ifndef USE_BARO_PRECISE_MATH

  //EM CPU MAIS LENTA USE UM CÁLCULO MENOS EXATO,PORÉM MAIS RÁPIDO
  float Scaling = Base_Pressure / Pressure;
  float CalcedTemperature = BaroTemperature + 273.15f;
  Result = logf(Scaling) * CalcedTemperature * 29.271267f;

#else

  //EM CPUs MAIS RÁPIDAS USE UM CÁLCULO MAIS EXATO
  float Scaling = Pressure / Base_Pressure;
  float CalcedTemperature = BaroTemperature + 273.15f;
  //ESTE É UM CÁLCULO EXATO QUE ESTÁ DENTRO DE +/- 2.5M DAS TABELAS DE ATMOSFERA PADRÃO NA TROPOSFERA (ATÉ 11.000M AMSL)
  Result = 153.8462f * CalcedTemperature * (1.0f - expf(0.190259f * logf(Scaling)));

#endif

  return Result;
}

void Calculate_Barometer_Altitude(void)
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Barometer.Calibration.GroundPressure = Barometer.Raw.PressureFiltered * 0.01f;
    Barometer.Calibration.GroundTemperature = ConvertCentiDegreesToDegrees(Barometer.Raw.Temperature);
  }
  else
  {
    Barometer.Altitude.Actual = ConverMetersToCM(Get_Altitude_Difference(Barometer.Calibration.GroundPressure,
                                                                         Barometer.Raw.PressureFiltered * 0.01f,
                                                                         Barometer.Calibration.GroundTemperature));
  }
}