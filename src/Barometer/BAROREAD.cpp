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
#include "Filters/AVERAGEFILTER.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "BAROBACKEND.h"

#define BARO_SPIKES_SIZE 0x15

AverageFilterInt32_Size5 Pressure_Filter; //INSTANCIA DO FILTRO AVERAGE PARA A PRESSÃO BARO,TAMANHO = 5 ITERAÇÕES
AverageFilterInt32_Size5 Altitude_Filter; //INSTANCIA DO FILTRO AVERAGE PARA A ALTITUDE,TAMANHO = 5 ITERAÇÕES

void RecalculateBaroTotalPressure()
{
  static int32_t PressureVector[BARO_SPIKES_SIZE];
  static uint8_t PressureIndex;
  uint8_t PressureIndexCount = (PressureIndex + 1);
  if (PressureIndexCount >= BARO_SPIKES_SIZE)
  {
    PressureIndexCount = 0;
  }
  PressureVector[PressureIndex] = Pressure_Filter.Apply(Barometer.Raw.Pressure);
  Barometer.Raw.PressureFiltered += PressureVector[PressureIndex];
  Barometer.Raw.PressureFiltered -= PressureVector[PressureIndexCount];
  PressureIndex = PressureIndexCount;
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

void CalculateBarometerAltitude()
{
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    Barometer.Calibration.GroundPressure = Barometer.Raw.PressureFiltered;
    Barometer.Calibration.GroundTemperature = Barometer.Raw.Temperature;
    Altitude_Filter.Reset();
  }
  else
  {
    Barometer.Altitude.Actual = Altitude_Filter.Apply((int32_t)Get_Altitude_Difference(Barometer.Calibration.GroundPressure,
                                                                                       Barometer.Raw.PressureFiltered,
                                                                                       Barometer.Calibration.GroundTemperature));
  }
}

int32_t GetAltitudeForGCS()
{
  static uint8_t InitialSamples = 0xC8;
  static int16_t BarometerTemperatureScaleForGCS;
  static int32_t BarometerGroundPressure;

  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    InitialSamples = 0xC8; //RECALIBRA A ALTITUDE DO BARO PARA O MODO DESARMADO
    return Barometer.Altitude.Actual;
  }

  if (InitialSamples > 0)
  {
    BarometerGroundPressure = Barometer.Raw.PressureFiltered;
    BarometerTemperatureScaleForGCS = Barometer.Raw.Temperature;
    InitialSamples--;
    return 0;
  }

  return (int32_t)Get_Altitude_Difference(BarometerGroundPressure, Barometer.Raw.PressureFiltered, BarometerTemperatureScaleForGCS);
}