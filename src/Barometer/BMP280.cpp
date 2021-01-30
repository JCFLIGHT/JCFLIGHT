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

#include "BMP280.h"
#include "I2C/I2C.h"
#include "BAROREAD.h"

static union
{
  uint8_t BMP280_CalculationArray[0x1A];
  struct
  {
    //CONSTRUIDO A PARTIR DA TABELA 17 DO DATASHEET
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
  };
} BMP280_Calculation;

int64_t BMP280_Pressure;

static void BMP280_Get_Calibration(void)
{
  I2C.RegisterBuffer(0x76, 0x88, BMP280_Calculation.BMP280_CalculationArray, 0x1A);
}

void BMP280_Set_Control_Mode(uint8_t OverSamplingSet_Temperature, uint8_t OverSamplingSet_Pressure, uint8_t PowerMode)
{
  I2C.WriteRegister(0x76, 0xF4, ((OverSamplingSet_Temperature & 0x7) << 5) | ((OverSamplingSet_Pressure & 0x7) << 2) | (PowerMode & 0x3));
}

void BMP280_Set_Operation_Mode(uint8_t StandByTime, uint8_t FilterCoefficient, uint8_t SPIModeEnabled)
{
  I2C.WriteRegister(0x76, 0xF5, ((StandByTime & 0x7) << 5) | ((FilterCoefficient & 0x7) << 2) | (SPIModeEnabled & 1));
}

void CalculatePressure(void)
{
  uint8_t BMP280_RawData[6];
  int32_t BMP280_Temperature_Raw, BMP280_Pressure_Raw, BMP280_VariationOne, BMP280_VariationTwo, BMP280_VariationOffSet;
  I2C.RegisterBuffer(0x76, 0xF7, BMP280_RawData, 6);
  BMP280_Pressure_Raw = ((int32_t)(BMP280_RawData[0]) << 16) | ((int32_t)(BMP280_RawData[1]) << 8) | ((int32_t)(BMP280_RawData[2]));
  BMP280_Temperature_Raw = ((int32_t)(BMP280_RawData[3]) << 16) | ((int32_t)(BMP280_RawData[4]) << 8) | ((int32_t)(BMP280_RawData[5]));
  BMP280_Temperature_Raw >>= 4;
  BMP280_Pressure_Raw >>= 4;
  BMP280_VariationOne = ((((BMP280_Temperature_Raw >> 3) - ((int32_t)BMP280_Calculation.dig_T1 << 1))) * ((int32_t)BMP280_Calculation.dig_T2)) >> 11;
  BMP280_VariationTwo = (((((BMP280_Temperature_Raw >> 4) - ((int32_t)BMP280_Calculation.dig_T1)) * ((BMP280_Temperature_Raw >> 4) -
                                                                                                     ((int32_t)BMP280_Calculation.dig_T1))) >>
                          12) *
                         ((int32_t)BMP280_Calculation.dig_T3)) >>
                        14;
  BMP280_VariationOffSet = BMP280_VariationOne + BMP280_VariationTwo;
  BaroTemperatureRaw = (BMP280_VariationOffSet * 5 + 128) >> 8;
  BMP280_VariationOne = ((int64_t)BMP280_VariationOffSet) - 128000;
  BMP280_VariationTwo = BMP280_VariationOne * BMP280_VariationOne * (int64_t)BMP280_Calculation.dig_P6;
  BMP280_VariationTwo = BMP280_VariationTwo + ((BMP280_VariationOne * (int64_t)BMP280_Calculation.dig_P5) << 17);
  BMP280_VariationTwo = BMP280_VariationTwo + (((int64_t)BMP280_Calculation.dig_P4) << 35);
  BMP280_VariationOne = ((BMP280_VariationOne * BMP280_VariationOne * (int64_t)BMP280_Calculation.dig_P3) >> 8) + ((BMP280_VariationOne * (int64_t)BMP280_Calculation.dig_P2) << 12);
  BMP280_VariationOne = (((((int64_t)1) << 47) + BMP280_VariationOne)) * ((int64_t)BMP280_Calculation.dig_P1) >> 33;
  if (BMP280_VariationOne == 0)
    return;
  BMP280_Pressure = 1048576 - BMP280_Pressure_Raw;
  BMP280_Pressure = (((BMP280_Pressure << 31) - BMP280_VariationTwo) * 3125) / BMP280_VariationOne;
  BMP280_VariationOne = (((int64_t)BMP280_Calculation.dig_P9) * (BMP280_Pressure >> 13) * (BMP280_Pressure >> 13)) >> 25;
  BMP280_VariationTwo = (((int64_t)BMP280_Calculation.dig_P8) * BMP280_Pressure) >> 19;
  BMP280_Pressure = ((BMP280_Pressure + BMP280_VariationOne + BMP280_VariationTwo) >> 8) + (((int64_t)BMP280_Calculation.dig_P7) << 4);
  BaroPressureRaw = BMP280_Pressure >> 8;
}

void BMP280_Initialization()
{
  BMP280_Get_Calibration();
  //VALORES DEFINIDOS A PARTIR DO DATASHEET
  //OverSamplingSet_Temperature:010 (x2 / 17 bit / 0.0025 °C)
  //OverSamplingSet_Pressure:5 (20 bit / 0.16 Pa / ULTRA HIGH RESOLUTION)
  //PowerMode:11 (NORMAL MODE)
  //StandByTime:000 (0.5MS)
  //FilterCoefficient:4
  //SPIModeEnabled:0x00 (MODO SPI DESATIVADO)
  BMP280_Set_Operation_Mode(0, 4, 0);
  BMP280_Set_Control_Mode(2, 5, 3);
}

void BMP280_Update()
{
  Baro_AverageFilter();
  CalculatePressure();
}
