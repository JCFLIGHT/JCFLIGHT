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

#include "MS5611.h"
#include "BAROREAD.h"
#include "I2C/I2C.h"
#include "Common/ENUM.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Barometer/BAROBACKEND.h"

static struct
{
  uint16_t CountVector[7];
  uint32_t UT;
  uint32_t UP;
  uint8_t CountState;
} StructBarometer;

void MS5611_Initialization(void)
{
  I2C.WriteRegister(ADDRESS_BAROMETER_MS5611, 0x1E, 0);
  SCHEDULERTIME.Sleep(100);
  union
  {
    uint16_t Value;
    uint8_t UT_UP_Raw[2];
  } BaroData;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xA2);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[1] = BaroData.Value;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xA4);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[2] = BaroData.Value;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xA6);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[3] = BaroData.Value;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xA8);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[4] = BaroData.Value;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xAA);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[5] = BaroData.Value;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0xAC);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[6] = BaroData.Value;
}

static void UT_UP_Start(uint8_t Register)
{
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(Register);
  I2C.Stop();
}

static void UT_UP_Read(uint32_t *Value)
{
  union
  {
    uint32_t Value;
    uint8_t UT_UP_Raw[4];
  } BaroData;
  I2C.Restart(ADDRESS_BAROMETER_MS5611 << 1);
  I2C.Write(0);
  I2C.Restart((ADDRESS_BAROMETER_MS5611 << 1) | 1);
  BaroData.UT_UP_Raw[2] = I2C.ReadACK();
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  *Value = BaroData.Value;
}

static void CalculatePressure()
{
  int32_t DeltaTimeSigned;
  float DeltaTimeFloat = (int32_t)StructBarometer.UT - (int32_t)((uint32_t)StructBarometer.CountVector[5] << 8);
  float OffSet = ((uint32_t)StructBarometer.CountVector[2] << 16) + ((DeltaTimeFloat * StructBarometer.CountVector[4]) / 128);
  float Sense = ((uint32_t)StructBarometer.CountVector[1] << 15) + ((DeltaTimeFloat * StructBarometer.CountVector[3]) / 256);
  DeltaTimeSigned = (DeltaTimeFloat * StructBarometer.CountVector[6]) / 8388608;
  Barometer.Raw.Temperature = DeltaTimeSigned + 2000;
  if (DeltaTimeSigned < 0)
  {
    DeltaTimeSigned *= 5 * DeltaTimeSigned;
    OffSet -= DeltaTimeSigned >> 1;
    Sense -= DeltaTimeSigned >> 2;
  }
  Barometer.Raw.Pressure = (((StructBarometer.UP * Sense) / 2097152) - OffSet) / 32768;
}

void MS5611_Update(void)
{
  uint8_t Command_UT_UP;
  uint32_t *RawValue_UT_UP;

  switch (StructBarometer.CountState)
  {
  case 0:
    Remove_Barometer_Spikes();
    RawValue_UT_UP = &StructBarometer.UT;
    Command_UT_UP = 0x48;
    break;

  case 2:

    CalculatePressure();
    StructBarometer.CountState = 0;
    return;

  default:
    RawValue_UT_UP = &StructBarometer.UP;
    Command_UT_UP = 0x58;
    break;
  }

  StructBarometer.CountState++;
  UT_UP_Read(RawValue_UT_UP);
  UT_UP_Start(Command_UT_UP);
}

/*

static uint32_t MS5611_UT;
static uint32_t MS5611_UP;
static uint16_t MS5611_ROM[8];
static uint8_t MS5611_OSR = 0x08;

static uint32_t MS5611_Read_Buffer(void)
{
  uint8_t MS5611Buffer[3];
  I2C.RegisterBuffer(ADDRESS_BAROMETER_MS5611, 0x00, MS5611Buffer, 0x03);
  return (MS5611Buffer[0] << 16) | (MS5611Buffer[1] << 8) | MS5611Buffer[2];
}

typedef enum
{
  BAROMETER_NEEDS_SAMPLES = 0,
  BAROMETER_NEEDS_CALCULATION
} BarometerState_Enum;

static void MS5611Calculate(void)
{
  int64_t Temperature;
  int64_t TemperatureDelta;
  int64_t DeltaTime = (int64_t)MS5611_UT - ((uint64_t)MS5611_ROM[5] * 256);
  int64_t OffSet = ((int64_t)MS5611_ROM[2] << 16) + (((int64_t)MS5611_ROM[4] * DeltaTime) >> 7);
  int64_t Scale = ((int64_t)MS5611_ROM[1] << 15) + (((int64_t)MS5611_ROM[3] * DeltaTime) >> 8);
  Temperature = 2000 + ((DeltaTime * (int64_t)MS5611_ROM[6]) >> 23);

  if (Temperature < 2000)
  {
    TemperatureDelta = Temperature - 2000;
    TemperatureDelta = 5 * TemperatureDelta * TemperatureDelta;
    OffSet -= TemperatureDelta >> 1;
    Scale -= TemperatureDelta >> 2;
    if (Temperature < -1500)
    {
      TemperatureDelta = Temperature + 1500;
      TemperatureDelta = TemperatureDelta * TemperatureDelta;
      OffSet -= 7 * TemperatureDelta;
      Scale -= (11 * TemperatureDelta) >> 1;
    }
    Temperature -= ((DeltaTime * DeltaTime) >> 31);
  }

  Barometer.Raw.Pressure = ((((int64_t)MS5611_UP * Scale) >> 21) - OffSet) >> 15;
  Barometer.Raw.Temperature = Temperature;
}

uint32_t BaroUpdate(void)
{
  static BarometerState_Enum State = BAROMETER_NEEDS_SAMPLES;

  switch (State)
  {

  default:
  case BAROMETER_NEEDS_SAMPLES:
    MS5611_UT = MS5611_Read_Buffer();
    I2C.WriteRegister(ADDRESS_BAROMETER_MS5611, 0x40 + 0x00 + MS5611_OSR, 0x01);
    State = BAROMETER_NEEDS_CALCULATION;
    return 10000;
    break;

  case BAROMETER_NEEDS_CALCULATION:
    MS5611_UP = MS5611_Read_Buffer();
    I2C.WriteRegister(ADDRESS_BAROMETER_MS5611, 0x40 + 0x10 + MS5611_OSR, 0x01);
    MS5611Calculate();
    //Barometer.Raw.Pressure = applyBarometerMedianFilter(Barometer.Raw.Pressure);
    State = BAROMETER_NEEDS_SAMPLES;
    return 10000;
    break;
  }
}

void MS5611_Initialization()
{
  for (uint8_t IndexCount = 0; IndexCount < 8; IndexCount++)
  {
    uint8_t MS5611Buffer[2] = {0, 0};
    I2C.RegisterBuffer(ADDRESS_BAROMETER_MS5611, 0xA0 + IndexCount * 2, MS5611Buffer, 0x02);
    MS5611_ROM[IndexCount] = (MS5611Buffer[0] << 8 | MS5611Buffer[1]);
  }
}

void MS5611_Update()
{
  BaroUpdate();
}
*/