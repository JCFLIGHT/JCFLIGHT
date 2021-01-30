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
#include "Scheduler/SCHEDULERTIME.h"

static struct
{
  uint16_t CountVector[7];
  uint32_t UT;
  uint32_t UP;
  uint8_t CountState;
} StructBarometer;

void MS5611_Initialization()
{
  I2C.WriteRegister(0x77, 0x1E, 0);
  SCHEDULER.Sleep(100);
  union
  {
    uint16_t Value;
    uint8_t UT_UP_Raw[2];
  } BaroData;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xA2);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[1] = BaroData.Value;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xA4);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[2] = BaroData.Value;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xA6);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[3] = BaroData.Value;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xA8);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[4] = BaroData.Value;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xAA);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[5] = BaroData.Value;
  I2C.Restart(0x77 << 1);
  I2C.Write(0xAC);
  I2C.Restart((0x77 << 1) | 1);
  BaroData.UT_UP_Raw[1] = I2C.ReadACK();
  BaroData.UT_UP_Raw[0] = I2C.ReadNAK();
  StructBarometer.CountVector[6] = BaroData.Value;
}

static void UT_UP_Start(uint8_t Register)
{
  I2C.Restart(0x77 << 1);
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
  I2C.Restart(0x77 << 1);
  I2C.Write(0);
  I2C.Restart((0x77 << 1) | 1);
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
  BaroTemperatureRaw = DeltaTimeSigned + 2000;
  if (DeltaTimeSigned < 0)
  {
    DeltaTimeSigned *= 5 * DeltaTimeSigned;
    OffSet -= DeltaTimeSigned >> 1;
    Sense -= DeltaTimeSigned >> 2;
  }
  BaroPressureRaw = (((StructBarometer.UP * Sense) / 2097152) - OffSet) / 32768;
}

void MS5611_Update()
{
  uint8_t Command_UT_UP;
  uint32_t *RawValue_UT_UP;
  if (StructBarometer.CountState == 2)
  {
    CalculatePressure();
    StructBarometer.CountState = 0;
    return;
  }
  if (StructBarometer.CountState == 0)
  {
    Baro_AverageFilter();
    RawValue_UT_UP = &StructBarometer.UT;
    Command_UT_UP = 0x48;
  }
  else
  {
    RawValue_UT_UP = &StructBarometer.UP;
    Command_UT_UP = 0x58;
  }
  StructBarometer.CountState++;
  UT_UP_Read(RawValue_UT_UP);
  UT_UP_Start(Command_UT_UP);
}
