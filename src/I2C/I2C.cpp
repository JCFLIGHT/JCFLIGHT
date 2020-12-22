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

#include "I2C.h"
#include "Common/STRUCTS.h"
#include "Common/VARIABLES.h"
#include "Barometer/BAROBACKEND.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Compass/COMPASSREAD.h"
#include "IMU/ACCGYROREAD.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FastSerial/PRINTF.h"
#include "BAR/BAR.h"

I2CPROTOCOL I2C;

//#define DEBUG_I2C

uint8_t BufferData[6];

void I2CPROTOCOL::Initialization(void)
{
  TWSR = 0;
  TWBR = 12;
  TWCR = 4;
  //CARREGA O TIPO DE COMPASS USADO PELO USUARIO SALVO NA EEPROM
  if (STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 0 || STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 3)
    Compass_Type = COMPASS_AK8975;
  else if (STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 1 || STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 4)
    Compass_Type = COMPASS_HMC5843;
  else if (STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 2 || STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) == 5)
    Compass_Type = COMPASS_HMC5883;
  I2C.SearchDevicesInBarrament();
  //AK8975 ENDEREÇO:0x0C
  //HMC5843 OU HMC5883 ENDEREÇO:0x1E OU 0x0D
  if (Compass_Type == COMPASS_AK8975)
  {
    MagAddress = 0x0C;
    MagRegister = 0x03;
  }
  else if ((Compass_Type == COMPASS_HMC5843) || (Compass_Type == COMPASS_HMC5883))
  {
    if (COMPASS.FakeHMC5883Address == 0x0D)
    {
      MagAddress = 0x0D;
      MagRegister = 0x00;
    }
    else
    {
      MagAddress = 0x1E;
      MagRegister = 0x03;
    }
  }
}

void __attribute__((noinline)) WaitTransmission(uint8_t _TWCR)
{
  TWCR = _TWCR;
  uint8_t CheckTWCRTWINTState = SizeOfThis;
  while (!(TWCR & (1 << TWINT)))
  {
    CheckTWCRTWINTState--;
    if (CheckTWCRTWINTState == 0)
    {
      TWCR = 0;
      ErrorI2C++;
      break;
    }
  }
}

void I2CPROTOCOL::Restart(uint8_t Address)
{
  WaitTransmission((1 << TWINT) | (1 << TWSTA) | (1 << TWEN));
  TWDR = Address;
  WaitTransmission((1 << TWINT) | (1 << TWEN));
}

void I2CPROTOCOL::Stop(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void I2CPROTOCOL::Write(uint8_t SendData)
{
  TWDR = SendData;
  WaitTransmission((1 << TWINT) | (1 << TWEN));
}

uint8_t I2CPROTOCOL::ReadACK()
{
  WaitTransmission((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
  return TWDR;
}

uint8_t I2CPROTOCOL::ReadNAK()
{
  WaitTransmission((1 << TWINT) | (1 << TWEN));
  uint8_t _TWDR = TWDR;
  Stop();
  return _TWDR;
}

void I2CPROTOCOL::RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size)
{
  Restart(Address << 1);
  Write(Register);
  Restart((Address << 1) | 1);
  uint8_t *BufferPointer = Buffer;
  while (Size--)
    *BufferPointer++ = ReadACK();
  WaitTransmission((1 << TWINT) | (1 << TWEN));
  uint8_t _TWDR = TWDR;
  Stop();
  *BufferPointer = _TWDR;
}

void I2CPROTOCOL::SensorsRead(uint8_t Address, uint8_t Register)
{
  RegisterBuffer(Address, Register, BufferData, 6);
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Value)
{
  Restart(Address << 1);
  Write(Register);
  Write(Value);
  Stop();
}

void I2CPROTOCOL::SearchDevicesInBarrament()
{
  static uint8_t Status;
  uint8_t Devices = 0;
#ifdef DEBUG_I2C
  FastSerialPrintln(PSTR("ESCANEANDO O BARRAMENTO I2C,AGUARDE...\n"));
  FastSerialPrintln(PSTR("\n"));
#endif
  for (uint8_t NumbGenerator = 0; NumbGenerator <= 0x7F; NumbGenerator++)
  {
    Status = 0;
    Status = I2C.StartToSearchDevices();
    if (!Status)
      Status = I2C.SendHexadecimalValues(NumbGenerator << 1);
    if (Status)
    {
      if (Status == 1)
      {
#ifdef DEBUG_I2C
        FastSerialPrintln(PSTR("OCORREU ALGUM ERRO,NÃO FOI POSSIVEL COMPLETAR\n"));
#endif
        return;
      }
    }
    else
    {
#ifdef DEBUG_I2C
      FastSerialPrintln(PSTR("DISPOSITIVO ENCONTRADO - "));
      if (NumbGenerator == 0x68)
        FastSerialPrintln(PSTR("0x68"));
      if (NumbGenerator == 0x68)
        FastSerialPrintln(PSTR(" << MPU6050"));
      if (NumbGenerator == 0x77)
      {
        FastSerialPrintln(PSTR("0x77"));
        FastSerialPrintln(PSTR(" << MS5611"));
      }
      if (NumbGenerator == 0x76)
      {
        FastSerialPrintln(PSTR("0x76"));
        FastSerialPrintln(PSTR(" << BMP280"));
      }
      if (NumbGenerator == 0x0C)
        FastSerialPrintln(PSTR("0x0C"));
      if (NumbGenerator == 0x0C)
        FastSerialPrintln(PSTR(" << AK8975"));
      if (NumbGenerator == 0x1E)
        FastSerialPrintln(PSTR("0x1E"));
      if (NumbGenerator == 0x0D)
        FastSerialPrintln(PSTR("0x0D"));
      if ((NumbGenerator == 0x1E) || (NumbGenerator == 0x0D))
        FastSerialPrintln(PSTR(" << HMC5843 OU HMC5883"));
      FastSerialPrintln(PSTR("\n"));
#endif
      if (NumbGenerator == 0x0D)
        COMPASS.FakeHMC5883Address = 0x0D;

      if (NumbGenerator == 0x77)
        SetBaroType(0x77);

      if (NumbGenerator == 0x76)
        SetBaroType(0x76);

      if ((NumbGenerator == 0x0C) || (NumbGenerator == 0x1E) || (NumbGenerator == 0x0D))
      {
        CompassFound = true;
      }
      if ((NumbGenerator == 0x77) || (NumbGenerator == 0x76))
        BarometerFound = true;
      Devices++;
#ifdef DEBUG_I2C
      AVRTIME.SchedulerSleep(20);
#endif
    }
    I2C.Stop();
  }
#ifdef DEBUG_I2C
  if (!Devices)
    FastSerialPrintln(PSTR("NENHUM DISPOSITVO ENCONTRADO\n"));
  if (CompassFound)
    FastSerialPrintln(PSTR("COMPASS CONECTADO AO I2C?SIM\n"));
  else
    FastSerialPrintln(PSTR("COMPASS CONECTADO AO I2C?NÃO\n"));
  if (BarometerFound)
    FastSerialPrintln(PSTR("BAROMETRO CONECTADO AO I2C?SIM\n"));
  else
    FastSerialPrintln(PSTR("BAROMETRO CONECTADO AO I2C?NÃO\n"));
#endif
}

uint8_t I2CPROTOCOL::StartToSearchDevices()
{
  uint32_t I2COldTime = AVRTIME.SchedulerMillis();
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if ((AVRTIME.SchedulerMillis() - I2COldTime) >= 80)
    {
      TWCR = 0;
      TWCR = _BV(TWEN) | _BV(TWEA);
      return 1;
    }
  }
  if (((TWSR & 0xF8) == 0x08) || ((TWSR & 0xF8) == 0x10))
    return 0;
  if ((TWSR & 0xF8) == 0x38)
  {
    uint8_t BufferedStatus = (TWSR & 0xF8);
    TWCR = 0;
    TWCR = _BV(TWEN) | _BV(TWEA);
    return BufferedStatus;
  }
  return (TWSR & 0xF8);
}

uint8_t I2CPROTOCOL::SendHexadecimalValues(uint8_t Address)
{
  TWDR = Address;
  uint32_t I2COldTime = AVRTIME.SchedulerMillis();
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if ((AVRTIME.SchedulerMillis() - I2COldTime) >= 80)
    {
      TWCR = 0;
      TWCR = _BV(TWEN) | _BV(TWEA);
      return 1;
    }
  }
  if (((TWSR & 0xF8) == 0x18) || ((TWSR & 0xF8) == 0x40))
    return 0;
  uint8_t BufferedStatus = (TWSR & 0xF8);
  if (((TWSR & 0xF8) == 0x20) || ((TWSR & 0xF8) == 0x48))
  {
    I2C.Stop();
    return BufferedStatus;
  }
  else
  {
    TWCR = 0;
    TWCR = _BV(TWEN) | _BV(TWEA);
    return BufferedStatus;
  }
}

void AllI2CInitialization()
{
  uint8_t ForceInitialization = 5;
  AVRTIME.SchedulerSleep(200);
  while (ForceInitialization)
  {
    ForceInitialization--;
    I2C.Initialization();
    Gyro_Initialization();
    if (I2C.BarometerFound)
      Baro_Initialization();
    if (I2C.CompassFound)
      COMPASS.Initialization();
    Acc_Initialization();
  }
}
