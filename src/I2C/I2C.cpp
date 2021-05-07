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
#include "Barometer/BAROBACKEND.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Compass/COMPASSREAD.h"
#include "IMU/ACCGYROREAD.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "FastSerial/PRINTF.h"

I2CPROTOCOL I2C;
I2C_Resources_Struct I2CResources;

//DEBUG
#define PRINTLN_I2C

#ifdef __AVR_ATmega2560__

void I2CPROTOCOL::Initialization(void)
{
  TWSR = 0;
  TWBR = 12;
  TWCR = 4;
  I2C.SearchDevicesInBarrament();
}

void __attribute__((noinline)) WaitTransmission(uint8_t _TWCR)
{
  TWCR = _TWCR;
  uint8_t CheckTWCRTWINTState = SIZE_OF_I2C_DEVICES;
  while (!(TWCR & (1 << TWINT)))
  {
    CheckTWCRTWINTState--;
    if (CheckTWCRTWINTState == 0)
    {
      TWCR = 0;
      I2CResources.Error.Count++;
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

uint8_t I2CPROTOCOL::ReadACK(void)
{
  WaitTransmission((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
  return TWDR;
}

uint8_t I2CPROTOCOL::ReadNAK(void)
{
  WaitTransmission((1 << TWINT) | (1 << TWEN));
  uint8_t _TWDR = TWDR;
  I2C.Stop();
  return _TWDR;
}

void I2CPROTOCOL::RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size)
{
  I2C.Restart(Address << 1);
  I2C.Write(Register);
  I2C.Restart((Address << 1) | 1);
  uint8_t *BufferPointer = Buffer;
  while (Size--)
  {
    *BufferPointer++ = I2C.ReadACK();
  }
  WaitTransmission((1 << TWINT) | (1 << TWEN));
  uint8_t _TWDR = TWDR;
  I2C.Stop();
  *BufferPointer = _TWDR;
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Data)
{
  I2C.Restart(Address << 1);
  I2C.Write(Register);
  I2C.Write(Data);
  I2C.Stop();
}

void I2CPROTOCOL::SearchDevicesInBarrament(void)
{
  static uint8_t Status;
  uint8_t Devices = 0;
#ifdef PRINTLN_I2C
  LINE_SPACE;
  LOG("ESCANEANDO O BARRAMENTO I2C,AGUARDE...");
  LINE_SPACE;
#endif
  for (uint8_t NumbGenerator = 0; NumbGenerator <= 0x7F; NumbGenerator++)
  {
    Status = 0;
    Status = I2C.StartToSearchDevices();
    if (!Status)
    {
      Status = I2C.SendHexadecimalValues(NumbGenerator << 1);
    }
    if (Status)
    {
      if (Status == 1)
      {
#ifdef PRINTLN_I2C
        LOG("OCORREU ALGUM ERRO,NÃO FOI POSSIVEL COMPLETAR");
#endif
        return;
      }
    }
    else
    {
#ifdef PRINTLN_I2C

      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        LOG("DISPOSITIVO ENCONTRADO - 0x77 << MS5611");
      }

      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        LOG("DISPOSITIVO ENCONTRADO - 0x76 << BMP280");
      }

      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        LOG("DISPOSITIVO ENCONTRADO - 0x0C < AK8975");
      }

      if (NumbGenerator == ADDRESS_COMPASS_HMC5883)
      {
        LOG("DISPOSITIVO ENCONTRADO - 0x1E << HMC5883");
      }

      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        LOG("DISPOSITIVO ENCONTRADO - 0x0D << QMC5883");
      }
#endif

      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        IMU.Compass.Type = COMPASS_AK8975;
      }

      if (NumbGenerator == ADDRESS_COMPASS_HMC5883)
      {
        IMU.Compass.Type = COMPASS_HMC5883;
      }

      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        IMU.Compass.Type = COMPASS_QMC5883;
      }

      if ((NumbGenerator == ADDRESS_COMPASS_AK8975) || (NumbGenerator == ADDRESS_COMPASS_HMC5883) || (NumbGenerator == ADDRESS_COMPASS_QMC5883))
      {
        I2CResources.Found.Compass = true;
      }

      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        BAROMETER.SetType(ADDRESS_BAROMETER_MS5611);
      }

      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        BAROMETER.SetType(ADDRESS_BAROMETER_BMP280);
      }

      if ((NumbGenerator == ADDRESS_BAROMETER_MS5611) || (NumbGenerator == ADDRESS_BAROMETER_BMP280))
      {
        I2CResources.Found.Barometer = true;
      }

      Devices++;

#ifdef PRINTLN_I2C
      SCHEDULERTIME.Sleep(20);
#endif
    }
    I2C.Stop();
  }
#ifdef PRINTLN_I2C
  if (!Devices)
  {
    LOG("NENHUM DISPOSITVO ENCONTRADO");
  }
  LINE_SPACE;
#endif
}

uint8_t I2CPROTOCOL::StartToSearchDevices(void)
{
  uint32_t I2COldTime = SCHEDULERTIME.GetMillis();
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if ((SCHEDULERTIME.GetMillis() - I2COldTime) >= 80)
    {
      TWCR = 0;
      TWCR = _BV(TWEN) | _BV(TWEA);
      return 1;
    }
  }
  if (((TWSR & 0xF8) == 0x08) || ((TWSR & 0xF8) == 0x10))
  {
    return 0;
  }
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
  uint32_t I2COldTime = SCHEDULERTIME.GetMillis();
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if ((SCHEDULERTIME.GetMillis() - I2COldTime) >= 80)
    {
      TWCR = 0;
      TWCR = _BV(TWEN) | _BV(TWEA);
      return 1;
    }
  }
  if (((TWSR & 0xF8) == 0x18) || ((TWSR & 0xF8) == 0x40))
  {
    return 0;
  }
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

#elif defined ESP32

#include <Wire.h>

void I2CPROTOCOL::Initialization(void)
{
  Wire.begin(21, 22, 400000);
  I2C.SearchDevicesInBarrament();
}

void I2CPROTOCOL::Restart(uint8_t Address)
{
}

void I2CPROTOCOL::Stop(void)
{
}

void I2CPROTOCOL::Write(uint8_t SendData)
{
}

uint8_t I2CPROTOCOL::ReadACK(void)
{
  return 0;
}

uint8_t I2CPROTOCOL::ReadNAK(void)
{
  return 0;
}

void I2CPROTOCOL::RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission(false);
  Wire.requestFrom(Address, Size);
  uint8_t index = 0;
  while (Wire.available())
  {
    Buffer[index++] = Wire.read();
  }
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void I2CPROTOCOL::SearchDevicesInBarrament(void)
{
  Serial.println("ESCANEANDO O BARRAMENTO I2C,AGUARDE...");
  Serial.println("\n");
  Wire.begin();
  for (uint8_t NumbGenerator = 0; NumbGenerator <= 0x7F; NumbGenerator++)
  {
    Wire.beginTransmission(NumbGenerator);
    if (Wire.endTransmission() == 0)
    {
      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        IMU.Compass.Type = COMPASS_AK8975;
        Serial.println("AK8975 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_COMPASS_HMC5883)
      {
        IMU.Compass.Type = COMPASS_HMC5883;
        Serial.println("HMC5883 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        IMU.Compass.Type = COMPASS_QMC5883;
        Serial.println("QMC5883 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        BAROMETER.SetType(ADDRESS_BAROMETER_MS5611);
        Serial.println("MS5611 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        BAROMETER.SetType(ADDRESS_BAROMETER_BMP280);
        Serial.println("BMP280 ENCONTRADO!");
      }

      if ((NumbGenerator == ADDRESS_COMPASS_AK8975) || (NumbGenerator == ADDRESS_COMPASS_HMC5883) || (NumbGenerator == ADDRESS_COMPASS_QMC5883))
      {
        I2CResources.Found.Compass = true;
      }

      if ((NumbGenerator == ADDRESS_BAROMETER_MS5611) || (NumbGenerator == ADDRESS_BAROMETER_BMP280))
      {
        I2CResources.Found.Barometer = true;
      }
    }
  }
}

#elif defined __arm__

void I2CPROTOCOL::Initialization(void)
{
}

void I2CPROTOCOL::Restart(uint8_t Address)
{
}

void I2CPROTOCOL::Stop(void)
{
}

void I2CPROTOCOL::Write(uint8_t SendData)
{
}

uint8_t I2CPROTOCOL::ReadACK(void)
{
  return 0;
}

uint8_t I2CPROTOCOL::ReadNAK(void)
{
  return 0;
}

void I2CPROTOCOL::RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size)
{
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Data)
{
}

#endif

void I2CPROTOCOL::All_Initialization(void)
{
  uint8_t ForceInitialization = 5;
  SCHEDULERTIME.Sleep(200);
  I2C.Initialization();
  MPU6050AccAndGyroInitialization();
  while (ForceInitialization--)
  {
    BAROMETER.Initialization();
    COMPASS.Initialization();
  }
}