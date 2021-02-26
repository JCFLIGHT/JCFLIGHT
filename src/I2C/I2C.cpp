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
#include "Barometer/BAROBACKEND.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Compass/COMPASSREAD.h"
#include "IMU/ACCGYROREAD.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "FastSerial/PRINTF.h"

I2CPROTOCOL I2C;

//#define DEBUG_I2C

uint8_t BufferData[6];

#ifdef __AVR_ATmega2560__

void I2CPROTOCOL::Initialization(void)
{
  TWSR = 0;
  TWBR = 12;
  TWCR = 4;
  I2C.SearchDevicesInBarrament();
  //AK8975 ENDEREÇO:0x0C
  //HMC5843 OU HMC5883 ENDEREÇO:0x1E
  //QMC5883 ENDEREÇO:0x0D
  if (COMPASS.Type == COMPASS_AK8975)
  {
    COMPASS.Address = ADDRESS_COMPASS_AK8975;
    COMPASS.Register = 0x03;
  }
  else if ((COMPASS.Type == COMPASS_HMC5843) || (COMPASS.Type == COMPASS_HMC5883))
  {
    COMPASS.Address = ADDRESS_COMPASS_HMC5843_OR_HMC5883;
    COMPASS.Register = 0x03;
  }
  else if (COMPASS.Type == COMPASS_QMC5883)
  {
    COMPASS.Address = ADDRESS_COMPASS_QMC5883;
    COMPASS.Register = 0x00;
  }
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
      I2C.Errors++;
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
  {
    *BufferPointer++ = ReadACK();
  }
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
  PRINTF.SendToConsole(PSTR("ESCANEANDO O BARRAMENTO I2C,AGUARDE...\n"));
  PRINTF.SendToConsole(PSTR("\n"));
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
#ifdef DEBUG_I2C
        PRINTF.SendToConsole(PSTR("OCORREU ALGUM ERRO,NÃO FOI POSSIVEL COMPLETAR\n"));
#endif
        return;
      }
    }
    else
    {
#ifdef DEBUG_I2C
      PRINTF.SendToConsole(PSTR("DISPOSITIVO ENCONTRADO - "));
      if (NumbGenerator == ADDRESS_IMU_MPU6050)
      {
        PRINTF.SendToConsole(PSTR("0x68"));
      }
      if (NumbGenerator == ADDRESS_IMU_MPU6050)
      {
        PRINTF.SendToConsole(PSTR(" << MPU6050"));
      }
      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        PRINTF.SendToConsole(PSTR("0x77"));
        PRINTF.SendToConsole(PSTR(" << MS5611"));
      }
      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        PRINTF.SendToConsole(PSTR("0x76"));
        PRINTF.SendToConsole(PSTR(" << BMP280"));
      }
      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        PRINTF.SendToConsole(PSTR("0x0C"));
      }
      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        PRINTF.SendToConsole(PSTR(" << AK8975"));
      }
      if (NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883)
      {
        PRINTF.SendToConsole(PSTR("0x1E"));
      }
      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        PRINTF.SendToConsole(PSTR("0x0D"));
      }
      if ((NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883) || (NumbGenerator == ADDRESS_COMPASS_QMC5883))
      {
        PRINTF.SendToConsole(PSTR(" << HMC5843 OU HMC5883"));
      }
      PRINTF.SendToConsole(PSTR("\n"));
#endif

      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        COMPASS.Type = COMPASS_AK8975;
      }

      if (NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883)
      {
        COMPASS.Type = COMPASS_HMC5843;
      }

      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        COMPASS.Type = COMPASS_QMC5883;
      }

      if ((NumbGenerator == ADDRESS_COMPASS_AK8975) || (NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883) || (NumbGenerator == ADDRESS_COMPASS_QMC5883))
      {
        CompassFound = true;
      }

      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        SetBaroType(ADDRESS_BAROMETER_MS5611);
      }

      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        SetBaroType(ADDRESS_BAROMETER_BMP280);
      }

      if ((NumbGenerator == ADDRESS_BAROMETER_MS5611) || (NumbGenerator == ADDRESS_BAROMETER_BMP280))
      {
        BarometerFound = true;
      }

      Devices++;

#ifdef DEBUG_I2C
      SCHEDULERTIME.Sleep(20);
#endif
    }
    I2C.Stop();
  }
#ifdef DEBUG_I2C
  if (!Devices)
  {
    PRINTF.SendToConsole(PSTR("NENHUM DISPOSITVO ENCONTRADO\n"));
  }
#endif
}

uint8_t I2CPROTOCOL::StartToSearchDevices()
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
  //AK8975 ENDEREÇO:0x0C
  //HMC5843 OU HMC5883 ENDEREÇO:0x1E
  //QMC5883 ENDEREÇO:0x0D
  if (COMPASS.Type == COMPASS_AK8975)
  {
    COMPASS.Address = ADDRESS_COMPASS_AK8975;
    COMPASS.Register = 0x03;
  }
  else if ((COMPASS.Type == COMPASS_HMC5843) || (COMPASS.Type == COMPASS_HMC5883))
  {
    COMPASS.Address = ADDRESS_COMPASS_HMC5843_OR_HMC5883;
    COMPASS.Register = 0x03;
  }
  else if (COMPASS.Type == COMPASS_QMC5883)
  {
    COMPASS.Address = ADDRESS_COMPASS_QMC5883;
    COMPASS.Register = 0x00;
  }
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

uint8_t I2CPROTOCOL::ReadACK()
{
  return 0;
}

uint8_t I2CPROTOCOL::ReadNAK()
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

void I2CPROTOCOL::SensorsRead(uint8_t Address, uint8_t Register)
{
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Value)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Value);
  Wire.endTransmission();
}

void I2CPROTOCOL::SearchDevicesInBarrament()
{
  Serial.println("ESCANEANDO O BARRAMENTO I2C,AGUARDE...");
  Serial.println("\n");
  Wire.begin();
  for (uint8_t NumbGenerator = 0; NumbGenerator <= 0x7F; NumbGenerator++)
  {
    Wire.beginTransmission(NumbGenerator);
    if (Wire.endTransmission() == 0)
    {
      if (NumbGenerator == ADDRESS_IMU_MPU6050)
      {
        Serial.println("MPU6050 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_COMPASS_AK8975)
      {
        COMPASS.Type = COMPASS_AK8975;
        Serial.println("AK8975 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883)
      {
        COMPASS.Type = COMPASS_HMC5843;
        Serial.println("HMC5843 OU HMC5883 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_COMPASS_QMC5883)
      {
        COMPASS.Type = COMPASS_QMC5883;
        Serial.println("QMC5883 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_BAROMETER_MS5611)
      {
        SetBaroType(ADDRESS_BAROMETER_MS5611);
        Serial.println("MS5611 ENCONTRADO!");
      }

      if (NumbGenerator == ADDRESS_BAROMETER_BMP280)
      {
        SetBaroType(ADDRESS_BAROMETER_BMP280);
        Serial.println("BMP280 ENCONTRADO!");
      }

      if ((NumbGenerator == ADDRESS_COMPASS_AK8975) || (NumbGenerator == ADDRESS_COMPASS_HMC5843_OR_HMC5883) || (NumbGenerator == ADDRESS_COMPASS_QMC5883))
      {
        CompassFound = true;
      }

      if ((NumbGenerator == ADDRESS_BAROMETER_MS5611) || (NumbGenerator == ADDRESS_BAROMETER_BMP280))
      {
        BarometerFound = true;
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

uint8_t I2CPROTOCOL::ReadACK()
{
  return 0;
}

uint8_t I2CPROTOCOL::ReadNAK()
{
  return 0;
}

void I2CPROTOCOL::RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size)
{
}

void I2CPROTOCOL::SensorsRead(uint8_t Address, uint8_t Register)
{
}

void I2CPROTOCOL::WriteRegister(uint8_t Address, uint8_t Register, uint8_t Value)
{
}

#endif

void I2CPROTOCOL::All_Initialization()
{
  uint8_t ForceInitialization = 5;
  SCHEDULERTIME.Sleep(200);
  I2C.Initialization();
  while (ForceInitialization--)
  {
    Gyro_Initialization();
    if (I2C.BarometerFound)
    {
      Baro_Initialization();
    }
    if (I2C.CompassFound)
    {
      COMPASS.Initialization();
    }
    Acc_Initialization();
  }
}