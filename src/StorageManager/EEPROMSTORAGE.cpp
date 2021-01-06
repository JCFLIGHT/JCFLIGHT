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

#include "EEPROMSTORAGE.h"

EEPROMSTORAGE STORAGEMANAGER;

#ifdef __AVR_ATmega2560__

#include <avr/eeprom.h>

void EEPROMSTORAGE::Write_8Bits(int16_t Address, uint8_t Value)
{
  eeprom_write_byte((uint8_t *)Address, Value);
}

void EEPROMSTORAGE::Write_16Bits(int16_t Address, int16_t Value)
{
  eeprom_write_word((uint16_t *)Address, Value);
}

void EEPROMSTORAGE::Write_32Bits(int16_t Address, int32_t Value)
{
  eeprom_write_dword((uint32_t *)Address, Value);
}

void EEPROMSTORAGE::Write_Float(int16_t Address, float Value)
{
  _Type_Union.FloatValue = Value;
  Write_32Bits(Address, _Type_Union.LongValue);
}

uint8_t EEPROMSTORAGE::Read_8Bits(int16_t Address)
{
  return eeprom_read_byte((const uint8_t *)Address);
}

int16_t EEPROMSTORAGE::Read_16Bits(int16_t Address)
{
  return eeprom_read_word((const uint16_t *)Address);
}

int32_t EEPROMSTORAGE::Read_32Bits(int16_t Address)
{
  return eeprom_read_dword((const uint32_t *)Address);
}

float EEPROMSTORAGE::Read_Float(int16_t Address)
{
  _Type_Union.LongValue = eeprom_read_dword((const uint32_t *)Address);
  return _Type_Union.FloatValue;
}

#elif defined ESP32

#include "HAL_ESP32/FLASHSTORAGE.h"

void EEPROMSTORAGE::Write_8Bits(int16_t Address, uint8_t Value)
{
  EEPROM_Write_8Bits(Address, Value);
}

void EEPROMSTORAGE::Write_16Bits(int16_t Address, int16_t Value)
{
  EEPROM_Write_16Bits(Address, Value);
}

void EEPROMSTORAGE::Write_32Bits(int16_t Address, int32_t Value)
{
  EEPROM_Write_32Bits(Address, Value);
}

void EEPROMSTORAGE::Write_Float(int16_t Address, float Value)
{
  EEPROM_Write_Float(Address, Value);
}

uint8_t EEPROMSTORAGE::Read_8Bits(int16_t Address)
{
  return EEPROM_Read_8Bits(Address);
}

int16_t EEPROMSTORAGE::Read_16Bits(int16_t Address)
{
  return EEPROM_Read_16Bits(Address);
}

int32_t EEPROMSTORAGE::Read_32Bits(int16_t Address)
{
  return EEPROM_Read_32Bits(Address);
}

float EEPROMSTORAGE::Read_Float(int16_t Address)
{
  return EEPROM_Read_Float(Address);
}

#elif defined __arm__

void EEPROMSTORAGE::Write_8Bits(int16_t Address, uint8_t Value)
{
}

void EEPROMSTORAGE::Write_16Bits(int16_t Address, int16_t Value)
{
}

void EEPROMSTORAGE::Write_32Bits(int16_t Address, int32_t Value)
{
}

void EEPROMSTORAGE::Write_Float(int16_t Address, float Value)
{
}

uint8_t EEPROMSTORAGE::Read_8Bits(int16_t Address)
{
  return 0;
}

int16_t EEPROMSTORAGE::Read_16Bits(int16_t Address)
{
  return 0;
}

int32_t EEPROMSTORAGE::Read_32Bits(int16_t Address)
{
  return 0;
}

float EEPROMSTORAGE::Read_Float(int16_t Address)
{
  return 0;
}

#endif