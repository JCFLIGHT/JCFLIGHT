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

#include "HALEEPROM.h"
#include "HAL_AVR/AVREEPROM.h"
#include "HAL_ESP32/ESP32EEPROM.h"
#include "HAL_STM32/STM32EEPROM.h"

HALEEPROMClass HAL_EEPROM;

void HALEEPROMClass::Write_8Bits(int16_t Address, uint8_t Value)
{
    EEPROM_Write_8Bits(Address, Value);
}

void HALEEPROMClass::Write_16Bits(int16_t Address, int16_t Value)
{
    EEPROM_Write_16Bits(Address, Value);
}

void HALEEPROMClass::Write_32Bits(int16_t Address, int32_t Value)
{
    EEPROM_Write_32Bits(Address, Value);
}

void HALEEPROMClass::Write_Float(int16_t Address, float Value)
{
    EEPROM_Write_Float(Address, Value);
}

uint8_t HALEEPROMClass::Read_8Bits(int16_t Address)
{
    return EEPROM_Read_8Bits(Address);
}

int16_t HALEEPROMClass::Read_16Bits(int16_t Address)
{
    return EEPROM_Read_16Bits(Address);
}

int32_t HALEEPROMClass::Read_32Bits(int16_t Address)
{
    return EEPROM_Read_32Bits(Address);
}

float HALEEPROMClass::Read_Float(int16_t Address)
{
    return EEPROM_Read_Float(Address);
}