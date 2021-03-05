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
#include "HAL/HALEEPROM.h"

EEPROMSTORAGE STORAGEMANAGER;

void EEPROMSTORAGE::Write_8Bits(int16_t Address, uint8_t Value)
{
  if (HAL_EEPROM.Read_8Bits(Address) == Value)
  {
    return;
  }
  HAL_EEPROM.Write_8Bits(Address, Value);
}

void EEPROMSTORAGE::Write_16Bits(int16_t Address, int16_t Value)
{
  if (HAL_EEPROM.Read_16Bits(Address) == Value)
  {
    return;
  }
  HAL_EEPROM.Write_16Bits(Address, Value);
}

void EEPROMSTORAGE::Write_32Bits(int16_t Address, int32_t Value)
{
  if (HAL_EEPROM.Read_32Bits(Address) == Value)
  {
    return;
  }
  HAL_EEPROM.Write_32Bits(Address, Value);
}

void EEPROMSTORAGE::Write_Float(int16_t Address, float Value)
{
  if (HAL_EEPROM.Read_Float(Address) == Value)
  {
    return;
  }
  HAL_EEPROM.Write_Float(Address, Value);
}

uint8_t EEPROMSTORAGE::Read_8Bits(int16_t Address)
{
  return HAL_EEPROM.Read_8Bits(Address);
}

int16_t EEPROMSTORAGE::Read_16Bits(int16_t Address)
{
  return HAL_EEPROM.Read_16Bits(Address);
}

int32_t EEPROMSTORAGE::Read_32Bits(int16_t Address)
{
  return HAL_EEPROM.Read_32Bits(Address);
}

float EEPROMSTORAGE::Read_Float(int16_t Address)
{
  return HAL_EEPROM.Read_Float(Address);
}