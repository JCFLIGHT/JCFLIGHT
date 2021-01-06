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

#ifndef AVREEPROM_H_
#define AVREEPROM_H_
#ifdef __AVR_ATmega2560__
#include "Arduino.h"
void EEPROM_Write_8Bits(int16_t Address, uint8_t Value);
void EEPROM_Write_16Bits(int16_t Address, int16_t Value);
void EEPROM_Write_32Bits(int16_t Address, int32_t Value);
void EEPROM_Write_Float(int16_t Address, float Value);
uint8_t EEPROM_Read_8Bits(int16_t Address);
int16_t EEPROM_Read_16Bits(int16_t Address);
int32_t EEPROM_Read_32Bits(int16_t Address);
float EEPROM_Read_Float(int16_t Address);
#endif
#endif