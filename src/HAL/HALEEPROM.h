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

#ifndef HALEEPROM_H_
#define HALEEPROM_H_
#include "Arduino.h"
class HALEEPROMClass
{
public:
    void Write_8Bits(int16_t Address, uint8_t Value);
    void Write_16Bits(int16_t Address, int16_t Value);
    void Write_32Bits(int16_t Address, int32_t Value);
    void Write_Float(int16_t Address, float Value);
    uint8_t Read_8Bits(int16_t Address);
    int16_t Read_16Bits(int16_t Address);
    int32_t Read_32Bits(int16_t Address);
    float Read_Float(int16_t Address);
};
extern HALEEPROMClass HAL_EEPROM;
#endif