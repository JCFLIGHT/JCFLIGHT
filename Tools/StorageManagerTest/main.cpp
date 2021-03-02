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

#include "Arduino.h"
#include "STORAGEMANAGER.h"

uint8_t Byte_Value = 99;
int16_t Short_Value = 30000;
int32_t Long_Value = 123456789;
float Float_Value = .01234;

StorageManagerClass STORAGEMANAGER;

void setup()
{
    Serial.begin(115200);
    Serial.println("Storage Manager exemplo de uso...");
    delay(250);

    STORAGEMANAGER.Write_8Bits(0, Byte_Value);   //1 BYTE  = APENAS 1 ENDEREÇO DA EEPROM
    STORAGEMANAGER.Write_16Bits(1, Short_Value); //2 BYTES = 2 ENDEREÇOS DA EEPROM
    STORAGEMANAGER.Write_32Bits(3, Long_Value);  //4 BYTES = 4 ENDEREÇOS DA EEPROM
    STORAGEMANAGER.Write_Float(7, Float_Value);  //4 BYTES = 4 ENDEREÇOS DA EEPROM
    //O PROXIMO ENDEREÇO QUE SE PODE SER UTILIZADO É A PARTIR DO 11º (POR QUE O FLOAT OCUPOU OS ENDEREÇOS 7,8,9 & 10 (4 BYTES))

    Serial.println(STORAGEMANAGER.Read_8Bits(0));
    Serial.println(STORAGEMANAGER.Read_16Bits(1));
    Serial.println(STORAGEMANAGER.Read_32Bits(3));
    Serial.println(STORAGEMANAGER.Read_Float(7), 5);
}

void loop()
{
}
