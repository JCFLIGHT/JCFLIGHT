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

#include "EEPROMCHECK.h"
#include "FastSerial/PRINTF.h"
#include "EEPROMSTORAGE.h"
#include "Scheduler/SCHEDULERTIME.h"

uint16_t EPPROM_Address = 0;

#ifdef __AVR_ATmega2560__
void Operator_Check_Values_In_Address(uint16_t Size)
{
    while (true)
    {
        FastSerialPrintln(PSTR("Address:%d ValorGuardado:%d\n"),
                          EPPROM_Address,
                          STORAGEMANAGER.Read_8Bits(EPPROM_Address));
        EPPROM_Address++;
        if (EPPROM_Address == Size)
        {
            FastSerialPrintln(PSTR("Completo!\n"));
            while (true)
                ;
        }
        SCHEDULERTIME.Sleep(4);
    }
}

#elif defined ESP32

void Operator_Check_Values_In_Address(uint16_t Size)
{
    while (true)
    {
        Serial.print("Address:");
        Serial.print(EPPROM_Address);
        Serial.print("  ");
        Serial.print("Valor Guardado:");
        Serial.println(STORAGEMANAGER.Read_8Bits(EPPROM_Address));
        EPPROM_Address++;
        if (EPPROM_Address == Size)
        {
            Serial.println("Completo!");
            while (true)
                ;
        }
        SCHEDULERTIME.Sleep(4);
    }
}

#elif defined __arm__

void Operator_Check_Values_In_Address(uint16_t Size)
{
    while (true)
    {
    }
}
#endif