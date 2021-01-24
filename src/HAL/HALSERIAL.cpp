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

#include "HALSERIAL.h"
#include "HAL_AVR/AVRSERIAL.h"
#include "HAL_ESP32/ESP32SERIAL.h"

HALSerialClass HAL_SERIAL;

void HALSerialClass::Initialization()
{
    Serial_Initialization();
}

void HALSerialClass::Begin(uint8_t SerialPort, uint32_t BaudRate)
{
    Serial_Begin(SerialPort, BaudRate);
}

void HALSerialClass::UartSendData(uint8_t SerialPort)
{
    Serial_UartSendData(SerialPort);
}

bool HALSerialClass::TXFree(uint8_t SerialPort)
{
    return Serial_TXFree(SerialPort);
}

void HALSerialClass::UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
    Serial_UartBufferStore(UartBuffer, SerialPort);
}

uint8_t HALSerialClass::Read(uint8_t SerialPort)
{
    return Serial_Read(SerialPort);
}

uint8_t HALSerialClass::Available(uint8_t SerialPort)
{
    return Serial_Available(SerialPort);
}

uint8_t HALSerialClass::UsedTXBuffer(uint8_t SerialPort)
{
    return Serial_UsedTXBuffer(SerialPort);
}

void HALSerialClass::StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
    Serial_StoreTX(SerialPort, WriteTX);
}

void HALSerialClass::Write(uint8_t SerialPort, uint8_t WriteData)
{
    Serial_Write(SerialPort, WriteData);
}