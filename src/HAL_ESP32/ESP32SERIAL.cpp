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

#include "ESP32SERIAL.h"
#include "Common/ENUM.h"

#if ESP32

void Serial_UartSendData(uint8_t SerialPort)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        break;

    case UART_NUMB_1:
        break;

    case UART_NUMB_2:
        break;

    case UART_NUMB_3:
        break;
    }
}

bool Serial_TXFree(uint8_t SerialPort)
{
    return true;
}

void Serial_Begin(uint8_t SerialPort, uint32_t BaudRate)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        Serial.begin(BaudRate);
        break;

    case UART_NUMB_1:
        Serial1.begin(BaudRate);
        break;

    case UART_NUMB_2:
        Serial2.begin(BaudRate);
        break;

    case UART_NUMB_3:
        break;
    }
}

void Serial_UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
}

uint8_t Serial_Read(uint8_t SerialPort)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        return Serial.read();
        break;

    case UART_NUMB_1:
        return Serial1.read();
        break;

    case UART_NUMB_2:
        return Serial2.read();
        break;

    case UART_NUMB_3:
        break;
    }
    return 0;
}

uint8_t Serial_Available(uint8_t SerialPort)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        return Serial.available();
        break;

    case UART_NUMB_1:
        return Serial1.available();
        break;

    case UART_NUMB_2:
        return Serial2.available();
        break;

    case UART_NUMB_3:
        break;
    }
    return 0;
}

uint8_t Serial_UsedTXBuffer(uint8_t SerialPort)
{
    return 0;
}

void Serial_StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
}

void Serial_Write(uint8_t SerialPort, uint8_t WriteData)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        Serial.write(WriteData);
        break;

    case UART_NUMB_1:
        Serial1.write(WriteData);
        break;

    case UART_NUMB_2:
        Serial2.write(WriteData);
        break;

    case UART_NUMB_3:
        break;
    }
}

#endif