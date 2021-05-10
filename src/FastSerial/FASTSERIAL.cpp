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

#include "FASTSERIAL.h"
#include "HAL/HALSERIAL.h"
#include "PRINTF.h"

Fast_Serial FASTSERIAL;

void Fast_Serial::Initialization(void)
{
  HAL_SERIAL.Initialization();
  PRINTF.Initialization();
}

void Fast_Serial::Begin(uint8_t SerialPort, uint32_t BaudRate)
{
  HAL_SERIAL.Begin(SerialPort, BaudRate);
}

void Fast_Serial::UartSendData(uint8_t SerialPort)
{
  HAL_SERIAL.UartSendData(SerialPort);
}

bool Fast_Serial::TXFree(uint8_t SerialPort)
{
  return HAL_SERIAL.TXFree(SerialPort);
}

void Fast_Serial::UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
  HAL_SERIAL.UartBufferStore(UartBuffer, SerialPort);
}

uint8_t Fast_Serial::Read(uint8_t SerialPort)
{
  return HAL_SERIAL.Read(SerialPort);
}

uint8_t Fast_Serial::Available(uint8_t SerialPort)
{
  return HAL_SERIAL.Available(SerialPort);
}

uint8_t Fast_Serial::UsedTXBuffer(uint8_t SerialPort)
{
  return HAL_SERIAL.UsedTXBuffer(SerialPort);
}

void Fast_Serial::StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
  HAL_SERIAL.StoreTX(SerialPort, WriteTX);
}

void Fast_Serial::Write(uint8_t SerialPort, uint8_t WriteData)
{
  HAL_SERIAL.Write(SerialPort, WriteData);
}