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

#ifndef AVRSERIAL_H_
#define AVRSERIAL_H_
#ifdef __AVR_ATmega2560__
#include "Build/LIBDEPENDENCIES.h"
void Serial_Initialization();
void Serial_Begin(uint8_t SerialPort, uint32_t BaudRate);
uint8_t Serial_Read(uint8_t SerialPort);
void Serial_Write(uint8_t SerialPort, uint8_t WriteData);
uint8_t Serial_Available(uint8_t SerialPort);
bool Serial_TXFree(uint8_t SerialPort);
uint8_t Serial_UsedTXBuffer(uint8_t SerialPort);
void Serial_StoreTX(uint8_t SerialPort, uint8_t WriteTX);
void Serial_UartSendData(uint8_t SerialPort);
void Serial_UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort);
#endif
#endif