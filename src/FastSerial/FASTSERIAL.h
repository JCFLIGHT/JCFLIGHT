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

#ifndef FASTSERIAL_H_
#define FASTSERIAL_H_
#include "Build/LIBDEPENDENCIES.h"
class Fast_Serial
{
public:
  void Initialization();
  void Begin(uint8_t SerialPort, uint32_t BaudRate);
  uint8_t Read(uint8_t SerialPort);
  void Write(uint8_t SerialPort, uint8_t WriteData);
  uint8_t Available(uint8_t SerialPort);
  bool TXFree(uint8_t SerialPort);
  uint8_t UsedTXBuffer(uint8_t SerialPort);
  void StoreTX(uint8_t SerialPort, uint8_t WriteTX);
  void UartSendData(uint8_t SerialPort);
  void UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort);
};
extern Fast_Serial FASTSERIAL;
#endif
