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
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "GPS/GPSREAD.h"
#include "BAR/BAR.h"

//*******************************************************
//FAST SERIAL
//*******************************************************

Fast_Serial FASTSERIAL;

#ifdef __AVR_ATmega2560__

static uint8_t UARTBufferRX[256][4];
static uint8_t UARTBufferTX[128][4];
static volatile uint8_t UARTHeadRX[4];
static volatile uint8_t UARTHeadTX[4];
static volatile uint8_t UARTTailRX[4];
static volatile uint8_t UARTTailTX[4];

void Fast_Serial::Initialization()
{
  //DEBUG E GCS
  FASTSERIAL.Begin(UART0, 115200);
  //GPS
  FASTSERIAL.Begin(UART1, 57600);
  GPS_SerialInit(57600);
  //IBUS & SBUS
  if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 0)
    FASTSERIAL.Begin(UART2, 115200);
  if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 1)
  {
    //CONFIGURAÇÃO DA UART2 PARA SBUS
    FASTSERIAL.Begin(UART2, 100000);
    UCSR2C |= (1 << UPM21) | (1 << USBS2);
  }
  else if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 2)
  {
    //CONFIGURAÇÃO DA UART2 PARA IBUS
    FASTSERIAL.Begin(UART2, 115200);
  }
  //MATEK3901L0X
  FASTSERIAL.Begin(UART3, 115200);
}

//SERIAL 0 DO MEGA (DEBUG E OSD)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART0_UDRE_vect)
{
  uint8_t TXBuffer = UARTTailTX[UART0];
  if (UARTHeadTX[UART0] != TXBuffer)
  {
    if (++TXBuffer >= 128)
      TXBuffer = 0;
    UDR0 = UARTBufferTX[TXBuffer][UART0];
    UARTTailTX[UART0] = TXBuffer;
  }
  if (TXBuffer == UARTHeadTX[UART0])
    UCSR0B &= ~(1 << UDRIE0);
}

//SERIAL 1 DO MEGA (GPS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART1_UDRE_vect)
{
  uint8_t TXBuffer = UARTTailTX[UART1];
  if (UARTHeadTX[UART1] != TXBuffer)
  {
    if (++TXBuffer >= 128)
      TXBuffer = 0;
    UDR1 = UARTBufferTX[TXBuffer][UART1];
    UARTTailTX[UART1] = TXBuffer;
  }
  if (TXBuffer == UARTHeadTX[UART1])
    UCSR1B &= ~(1 << UDRIE1);
}

//SERIAL 2 DO MEGA
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART2_UDRE_vect)
{
  uint8_t TXBuffer = UARTTailTX[UART2];
  if (UARTHeadTX[UART2] != TXBuffer)
  {
    if (++TXBuffer >= 128)
      TXBuffer = 0;
    UDR2 = UARTBufferTX[TXBuffer][UART2];
    UARTTailTX[UART2] = TXBuffer;
  }
  if (TXBuffer == UARTHeadTX[UART2])
    UCSR2B &= ~(1 << UDRIE2);
}

//SERIAL 3 DO MEGA
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART3_UDRE_vect)
{
  uint8_t TXBuffer = UARTTailTX[UART3];
  if (UARTHeadTX[UART3] != TXBuffer)
  {
    if (++TXBuffer >= 128)
      TXBuffer = 0;
    UDR3 = UARTBufferTX[TXBuffer][UART3];
    UARTTailTX[UART3] = TXBuffer;
  }
  if (TXBuffer == UARTHeadTX[UART3])
    UCSR3B &= ~(1 << UDRIE3);
}

void Fast_Serial::UartSendData(uint8_t SerialPort)
{
  switch (SerialPort)
  {

  case UART0:
    UCSR0B |= (1 << UDRIE0);
    break;

  case UART1:
    UCSR1B |= (1 << UDRIE1);
    break;

  case UART2:
    UCSR2B |= (1 << UDRIE2);
    break;

  case UART3:
    UCSR3B |= (1 << UDRIE3);
    break;
  }
}

bool Fast_Serial::Flush(uint8_t SerialPort)
{
  return (UARTHeadTX[SerialPort] == UARTTailTX[SerialPort]);
}

void Fast_Serial::Begin(uint8_t SerialPort, uint32_t BaudRate)
{
  uint8_t h = ((F_CPU / 4 / BaudRate - 1) / 2) >> 8;
  uint8_t l = ((F_CPU / 4 / BaudRate - 1) / 2);
  switch (SerialPort)
  {

  case UART0:
    UCSR0A = (1 << U2X0);
    UBRR0H = h;
    UBRR0L = l;
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    break;

  case UART1:
    UCSR1A = (1 << U2X1);
    UBRR1H = h;
    UBRR1L = l;
    UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    break;

  case UART2:
    UCSR2A = (1 << U2X2);
    UBRR2H = h;
    UBRR2L = l;
    UCSR2B |= (1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2);
    break;

  case UART3:
    UCSR3A = (1 << U2X3);
    UBRR3H = h;
    UBRR3L = l;
    UCSR3B |= (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3);
    break;
  }
}

void Fast_Serial::UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
  uint8_t RXBuffer = UARTHeadRX[SerialPort];
  UARTBufferRX[RXBuffer++][SerialPort] = UartBuffer;
  if (RXBuffer >= 256)
    RXBuffer = 0;
  UARTHeadRX[SerialPort] = RXBuffer;
}

ISR(USART0_RX_vect)
{
  FASTSERIAL.UartBufferStore(UDR0, UART0);
}
ISR(USART1_RX_vect)
{
  FASTSERIAL.UartBufferStore(UDR1, UART1);
}
ISR(USART2_RX_vect)
{
  FASTSERIAL.UartBufferStore(UDR2, UART2);
}
ISR(USART3_RX_vect)
{
  FASTSERIAL.UartBufferStore(UDR3, UART3);
}

uint8_t Fast_Serial::Read(uint8_t SerialPort)
{
  uint8_t CheckRXBuffer = UARTTailRX[SerialPort];
  uint8_t RXBuffer = UARTBufferRX[CheckRXBuffer][SerialPort];
  if (UARTHeadRX[SerialPort] != CheckRXBuffer)
  {
    if (++CheckRXBuffer >= 256)
      CheckRXBuffer = 0;
    UARTTailRX[SerialPort] = CheckRXBuffer;
  }
  return RXBuffer;
}

uint8_t Fast_Serial::Available(uint8_t SerialPort)
{
  return ((uint8_t)(UARTHeadRX[SerialPort] - UARTTailRX[SerialPort])) % 256;
}

uint8_t Fast_Serial::TXBuffer(uint8_t SerialPort)
{
  return ((uint8_t)(UARTHeadTX[SerialPort] - UARTTailTX[SerialPort])) % 128;
}

void Fast_Serial::TX_Send(uint8_t SerialPort, uint8_t WriteTX)
{
  uint8_t TXBuffer = UARTHeadTX[SerialPort];
  if (++TXBuffer >= 128)
    TXBuffer = 0;
  UARTBufferTX[TXBuffer][SerialPort] = WriteTX;
  UARTHeadTX[SerialPort] = TXBuffer;
}

void Fast_Serial::Write(uint8_t SerialPort, uint8_t WriteData)
{
  TX_Send(SerialPort, WriteData);
  UartSendData(SerialPort);
}

#elif defined __arm__

void Fast_Serial::Initialization()
{
  //DEBUG E GCS
  FASTSERIAL.Begin(UART0, 115200);
  //GPS
  FASTSERIAL.Begin(UART1, 57600);
  GPS_SerialInit(57600);
  //IBUS & SBUS
  if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 0)
    FASTSERIAL.Begin(UART2, 115200);
  if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 1)
  {
    //CONFIGURAÇÃO DA UART2 PARA SBUS
    FASTSERIAL.Begin(UART2, 100000);
  }
  else if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) == 2)
  {
    //CONFIGURAÇÃO DA UART2 PARA IBUS
    FASTSERIAL.Begin(UART2, 115200);
  }
  //MATEK3901L0X
  FASTSERIAL.Begin(UART3, 115200);
}

void Fast_Serial::UartSendData(uint8_t SerialPort)
{
  switch (SerialPort)
  {

  case UART0:
    break;

  case UART1:
    break;

  case UART2:
    break;

  case UART3:
    break;
  }
}

bool Fast_Serial::Flush(uint8_t SerialPort)
{
  return 0;
}

void Fast_Serial::Begin(uint8_t SerialPort, uint32_t BaudRate)
{
  switch (SerialPort)
  {

  case UART0:
    break;

  case UART1:
    break;

  case UART2:
    break;

  case UART3:
    break;
  }
}

void Fast_Serial::UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
}

uint8_t Fast_Serial::Read(uint8_t SerialPort)
{
  return 0;
}

uint8_t Fast_Serial::Available(uint8_t SerialPort)
{
  return 0;
}

uint8_t Fast_Serial::TXBuffer(uint8_t SerialPort)
{
  return 0;
}

void Fast_Serial::TX_Send(uint8_t SerialPort, uint8_t WriteTX)
{
}

void Fast_Serial::Write(uint8_t SerialPort, uint8_t WriteData)
{
}

#endif