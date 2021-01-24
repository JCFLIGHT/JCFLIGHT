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

#include "AVRSERIAL.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "GPS/GPSREAD.h"
#include "BAR/BAR.h"

#ifdef __AVR_ATmega2560__

static uint8_t UARTBufferRX[256][4];
static uint8_t UARTBufferTX[128][4];
static volatile uint8_t UARTHeadRX[4];
static volatile uint8_t UARTHeadTX[4];
static volatile uint8_t UARTTailRX[4];
static volatile uint8_t UARTTailTX[4];

void Serial_Initialization()
{
    //DEBUG E GCS
    Serial_Begin(UART_NUMB_0, 115200);
    //GPS
    Serial_Begin(UART_NUMB_1, 57600);
    GPS_SerialInit(57600);
    //IBUS & SBUS
    if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 0)
    {
        Serial_Begin(UART_NUMB_2, 115200);
    }
    if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 1)
    {
        //CONFIGURAÇÃO DA UART_NUMB_2 PARA SBUS
        Serial_Begin(UART_NUMB_2, 100000);
        UCSR2C |= (1 << UPM21) | (1 << USBS2);
    }
    else if (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 2)
    {
        //CONFIGURAÇÃO DA UART_NUMB_2 PARA IBUS
        Serial_Begin(UART_NUMB_2, 115200);
    }
    //MATEK3901L0X,SD LOGGER & OSD
    Serial_Begin(UART_NUMB_3, 115200);
}

void Serial_Begin(uint8_t SerialPort, uint32_t BaudRate)
{
    uint8_t h = ((F_CPU / 4 / BaudRate - 1) / 2) >> 8;
    uint8_t l = ((F_CPU / 4 / BaudRate - 1) / 2);
    switch (SerialPort)
    {

    case UART_NUMB_0:
        UCSR0A = (1 << U2X0);
        UBRR0H = h;
        UBRR0L = l;
        UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
        break;

    case UART_NUMB_1:
        UCSR1A = (1 << U2X1);
        UBRR1H = h;
        UBRR1L = l;
        UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
        break;

    case UART_NUMB_2:
        UCSR2A = (1 << U2X2);
        UBRR2H = h;
        UBRR2L = l;
        UCSR2B |= (1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2);
        break;

    case UART_NUMB_3:
        UCSR3A = (1 << U2X3);
        UBRR3H = h;
        UBRR3L = l;
        UCSR3B |= (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3);
        break;
    }
}

void Serial_UartSendData(uint8_t SerialPort)
{
    switch (SerialPort)
    {

    case UART_NUMB_0:
        UCSR0B |= (1 << UDRIE0);
        break;

    case UART_NUMB_1:
        UCSR1B |= (1 << UDRIE1);
        break;

    case UART_NUMB_2:
        UCSR2B |= (1 << UDRIE2);
        break;

    case UART_NUMB_3:
        UCSR3B |= (1 << UDRIE3);
        break;
    }
}

bool Serial_TXFree(uint8_t SerialPort)
{
    return (UARTHeadTX[SerialPort] == UARTTailTX[SerialPort]);
}

void Serial_UartBufferStore(uint8_t UartBuffer, uint8_t SerialPort)
{
    uint8_t RXBuffer = UARTHeadRX[SerialPort];
    UARTBufferRX[RXBuffer++][SerialPort] = UartBuffer;
    if (RXBuffer >= 256)
    {
        RXBuffer = 0;
    }
    UARTHeadRX[SerialPort] = RXBuffer;
}

uint8_t Serial_Read(uint8_t SerialPort)
{
    uint8_t CheckRXBuffer = UARTTailRX[SerialPort];
    uint8_t RXBuffer = UARTBufferRX[CheckRXBuffer][SerialPort];
    if (UARTHeadRX[SerialPort] != CheckRXBuffer)
    {
        if (++CheckRXBuffer >= 256)
        {
            CheckRXBuffer = 0;
        }
        UARTTailRX[SerialPort] = CheckRXBuffer;
    }
    return RXBuffer;
}

uint8_t Serial_Available(uint8_t SerialPort)
{
    return ((uint8_t)(UARTHeadRX[SerialPort] - UARTTailRX[SerialPort])) % 256;
}

uint8_t Serial_UsedTXBuffer(uint8_t SerialPort)
{
    return ((uint8_t)(UARTHeadTX[SerialPort] - UARTTailTX[SerialPort])) % 128;
}

void Serial_StoreTX(uint8_t SerialPort, uint8_t WriteTX)
{
    uint8_t TXBuffer = UARTHeadTX[SerialPort];
    if (++TXBuffer >= 128)
    {
        TXBuffer = 0;
    }
    UARTBufferTX[TXBuffer][SerialPort] = WriteTX;
    UARTHeadTX[SerialPort] = TXBuffer;
}

void Serial_Write(uint8_t SerialPort, uint8_t WriteData)
{
    Serial_StoreTX(SerialPort, WriteData);
    Serial_UartSendData(SerialPort);
}

//SERIAL 0 DO MEGA (DEBUG & GCS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART0_UDRE_vect)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_0];
    if (UARTHeadTX[UART_NUMB_0] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        UDR0 = UARTBufferTX[TXBuffer][UART_NUMB_0];
        UARTTailTX[UART_NUMB_0] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_0])
    {
        UCSR0B &= ~(1 << UDRIE0);
    }
}

//SERIAL 1 DO MEGA (GPS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART1_UDRE_vect)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_1];
    if (UARTHeadTX[UART_NUMB_1] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        UDR1 = UARTBufferTX[TXBuffer][UART_NUMB_1];
        UARTTailTX[UART_NUMB_1] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_1])
    {
        UCSR1B &= ~(1 << UDRIE1);
    }
}

//SERIAL 2 DO MEGA (SBUS & IBUS)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART2_UDRE_vect)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_2];
    if (UARTHeadTX[UART_NUMB_2] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        UDR2 = UARTBufferTX[TXBuffer][UART_NUMB_2];
        UARTTailTX[UART_NUMB_2] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_2])
    {
        UCSR2B &= ~(1 << UDRIE2);
    }
}

//SERIAL 3 DO MEGA (MATEK3901L0X,SD LOGGER & OSD)
//ROTINA DE INTERRUPÇÃO PARA O PINO TX
ISR(USART3_UDRE_vect)
{
    uint8_t TXBuffer = UARTTailTX[UART_NUMB_3];
    if (UARTHeadTX[UART_NUMB_3] != TXBuffer)
    {
        if (++TXBuffer >= 128)
        {
            TXBuffer = 0;
        }
        UDR3 = UARTBufferTX[TXBuffer][UART_NUMB_3];
        UARTTailTX[UART_NUMB_3] = TXBuffer;
    }
    if (TXBuffer == UARTHeadTX[UART_NUMB_3])
    {
        UCSR3B &= ~(1 << UDRIE3);
    }
}

ISR(USART0_RX_vect)
{
    Serial_UartBufferStore(UDR0, UART_NUMB_0);
}

ISR(USART1_RX_vect)
{
    Serial_UartBufferStore(UDR1, UART_NUMB_1);
}

ISR(USART2_RX_vect)
{
    Serial_UartBufferStore(UDR2, UART_NUMB_2);
}

ISR(USART3_RX_vect)
{
    Serial_UartBufferStore(UDR3, UART_NUMB_3);
}

#endif