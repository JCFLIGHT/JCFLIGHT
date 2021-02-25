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

#include "GPSSERIALREAD.h"
#include "FastSerial/FASTSERIAL.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPS/GPSREAD.h"
#include "Common/ENUM.h"

void GPS_Serial_Read()
{
    uint8_t SerialAvailableGPS;
    uint8_t SerialReadGPS;
    uint8_t CheckGPSTXBuffer;
    SerialAvailableGPS = FASTSERIAL.Available(UART_NUMB_1);
    while (SerialAvailableGPS--)
    {
        CheckGPSTXBuffer = FASTSERIAL.UsedTXBuffer(UART_NUMB_1);
        if (CheckGPSTXBuffer > 78)
        {
            return;
        }
        SerialReadGPS = FASTSERIAL.Read(UART_NUMB_1);
        GPS_SerialRead(SerialReadGPS);
    }
}