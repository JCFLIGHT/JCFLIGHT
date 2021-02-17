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

#include "BAROBACKEND.h"
#include "Common/STRUCTS.h"
#include "MS5611.h"
#include "BMP280.h"
#include "I2C/I2C.h"

uint8_t BaroType = 0; //DETECTA O BARÔMETRO AUTOMATICAMENTE

void SetBaroType(uint8_t _BaroType)
{
    if (_BaroType == ADDRESS_BAROMETER_MS5611)
    {
        BaroType = BAROMETER_MS5611;
        return;
    }
    if (_BaroType == ADDRESS_BAROMETER_BMP280)
    {
        BaroType = BAROMETER_BMP280;
        return;
    }
}

void Baro_Initialization()
{
    if (BaroType == BAROMETER_MS5611)
    {
        MS5611_Initialization();
        return;
    }
    if (BaroType == BAROMETER_BMP280)
    {
        BMP280_Initialization();
        return;
    }
}

void Barometer_Update()
{
    if (!I2C.BarometerFound)
        return;
    if (BaroType == BAROMETER_MS5611)
    {
        MS5611_Update();
        return;
    }
    if (BaroType == BAROMETER_BMP280)
    {
        BMP280_Update();
        return;
    }
}