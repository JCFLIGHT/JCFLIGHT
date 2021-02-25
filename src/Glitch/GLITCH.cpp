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

#include "GLITCH.h"
#include "I2C/I2C.h"
#include "GPS/GPSSTATES.h"

GlitchClass GLITCH;

bool GlitchClass::CheckGPS(void)
{
    if (Get_GPS_In_Bad_Condition())
    {
        return false;
    }
    return true;
}

bool GlitchClass::CheckBarometer(void)
{
    //BAROMETRO NÃO ENCONTRADO NO BARRAMENTO I2C
    if (!I2C.BarometerFound)
    {
        return false;
    }
    return true;
}

bool GlitchClass::CheckCompass(void)
{
    //COMPASS NÃO ENCONTRADO NO BARRAMENTO I2C
    if (!I2C.CompassFound)
    {
        return false;
    }
    return true;
}