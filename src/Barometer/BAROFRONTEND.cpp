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

#include "BAROFRONTEND.h"
#include "BAROBACKEND.h"
#include "BAROREAD.h"
#include "Math/MATHSUPPORT.h"

//https://en.wikipedia.org/wiki/Equivalent_airspeed

int32_t EAS2TAS;
int32_t Last_EAS2TAS;

//FATOR DE ESCALA PARA CONVERTER A VELOCIDADE EQUIVALENTE DO TUBO DE PITOT EM VELOCIDADE REAL
int32_t Get_EAS2TAS(void)
{
    if ((ABS(Barometer.Altitude.Actual - Last_EAS2TAS) < 100) && (EAS2TAS != 0))
    {
        return EAS2TAS;
    }
    int32_t TempKelvin = ((float)Barometer.Calibration.GroundTemperature) + 27315 - 0.0065f * Barometer.Altitude.Actual;
    EAS2TAS = Fast_SquareRoot(1.225f / ((float)Barometer.Raw.PressureFiltered / (287.26f * TempKelvin)));
    Last_EAS2TAS = Barometer.Altitude.Actual;
    return EAS2TAS;
}