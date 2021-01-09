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

#include "COMPASSLPF.h"
#include "Common/VARIABLES.h"

CompassLPFClass COMPASSLPF;

void CompassLPFClass::Apply()
{
    //APLICA O LPF NO COMPASS PARA EVITAR SPIKES DURANTE A CALIBRAÇÃO DO COMPASS
    if (!COMMAND_ARM_DISARM)
    {
        MagnetometerRead[ROLL] = MagnetometerRead[ROLL] * 0.9f + IMU.CompassRead[ROLL] * 0.1f;
        MagnetometerRead[PITCH] = MagnetometerRead[PITCH] * 0.9f + IMU.CompassRead[PITCH] * 0.1f;
        MagnetometerRead[YAW] = MagnetometerRead[YAW] * 0.9f + IMU.CompassRead[YAW] * 0.1f;
    }
}