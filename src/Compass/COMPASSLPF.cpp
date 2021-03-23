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
#include "BitArray/BITARRAY.h"
#include "Common/ENUM.h"
#include "Common/STRUCTS.h"
#include "IMU/ACCGYROREAD.h"

CompassLPFClass COMPASSLPF;

void CompassLPFClass::ApplyFilter()
{
    //APLICA O LPF NO COMPASS PARA EVITAR SPIKES DURANTE A CALIBRAÇÃO DO COMPASS
    if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
        IMU.Compass.ReadSmooth[ROLL] = IMU.Compass.ReadSmooth[ROLL] * 0.9f + IMU.Compass.Read[ROLL] * 0.1f;
        IMU.Compass.ReadSmooth[PITCH] = IMU.Compass.ReadSmooth[PITCH] * 0.9f + IMU.Compass.Read[PITCH] * 0.1f;
        IMU.Compass.ReadSmooth[YAW] = IMU.Compass.ReadSmooth[YAW] * 0.9f + IMU.Compass.Read[YAW] * 0.1f;
    }
}