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

#include "ORIENTATION.h"
#include "COMPASSREAD.h"
#include "I2C/I2C.h"
#include "Common/STRUCTS.h"
#include "Common/ENUM.h"
#include "GPS/DJINAZAGPS.h"
#include "IMU/ACCGYROREAD.h"

ClassCompassOrientation COMPASSORIENTATION;

void ClassCompassOrientation::SetOrientation(uint8_t _CompassType)
{

    switch (_CompassType)
    {

    case COMPASS_AK8975:
        //ORIENTAÇÃO PARA O COMPASS AK8975
        IMU.Compass.Read[ROLL] = (I2CResources.Buffer.Data[1] << 8) | I2CResources.Buffer.Data[0];
        IMU.Compass.Read[PITCH] = (I2CResources.Buffer.Data[3] << 8) | I2CResources.Buffer.Data[2];
        IMU.Compass.Read[YAW] = (I2CResources.Buffer.Data[5] << 8) | I2CResources.Buffer.Data[4];
        break;

    case COMPASS_HMC5883:
        //ORIENTAÇÃO PARA O COMPASS HMC5883
        IMU.Compass.Read[ROLL] = (I2CResources.Buffer.Data[0] << 8) | I2CResources.Buffer.Data[1];
        IMU.Compass.Read[PITCH] = (I2CResources.Buffer.Data[4] << 8) | I2CResources.Buffer.Data[5];
        IMU.Compass.Read[YAW] = (I2CResources.Buffer.Data[2] << 8) | I2CResources.Buffer.Data[3];
        break;

    case COMPASS_QMC5883:
        //ORIENTAÇÃO PARA O COMPASS QMC5883
        IMU.Compass.Read[ROLL] = (I2CResources.Buffer.Data[1] << 8) | I2CResources.Buffer.Data[0];
        IMU.Compass.Read[PITCH] = (I2CResources.Buffer.Data[3] << 8) | I2CResources.Buffer.Data[2];
        IMU.Compass.Read[YAW] = (I2CResources.Buffer.Data[5] << 8) | I2CResources.Buffer.Data[4];
        break;

    case COMPASS_DJI_NAZA:
        //ORIENTAÇÃO PARA O COMPASS DO GPS DA NAZA
        IMU.Compass.Read[ROLL] = DJINaza_Compass_Roll;
        IMU.Compass.Read[PITCH] = DJINaza_Compass_Pitch;
        IMU.Compass.Read[YAW] = DJINaza_Compass_Yaw;
        break;
    }
}