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

ClassCompassOrientation COMPASSORIENTATION;

void ClassCompassOrientation::SetOrientation(uint8_t Orientation, uint8_t _CompassType)
{

    switch (Orientation)
    {

    case GPS_ONBOARD_COMPASS:
        //ORIENTAÇÃO PARA COMPASS ONBOARD DOS GPS M7 E M8
        if (_CompassType == COMPASS_AK8975)
        {
            //ORIENTAÇÃO PARA O COMPASS AK8975
            IMU.CompassRead[ROLL] = -((BufferData[1] << 8) | BufferData[0]);
            IMU.CompassRead[PITCH] = -((BufferData[3] << 8) | BufferData[2]);
            IMU.CompassRead[YAW] = ((BufferData[5] << 8) | BufferData[4]);
            return;
        }
        else if (_CompassType == COMPASS_HMC5843)
        {
            //ORIENTAÇÃO PARA O COMPASS HMC5843
            IMU.CompassRead[ROLL] = -((BufferData[0] << 8) | BufferData[1]);
            IMU.CompassRead[PITCH] = -((BufferData[2] << 8) | BufferData[3]);
            IMU.CompassRead[YAW] = ((BufferData[4] << 8) | BufferData[5]);
            return;
        }
        else if (_CompassType == COMPASS_HMC5883)
        {
            if (COMPASS.FakeHMC5883Address != ADDRESS_COMPASS_QMC5883)
            {
                //ORIENTAÇÃO PARA O COMPASS HMC5883
                IMU.CompassRead[ROLL] = -((BufferData[0] << 8) | BufferData[1]);
                IMU.CompassRead[PITCH] = -((BufferData[4] << 8) | BufferData[5]);
                IMU.CompassRead[YAW] = ((BufferData[2] << 8) | BufferData[3]);
            }
            else
            {
                //ORIENTAÇÃO PARA O COMPASS QMC5883
                IMU.CompassRead[ROLL] = -((BufferData[1] << 8) | BufferData[0]);
                IMU.CompassRead[PITCH] = -((BufferData[3] << 8) | BufferData[2]);
                IMU.CompassRead[YAW] = ((BufferData[5] << 8) | BufferData[4]);
            }
            return;
        }
        break;

    case EXTERNAL_COMPASS:
        //ORIENTAÇÃO NORMAL PARA COMPASS EXTERNO
        if (_CompassType == COMPASS_AK8975)
        {
            //ORIENTAÇÃO PARA O COMPASS AK8975
            IMU.CompassRead[ROLL] = ((BufferData[1] << 8) | BufferData[0]);
            IMU.CompassRead[PITCH] = ((BufferData[3] << 8) | BufferData[2]);
            IMU.CompassRead[YAW] = -((BufferData[5] << 8) | BufferData[4]);
            return;
        }
        else if (_CompassType == COMPASS_HMC5843)
        {
            //ORIENTAÇÃO PARA O COMPASS HMC5843
            IMU.CompassRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]);
            IMU.CompassRead[PITCH] = ((BufferData[2] << 8) | BufferData[3]);
            IMU.CompassRead[YAW] = -((BufferData[4] << 8) | BufferData[5]);
            return;
        }
        else if (_CompassType == COMPASS_HMC5883)
        {
            if (COMPASS.FakeHMC5883Address != ADDRESS_COMPASS_QMC5883)
            {
                //ORIENTAÇÃO PARA O COMPASS HMC5883
                IMU.CompassRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]);
                IMU.CompassRead[PITCH] = ((BufferData[4] << 8) | BufferData[5]);
                IMU.CompassRead[YAW] = -((BufferData[2] << 8) | BufferData[3]);
            }
            else
            {
                //ORIENTAÇÃO PARA O COMPASS QMC5883
                IMU.CompassRead[ROLL] = ((BufferData[1] << 8) | BufferData[0]);
                IMU.CompassRead[PITCH] = ((BufferData[3] << 8) | BufferData[2]);
                IMU.CompassRead[YAW] = -((BufferData[5] << 8) | BufferData[4]);
            }
            return;
        }
        break;
    }
}