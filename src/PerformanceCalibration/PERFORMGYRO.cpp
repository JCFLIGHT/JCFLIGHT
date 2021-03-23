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

#include "PERFORMGYRO.h"
#include "IMU/ACCGYROREAD.h"
#include "DEVICE.h"
#include "Buzzer/BUZZER.h"
#include "Math/MATHSUPPORT.h"

int16_t CalibratingGyroscope = ACC_1G;
int16_t StoredPreviousValueOfGyro[3] = {0, 0, 0};
int32_t StoredGyroSum[3] = {0, 0, 0};

void StartGyroCalibration()
{
    CalibratingGyroscope = ACC_1G;
}

bool GyroCalibrationRunning()
{
    return CalibratingGyroscope > 0;
}

void Gyroscope_Calibration()
{
    float GyroDeviation[3];
    static Device_Struct GyroDevice[3];

    if (CalibratingGyroscope > 0)
    {
        switch (CalibratingGyroscope)
        {

        case 1:
            GyroDeviation[ROLL] = DeviceStandardDeviation(&GyroDevice[ROLL]);
            GyroDeviation[PITCH] = DeviceStandardDeviation(&GyroDevice[PITCH]);
            GyroDeviation[YAW] = DeviceStandardDeviation(&GyroDevice[YAW]);
            //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
            if ((GyroDeviation[ROLL] > 32) || (GyroDeviation[PITCH] > 32) || (GyroDeviation[YAW] > 32))
            {
                CalibratingGyroscope = ACC_1G + 1; //REINICIA A CALIBRAÇÃO
                BEEPER.Play(BEEPER_ACTION_FAIL);   //SINALIZA COM O BUZZER QUE HOUVE UM ERRO
            }
            else
            {
                StoredGyroSum[ROLL] /= ACC_1G;
                StoredGyroSum[PITCH] /= ACC_1G;
                StoredGyroSum[YAW] /= ACC_1G;
                BEEPER.Play(BEEPER_CALIBRATION_DONE); //SINALIZA COM O BUZZER QUE TUDO OCORREU BEM
            }
            break;

        case ACC_1G:
            StoredGyroSum[ROLL] = 0;
            StoredGyroSum[PITCH] = 0;
            StoredGyroSum[YAW] = 0;
            DeviceClear(&GyroDevice[ROLL]);
            DeviceClear(&GyroDevice[PITCH]);
            DeviceClear(&GyroDevice[YAW]);
            break;

        default:
            StoredGyroSum[ROLL] += IMU.Gyroscope.Read[ROLL];
            StoredGyroSum[PITCH] += IMU.Gyroscope.Read[PITCH];
            StoredGyroSum[YAW] += IMU.Gyroscope.Read[YAW];
            DevicePushValues(&GyroDevice[ROLL], IMU.Gyroscope.Read[ROLL]);
            DevicePushValues(&GyroDevice[PITCH], IMU.Gyroscope.Read[PITCH]);
            DevicePushValues(&GyroDevice[YAW], IMU.Gyroscope.Read[YAW]);
            break;
        }
        CalibratingGyroscope--;
    }
    else
    {
        //ROLL
        IMU.Gyroscope.Read[ROLL] = (IMU.Gyroscope.Read[ROLL] - StoredGyroSum[ROLL]);
        IMU.Gyroscope.Read[ROLL] = Constrain_16Bits(IMU.Gyroscope.Read[ROLL], StoredPreviousValueOfGyro[ROLL] - 800, StoredPreviousValueOfGyro[ROLL] + 800);
        StoredPreviousValueOfGyro[ROLL] = IMU.Gyroscope.Read[ROLL];
        IMU.Gyroscope.Read[ROLL] = IMU.Gyroscope.Read[ROLL] * GYRO_SCALE;
        //PITCH
        IMU.Gyroscope.Read[PITCH] = (IMU.Gyroscope.Read[PITCH] - StoredGyroSum[PITCH]);
        IMU.Gyroscope.Read[PITCH] = Constrain_16Bits(IMU.Gyroscope.Read[PITCH], StoredPreviousValueOfGyro[PITCH] - 800, StoredPreviousValueOfGyro[PITCH] + 800);
        StoredPreviousValueOfGyro[PITCH] = IMU.Gyroscope.Read[PITCH];
        IMU.Gyroscope.Read[PITCH] = IMU.Gyroscope.Read[PITCH] * GYRO_SCALE;
        //YAW
        IMU.Gyroscope.Read[YAW] = (IMU.Gyroscope.Read[YAW] - StoredGyroSum[YAW]);
        IMU.Gyroscope.Read[YAW] = Constrain_16Bits(IMU.Gyroscope.Read[YAW], StoredPreviousValueOfGyro[YAW] - 800, StoredPreviousValueOfGyro[YAW] + 800);
        StoredPreviousValueOfGyro[YAW] = IMU.Gyroscope.Read[YAW];
        IMU.Gyroscope.Read[YAW] = IMU.Gyroscope.Read[YAW] * GYRO_SCALE;
    }
}