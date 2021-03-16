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
            if ((GyroDeviation[ROLL] > 32) || (GyroDeviation[PITCH] > 32) || (GyroDeviation[YAW] > 32))
            {                                      //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
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
            StoredGyroSum[ROLL] += IMU.GyroscopeRead[ROLL];
            StoredGyroSum[PITCH] += IMU.GyroscopeRead[PITCH];
            StoredGyroSum[YAW] += IMU.GyroscopeRead[YAW];
            DevicePushValues(&GyroDevice[ROLL], IMU.GyroscopeRead[ROLL] * GYRO_SCALE);
            DevicePushValues(&GyroDevice[PITCH], IMU.GyroscopeRead[PITCH] * GYRO_SCALE);
            DevicePushValues(&GyroDevice[YAW], IMU.GyroscopeRead[YAW] * GYRO_SCALE);
            break;
        }
        CalibratingGyroscope--;
    }
    else
    {
        //ROLL
        IMU.GyroscopeRead[ROLL] = (IMU.GyroscopeRead[ROLL] - StoredGyroSum[ROLL]) * GYRO_SCALE;
        IMU.GyroscopeRead[ROLL] = Constrain_16Bits(IMU.GyroscopeRead[ROLL], StoredPreviousValueOfGyro[ROLL] - 800, StoredPreviousValueOfGyro[ROLL] + 800);
        StoredPreviousValueOfGyro[ROLL] = IMU.GyroscopeRead[ROLL];
        //PITCH
        IMU.GyroscopeRead[PITCH] = (IMU.GyroscopeRead[PITCH] - StoredGyroSum[PITCH]) * GYRO_SCALE;
        IMU.GyroscopeRead[PITCH] = Constrain_16Bits(IMU.GyroscopeRead[PITCH], StoredPreviousValueOfGyro[PITCH] - 800, StoredPreviousValueOfGyro[PITCH] + 800);
        StoredPreviousValueOfGyro[PITCH] = IMU.GyroscopeRead[PITCH];
        //YAW
        IMU.GyroscopeRead[YAW] = (IMU.GyroscopeRead[YAW] - StoredGyroSum[YAW]) * GYRO_SCALE;
        IMU.GyroscopeRead[YAW] = Constrain_16Bits(IMU.GyroscopeRead[YAW], StoredPreviousValueOfGyro[YAW] - 800, StoredPreviousValueOfGyro[YAW] + 800);
        StoredPreviousValueOfGyro[YAW] = IMU.GyroscopeRead[YAW];
    }
}