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
#include "LedRGB/LEDRGB.h"
#include "PERFORMACC.h"

#define GYRO_MAX_DELTA 32

void StartGyroCalibration(void)
{
    Calibration.Gyroscope.Counter = ACC_1G;
}

bool GyroCalibrationRunning(void)
{
    return Calibration.Gyroscope.Counter > 0;
}

void Gyroscope_Calibration(void)
{
    if (GyroCalibrationRunning())
    {
        static Device_Struct GyroDevice[3];

        RGB.Function(CALL_LED_GYRO_CALIBRATION);

        switch (Calibration.Gyroscope.Counter)
        {

        case 1:
            Calibration.Gyroscope.Deviation[ROLL] = DeviceStandardDeviation(&GyroDevice[ROLL]);
            Calibration.Gyroscope.Deviation[PITCH] = DeviceStandardDeviation(&GyroDevice[PITCH]);
            Calibration.Gyroscope.Deviation[YAW] = DeviceStandardDeviation(&GyroDevice[YAW]);
            //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
            if ((Calibration.Gyroscope.Deviation[ROLL] > GYRO_MAX_DELTA) ||
                (Calibration.Gyroscope.Deviation[PITCH] > GYRO_MAX_DELTA) ||
                (Calibration.Gyroscope.Deviation[YAW] > GYRO_MAX_DELTA))
            {
                Calibration.Gyroscope.Counter = ACC_1G + 1; //REINICIA A CALIBRAÇÃO
                BEEPER.Play(BEEPER_ACTION_FAIL);            //SINALIZA COM O BUZZER QUE HOUVE UM ERRO
            }
            else
            {
                Calibration.Gyroscope.Sum[ROLL] /= ACC_1G;
                Calibration.Gyroscope.Sum[PITCH] /= ACC_1G;
                Calibration.Gyroscope.Sum[YAW] /= ACC_1G;
                BEEPER.Play(BEEPER_CALIBRATION_DONE); //SINALIZA COM O BUZZER QUE TUDO OCORREU BEM
            }
            break;

        case ACC_1G:
            Calibration.Gyroscope.Sum[ROLL] = 0;
            Calibration.Gyroscope.Sum[PITCH] = 0;
            Calibration.Gyroscope.Sum[YAW] = 0;
            DeviceClear(&GyroDevice[ROLL]);
            DeviceClear(&GyroDevice[PITCH]);
            DeviceClear(&GyroDevice[YAW]);
            break;

        default:
            Calibration.Gyroscope.Sum[ROLL] += IMU.Gyroscope.Read[ROLL];
            Calibration.Gyroscope.Sum[PITCH] += IMU.Gyroscope.Read[PITCH];
            Calibration.Gyroscope.Sum[YAW] += IMU.Gyroscope.Read[YAW];
            DevicePushValues(&GyroDevice[ROLL], IMU.Gyroscope.Read[ROLL]);
            DevicePushValues(&GyroDevice[PITCH], IMU.Gyroscope.Read[PITCH]);
            DevicePushValues(&GyroDevice[YAW], IMU.Gyroscope.Read[YAW]);
            break;
        }
        Calibration.Gyroscope.Counter--;
    }
    else
    {
        //ROLL
        IMU.Gyroscope.Read[ROLL] = (IMU.Gyroscope.Read[ROLL] - Calibration.Gyroscope.Sum[ROLL]);
        IMU.Gyroscope.Read[ROLL] = Constrain_16Bits(IMU.Gyroscope.Read[ROLL], Calibration.Gyroscope.PreviousValue[ROLL] - 800, Calibration.Gyroscope.PreviousValue[ROLL] + 800);
        Calibration.Gyroscope.PreviousValue[ROLL] = IMU.Gyroscope.Read[ROLL];
        IMU.Gyroscope.Read[ROLL] = IMU.Gyroscope.Read[ROLL] * GYRO_SCALE;
        //PITCH
        IMU.Gyroscope.Read[PITCH] = (IMU.Gyroscope.Read[PITCH] - Calibration.Gyroscope.Sum[PITCH]);
        IMU.Gyroscope.Read[PITCH] = Constrain_16Bits(IMU.Gyroscope.Read[PITCH], Calibration.Gyroscope.PreviousValue[PITCH] - 800, Calibration.Gyroscope.PreviousValue[PITCH] + 800);
        Calibration.Gyroscope.PreviousValue[PITCH] = IMU.Gyroscope.Read[PITCH];
        IMU.Gyroscope.Read[PITCH] = IMU.Gyroscope.Read[PITCH] * GYRO_SCALE;
        //YAW
        IMU.Gyroscope.Read[YAW] = (IMU.Gyroscope.Read[YAW] - Calibration.Gyroscope.Sum[YAW]);
        IMU.Gyroscope.Read[YAW] = Constrain_16Bits(IMU.Gyroscope.Read[YAW], Calibration.Gyroscope.PreviousValue[YAW] - 800, Calibration.Gyroscope.PreviousValue[YAW] + 800);
        Calibration.Gyroscope.PreviousValue[YAW] = IMU.Gyroscope.Read[YAW];
        IMU.Gyroscope.Read[YAW] = IMU.Gyroscope.Read[YAW] * GYRO_SCALE;
    }
}