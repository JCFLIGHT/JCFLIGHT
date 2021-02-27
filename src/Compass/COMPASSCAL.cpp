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

#include "COMPASSCAL.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "LedRGB/LEDRGB.h"
#include "IMU/IMUHEALTH.h"
#include "Buzzer/BUZZER.h"
#include "Common/STRUCTS.h"

CompassCalClass COMPASSCAL;

//TEMPO MAXIMO DE CALIBRAÇÃO DO COMPASS
#define CALIBRATION_COUNT 60 //SEGUNDOS

void CompassCalClass::ApplyGain()
{
    //APLICA O GANHO CALCULADO
    IMU.CompassRead[ROLL] = IMU.CompassRead[ROLL] * COMPASS.MagnetometerGain[ROLL];
    IMU.CompassRead[PITCH] = IMU.CompassRead[PITCH] * COMPASS.MagnetometerGain[PITCH];
    IMU.CompassRead[YAW] = IMU.CompassRead[YAW] * COMPASS.MagnetometerGain[YAW];
}

void CompassCalClass::ApplyCalibration()
{
    //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
    if (!COMPASS.Calibrating)
    {
        IMU.CompassRead[ROLL] -= CALIBRATION.Magnetometer[ROLL];
        IMU.CompassRead[PITCH] -= CALIBRATION.Magnetometer[PITCH];
        IMU.CompassRead[YAW] -= CALIBRATION.Magnetometer[YAW];
    }
}
#include "FastSerial/FASTSERIAL.h"
#include "FastSerial/PRINTF.h"
void CompassCalClass::RunningCalibration()
{
    if (!COMPASS.Calibrating)
    {
        return;
    }
    else
    {
        COMPASS.CalibrationCount++;
        if (COMPASS.CalibrationCount < ((CALIBRATION_COUNT * 10) - 30))
        {
            RGB.Function(MAGLED);
            if (COMPASS.CalibrationCount == 1)
            {
                COMPASS.MagCalibrationMinVector[ROLL] = COMPASS.MagnetometerRead[ROLL];
                COMPASS.MagCalibrationMaxVector[ROLL] = COMPASS.MagnetometerRead[ROLL];
                COMPASS.MagCalibrationMinVector[PITCH] = COMPASS.MagnetometerRead[PITCH];
                COMPASS.MagCalibrationMaxVector[PITCH] = COMPASS.MagnetometerRead[PITCH];
                COMPASS.MagCalibrationMinVector[YAW] = COMPASS.MagnetometerRead[YAW];
                COMPASS.MagCalibrationMaxVector[YAW] = COMPASS.MagnetometerRead[YAW];
            }
            if (((int16_t)COMPASS.MagnetometerRead[ROLL]) < COMPASS.MagCalibrationMinVector[ROLL])
            {
                COMPASS.MagCalibrationMinVector[ROLL] = COMPASS.MagnetometerRead[ROLL];
            }
            if (((int16_t)COMPASS.MagnetometerRead[PITCH]) < COMPASS.MagCalibrationMinVector[PITCH])
            {
                COMPASS.MagCalibrationMinVector[PITCH] = COMPASS.MagnetometerRead[PITCH];
            }
            if (((int16_t)COMPASS.MagnetometerRead[YAW]) < COMPASS.MagCalibrationMinVector[YAW])
            {
                COMPASS.MagCalibrationMinVector[YAW] = COMPASS.MagnetometerRead[YAW];
            }
            if (((int16_t)COMPASS.MagnetometerRead[ROLL]) > COMPASS.MagCalibrationMaxVector[ROLL])
            {
                COMPASS.MagCalibrationMaxVector[ROLL] = COMPASS.MagnetometerRead[ROLL];
            }
            if (((int16_t)COMPASS.MagnetometerRead[PITCH]) > COMPASS.MagCalibrationMaxVector[PITCH])
            {
                COMPASS.MagCalibrationMaxVector[PITCH] = COMPASS.MagnetometerRead[PITCH];
            }
            if (((int16_t)COMPASS.MagnetometerRead[YAW]) > COMPASS.MagCalibrationMaxVector[YAW])
            {
                COMPASS.MagCalibrationMaxVector[YAW] = COMPASS.MagnetometerRead[YAW];
            }
            CALIBRATION.Magnetometer[ROLL] = (COMPASS.MagCalibrationMinVector[ROLL] + COMPASS.MagCalibrationMaxVector[ROLL]) >> 1;
            CALIBRATION.Magnetometer[PITCH] = (COMPASS.MagCalibrationMinVector[PITCH] + COMPASS.MagCalibrationMaxVector[PITCH]) >> 1;
            CALIBRATION.Magnetometer[YAW] = (COMPASS.MagCalibrationMinVector[YAW] + COMPASS.MagCalibrationMaxVector[YAW]) >> 1;
        }
        else
        {
            COMPASS.Calibrating = false;
            COMPASS.CalibrationCount = 0;
            STORAGEMANAGER.Write_16Bits(MAG_ROLL_ADDR, CALIBRATION.Magnetometer[ROLL]);
            STORAGEMANAGER.Write_16Bits(MAG_PITCH_ADDR, CALIBRATION.Magnetometer[PITCH]);
            STORAGEMANAGER.Write_16Bits(MAG_YAW_ADDR, CALIBRATION.Magnetometer[YAW]);
            UpdateCompassCalibration();
            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }
}

void CompassCalClass::UpdateCompassCalibration()
{
    CALIBRATION.Magnetometer[ROLL] = STORAGEMANAGER.Read_16Bits(MAG_ROLL_ADDR);
    CALIBRATION.Magnetometer[PITCH] = STORAGEMANAGER.Read_16Bits(MAG_PITCH_ADDR);
    CALIBRATION.Magnetometer[YAW] = STORAGEMANAGER.Read_16Bits(MAG_YAW_ADDR);
}