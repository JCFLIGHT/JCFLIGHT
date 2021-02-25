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
    IMU.CompassRead[ROLL] = IMU.CompassRead[ROLL] * MagnetometerGain[ROLL];
    IMU.CompassRead[PITCH] = IMU.CompassRead[PITCH] * MagnetometerGain[PITCH];
    IMU.CompassRead[YAW] = IMU.CompassRead[YAW] * MagnetometerGain[YAW];
}

void CompassCalClass::ApplyCalibration()
{
    //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
    if (!Calibrating)
    {
        IMU.CompassRead[ROLL] -= CALIBRATION.Magnetometer[ROLL];
        IMU.CompassRead[PITCH] -= CALIBRATION.Magnetometer[PITCH];
        IMU.CompassRead[YAW] -= CALIBRATION.Magnetometer[YAW];
    }
}

void CompassCalClass::RunningCalibration()
{
    if (!Calibrating)
    {
        return;
    }
    else
    {
        CalibrationCount++;
        if (CalibrationCount < ((CALIBRATION_COUNT * 10) - 30))
        {
            RGB.Function(MAGLED);
            if (CalibrationCount == 1)
            {
                MagCalibrationMinVector[ROLL] = MagnetometerRead[ROLL];
                MagCalibrationMaxVector[ROLL] = MagnetometerRead[ROLL];
                MagCalibrationMinVector[PITCH] = MagnetometerRead[PITCH];
                MagCalibrationMaxVector[PITCH] = MagnetometerRead[PITCH];
                MagCalibrationMinVector[YAW] = MagnetometerRead[YAW];
                MagCalibrationMaxVector[YAW] = MagnetometerRead[YAW];
            }
            if (((int16_t)MagnetometerRead[ROLL]) < MagCalibrationMinVector[ROLL])
            {
                MagCalibrationMinVector[ROLL] = MagnetometerRead[ROLL];
            }
            if (((int16_t)MagnetometerRead[PITCH]) < MagCalibrationMinVector[PITCH])
            {
                MagCalibrationMinVector[PITCH] = MagnetometerRead[PITCH];
            }
            if (((int16_t)MagnetometerRead[YAW]) < MagCalibrationMinVector[YAW])
            {
                MagCalibrationMinVector[YAW] = MagnetometerRead[YAW];
            }
            if (((int16_t)MagnetometerRead[ROLL]) > MagCalibrationMaxVector[ROLL])
            {
                MagCalibrationMaxVector[ROLL] = MagnetometerRead[ROLL];
            }
            if (((int16_t)MagnetometerRead[PITCH]) > MagCalibrationMaxVector[PITCH])
            {
                MagCalibrationMaxVector[PITCH] = MagnetometerRead[PITCH];
            }
            if (((int16_t)MagnetometerRead[YAW]) > MagCalibrationMaxVector[YAW])
            {
                MagCalibrationMaxVector[YAW] = MagnetometerRead[YAW];
            }
            CALIBRATION.Magnetometer[ROLL] = (MagCalibrationMinVector[ROLL] + MagCalibrationMaxVector[ROLL]) >> 1;
            CALIBRATION.Magnetometer[PITCH] = (MagCalibrationMinVector[PITCH] + MagCalibrationMaxVector[PITCH]) >> 1;
            CALIBRATION.Magnetometer[YAW] = (MagCalibrationMinVector[YAW] + MagCalibrationMaxVector[YAW]) >> 1;
        }
        else
        {
            Calibrating = false;
            CalibrationCount = 0;
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