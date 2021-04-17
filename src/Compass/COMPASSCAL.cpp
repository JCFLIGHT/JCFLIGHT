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
#include "PerformanceCalibration/PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "Param/PARAM.h"

CompassCalClass COMPASSCAL;

#ifdef __AVR_ATmega2560__
//TEMPO MAXIMO DE CALIBRAÇÃO DO COMPASS
#define CALIBRATION_COUNT 60 //SEGUNDOS
#else
#define CALIBRATION_COUNT JCF_Param.Compass_Cal_Timer
#endif

void CompassCalClass::ApplyGain()
{
    //APLICA O GANHO CALCULADO
    IMU.Compass.Read[ROLL] = IMU.Compass.Read[ROLL] * Calibration.Magnetometer.Gain[ROLL];
    IMU.Compass.Read[PITCH] = IMU.Compass.Read[PITCH] * Calibration.Magnetometer.Gain[PITCH];
    IMU.Compass.Read[YAW] = IMU.Compass.Read[YAW] * Calibration.Magnetometer.Gain[YAW];
}

void CompassCalClass::ApplyCalibration()
{
    //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
    if (!IMU.Compass.Calibrating)
    {
        IMU.Compass.Read[ROLL] -= Calibration.Magnetometer.OffSet[ROLL];
        IMU.Compass.Read[PITCH] -= Calibration.Magnetometer.OffSet[PITCH];
        IMU.Compass.Read[YAW] -= Calibration.Magnetometer.OffSet[YAW];
    }
}

void CompassCalClass::RunningCalibration()
{
    if (!IMU.Compass.Calibrating)
    {
        return;
    }
    else
    {
        Calibration.Magnetometer.Count++;
        if (Calibration.Magnetometer.Count < (CALIBRATION_COUNT * 10))
        {
            RGB.Function(CALL_LED_MAG_CALIBRATION);
            if (Calibration.Magnetometer.Count == 1)
            {
                Calibration.Magnetometer.MinOffSet[ROLL] = IMU.Compass.ReadSmooth[ROLL];
                Calibration.Magnetometer.MaxOffSet[ROLL] = IMU.Compass.ReadSmooth[ROLL];
                Calibration.Magnetometer.MinOffSet[PITCH] = IMU.Compass.ReadSmooth[PITCH];
                Calibration.Magnetometer.MaxOffSet[PITCH] = IMU.Compass.ReadSmooth[PITCH];
                Calibration.Magnetometer.MinOffSet[YAW] = IMU.Compass.ReadSmooth[YAW];
                Calibration.Magnetometer.MaxOffSet[YAW] = IMU.Compass.ReadSmooth[YAW];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[ROLL]) < Calibration.Magnetometer.MinOffSet[ROLL])
            {
                Calibration.Magnetometer.MinOffSet[ROLL] = IMU.Compass.ReadSmooth[ROLL];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[PITCH]) < Calibration.Magnetometer.MinOffSet[PITCH])
            {
                Calibration.Magnetometer.MinOffSet[PITCH] = IMU.Compass.ReadSmooth[PITCH];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[YAW]) < Calibration.Magnetometer.MinOffSet[YAW])
            {
                Calibration.Magnetometer.MinOffSet[YAW] = IMU.Compass.ReadSmooth[YAW];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[ROLL]) > Calibration.Magnetometer.MaxOffSet[ROLL])
            {
                Calibration.Magnetometer.MaxOffSet[ROLL] = IMU.Compass.ReadSmooth[ROLL];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[PITCH]) > Calibration.Magnetometer.MaxOffSet[PITCH])
            {
                Calibration.Magnetometer.MaxOffSet[PITCH] = IMU.Compass.ReadSmooth[PITCH];
            }
            if (((int16_t)IMU.Compass.ReadSmooth[YAW]) > Calibration.Magnetometer.MaxOffSet[YAW])
            {
                Calibration.Magnetometer.MaxOffSet[YAW] = IMU.Compass.ReadSmooth[YAW];
            }
            Calibration.Magnetometer.OffSet[ROLL] = (Calibration.Magnetometer.MinOffSet[ROLL] + Calibration.Magnetometer.MaxOffSet[ROLL]) >> 1;
            Calibration.Magnetometer.OffSet[PITCH] = (Calibration.Magnetometer.MinOffSet[PITCH] + Calibration.Magnetometer.MaxOffSet[PITCH]) >> 1;
            Calibration.Magnetometer.OffSet[YAW] = (Calibration.Magnetometer.MinOffSet[YAW] + Calibration.Magnetometer.MaxOffSet[YAW]) >> 1;
        }
        else
        {
            IMU.Compass.Calibrating = false;
            Calibration.Magnetometer.Count = 0;
            STORAGEMANAGER.Write_16Bits(MAG_ROLL_OFFSET_ADDR, Calibration.Magnetometer.OffSet[ROLL]);
            STORAGEMANAGER.Write_16Bits(MAG_PITCH_OFFSET_ADDR, Calibration.Magnetometer.OffSet[PITCH]);
            STORAGEMANAGER.Write_16Bits(MAG_YAW_OFFSET_ADDR, Calibration.Magnetometer.OffSet[YAW]);
            UpdateCompassCalibration();
            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }
}

void CompassCalClass::UpdateCompassCalibration()
{
    Calibration.Magnetometer.OffSet[ROLL] = STORAGEMANAGER.Read_16Bits(MAG_ROLL_OFFSET_ADDR);
    Calibration.Magnetometer.OffSet[PITCH] = STORAGEMANAGER.Read_16Bits(MAG_PITCH_OFFSET_ADDR);
    Calibration.Magnetometer.OffSet[YAW] = STORAGEMANAGER.Read_16Bits(MAG_YAW_OFFSET_ADDR);
}