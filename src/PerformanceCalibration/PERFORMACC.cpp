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

#include "PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "Common/STRUCTS.h"
#include "Buzzer/BUZZER.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "LedRGB/LEDRGB.h"
#include "Math/MATHSUPPORT.h"
#include "IMU/IMUHEALTH.h"
#include "Scheduler/SCHEDULER.h"

int16_t CalibratingAccelerometer;

void StartAccCalibration()
{
    CalibratingAccelerometer = ACC_1G;
}

bool AccCalibrationRunning()
{
    return CalibratingAccelerometer > 0;
}

void Accelerometer_Calibration()
{
    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.AccelerometerRead);
    static uint8_t AxisToCalibration = YAW;
    static int16_t AccMinMaxValue[3][2] = {{0, 0}, {0, 0}, {0, 0}};
    static int32_t AccReadVector[3] = {0, 0, 0};

    if ((GetActualPositionOfAcc == -1 && AccCalibrationRunning()) ||
        (AccCalibratedPosition[GetActualPositionOfAcc] && AccCalibrationRunning()))
    {
        CalibratingAccelerometer = 0;
        BEEPER.Play(BEEPER_ACTION_FAIL);
        return;
    }

#ifndef __AVR_ATmega2560__
    static Scheduler_Struct ACC_Calibration_Timer;
    if (Scheduler(&ACC_Calibration_Timer, SCHEDULER_SET_FREQUENCY(100, "Hz")))
    {
#endif

        if (AccCalibrationRunning())
        {
            RGB.Function(CALL_LED_ACC_CALIBRATION);
            for (uint8_t AccCalibIndex = 0; AccCalibIndex < 3; AccCalibIndex++)
            {
                if (CalibratingAccelerometer == ACC_1G)
                {
                    uint8_t AccCalibIndexTwo = (AccCalibIndex + 1) % 3;
                    uint8_t AccCalibIndexThree = (AccCalibIndex + 2) % 3;
                    if (ABS(IMU.AccelerometerRead[AccCalibIndex] - CALIBRATION.AccelerometerZero[AccCalibIndex]) > ABS(IMU.AccelerometerRead[AccCalibIndexTwo] - CALIBRATION.AccelerometerZero[AccCalibIndexTwo]) &&
                        ABS(IMU.AccelerometerRead[AccCalibIndex] - CALIBRATION.AccelerometerZero[AccCalibIndex]) > ABS(IMU.AccelerometerRead[AccCalibIndexThree] - CALIBRATION.AccelerometerZero[AccCalibIndexThree]))
                    {
                        AxisToCalibration = AccCalibIndex;
                        CALIBRATION.AccelerometerZero[AxisToCalibration] = 0;
                        CALIBRATION.AccelerometerScale[AxisToCalibration] = 0;
                    }
                    AccReadVector[AccCalibIndex] = 0;
                }
                AccReadVector[AccCalibIndex] += IMU.AccelerometerRead[AccCalibIndex];
            }
            IMU.AccelerometerRead[AxisToCalibration] = 0;
            if (CalibratingAccelerometer == 1)
            {
                uint8_t MeasuredLimit = AccReadVector[AxisToCalibration] > 0 ? 0 : 1;
                AccMinMaxValue[AxisToCalibration][MeasuredLimit] = AccReadVector[AxisToCalibration] / ACC_1G;
                if (AccMinMaxValue[AxisToCalibration][0] > 0 && AccMinMaxValue[AxisToCalibration][1] < 0)
                {
                    CALIBRATION.AccelerometerZero[AxisToCalibration] = (AccMinMaxValue[AxisToCalibration][0] + AccMinMaxValue[AxisToCalibration][1]) / 2;
                    CALIBRATION.AccelerometerScale[AxisToCalibration] = ((int32_t)ACC_1G * 2048) / (AccMinMaxValue[AxisToCalibration][0] - AccMinMaxValue[AxisToCalibration][1]);
                }
                else if (AxisToCalibration == YAW && MeasuredLimit == NONE)
                {
                    CALIBRATION.AccelerometerZero[ROLL] = AccReadVector[ROLL] / ACC_1G;
                    CALIBRATION.AccelerometerZero[PITCH] = AccReadVector[PITCH] / ACC_1G;
                    CALIBRATION.AccelerometerZero[YAW] = AccReadVector[YAW] / ACC_1G;
                    CALIBRATION.AccelerometerZero[YAW] -= ACC_1G;
                    CALIBRATION.AccelerometerScale[ROLL] = 0;
                    CALIBRATION.AccelerometerScale[PITCH] = 0;
                    CALIBRATION.AccelerometerScale[YAW] = 0;
                }
                AccCalibratedPosition[GetActualPositionOfAcc] = true;
                //PONTO A SER MARCADO COMO ZERO
                STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADDR, CALIBRATION.AccelerometerZero[ROLL]);
                STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADDR, CALIBRATION.AccelerometerZero[PITCH]);
                STORAGEMANAGER.Write_16Bits(ACC_YAW_ADDR, CALIBRATION.AccelerometerZero[YAW]);
                //ESCALA
                STORAGEMANAGER.Write_16Bits(ACC_ROLL_SCALE_ADDR, CALIBRATION.AccelerometerScale[ROLL]);
                STORAGEMANAGER.Write_16Bits(ACC_PITCH_SCALE_ADDR, CALIBRATION.AccelerometerScale[PITCH]);
                STORAGEMANAGER.Write_16Bits(ACC_YAW_SCALE_ADDR, CALIBRATION.AccelerometerScale[YAW]);
                //ATUALIZA PARA OS NOVOS VALORES
                CheckAndUpdateIMUCalibration();
                BEEPER.Play(BEEPER_CALIBRATION_DONE);
            }
            CalibratingAccelerometer--;
        }

#ifndef __AVR_ATmega2560__
    }
#endif

    //APLICA A ACELERAÇÃO ZERO
    if (CALIBRATION.AccelerometerScale[ROLL] || CALIBRATION.AccelerometerScale[PITCH] || CALIBRATION.AccelerometerScale[YAW])
    {
        IMU.AccelerometerRead[ROLL] = (((int32_t)(IMU.AccelerometerRead[ROLL] - CALIBRATION.AccelerometerZero[ROLL])) * CALIBRATION.AccelerometerScale[ROLL]) / 1024;
        IMU.AccelerometerRead[PITCH] = (((int32_t)(IMU.AccelerometerRead[PITCH] - CALIBRATION.AccelerometerZero[PITCH])) * CALIBRATION.AccelerometerScale[PITCH]) / 1024;
        IMU.AccelerometerRead[YAW] = (((int32_t)(IMU.AccelerometerRead[YAW] - CALIBRATION.AccelerometerZero[YAW])) * CALIBRATION.AccelerometerScale[YAW]) / 1024;
    }
    else
    {
        IMU.AccelerometerRead[ROLL] -= CALIBRATION.AccelerometerZero[ROLL];
        IMU.AccelerometerRead[PITCH] -= CALIBRATION.AccelerometerZero[PITCH];
        IMU.AccelerometerRead[YAW] -= CALIBRATION.AccelerometerZero[YAW];
    }
}