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
#include "Buzzer/BUZZER.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "LedRGB/LEDRGB.h"
#include "Math/MATHSUPPORT.h"
#include "IMU/IMUHEALTH.h"
#include "Scheduler/SCHEDULER.h"
#include "GaussNewton/GAUSSNEWTON.h"

static GaussNewtonMatrices_Struct CalibrationState;

#define CALIBRATING_ACC_CYCLES 400

int16_t CalibratingAccelerometer;
static int16_t AccCalibratedPositionCount = 0;
static int32_t AccSamples[6][3];

void StartAccCalibration(void)
{
    CalibratingAccelerometer = CALIBRATING_ACC_CYCLES;
}

bool AccCalibrationRunning(void)
{
    return CalibratingAccelerometer > 0;
}

bool FirstAccelerationCalibrationCycle(void)
{
    return CalibratingAccelerometer == CALIBRATING_ACC_CYCLES;
}

bool FinalAccelerationCalibrationCycle(void)
{
    return CalibratingAccelerometer == 1;
}

void PerformAccelerationCalibration(void)
{
    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.AccelerometerRead);

    if ((GetActualPositionOfAcc == -1 && AccCalibrationRunning()) ||
        (AccCalibratedPosition[GetActualPositionOfAcc] && AccCalibrationRunning()))
    {
        CalibratingAccelerometer = 0;
        BEEPER.Play(BEEPER_ACTION_FAIL);
        return;
    }

    RGB.Function(CALL_LED_ACC_CALIBRATION);

    //RESETA TODOS OS PARAMETROS DA CALIBRAÇÃO E A MATRIX DE GAUSS NEWTON
    if (GetActualPositionOfAcc == 0 && FirstAccelerationCalibrationCycle())
    {
        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            AccCalibratedPosition[AxisIndex] = false;
            AccSamples[AxisIndex][ROLL] = 0;
            AccSamples[AxisIndex][PITCH] = 0;
            AccSamples[AxisIndex][YAW] = 0;
        }

        AccCalibratedPositionCount = 0;
        ClearGaussNewtonMatrices(&CalibrationState);
    }

    if (!AccCalibratedPosition[GetActualPositionOfAcc])
    {
        GaussNewtonPushSampleForOffSetCalculation(&CalibrationState, IMU.AccelerometerRead);
        AccSamples[GetActualPositionOfAcc][ROLL] += IMU.AccelerometerRead[ROLL];
        AccSamples[GetActualPositionOfAcc][PITCH] += IMU.AccelerometerRead[PITCH];
        AccSamples[GetActualPositionOfAcc][YAW] += IMU.AccelerometerRead[YAW];

        if (FinalAccelerationCalibrationCycle())
        {
            AccCalibratedPosition[GetActualPositionOfAcc] = true;
            AccCalibratedPositionCount++;

            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }

    if (AccCalibratedPositionCount == 6)
    {
        float AccOffSetAndScaleBeta[3];
        int16_t AccSampleToScale[3];

        //CALCULA O OFFSET
        GaussNewtonSolveForOffSet(&CalibrationState, AccOffSetAndScaleBeta);

        for (uint8_t AxisIndex = 0; AxisIndex < 3; AxisIndex++)
        {
            CALIBRATION.AccelerometerZero[AxisIndex] = lrintf(AccOffSetAndScaleBeta[AxisIndex]);
        }

        //LIMPA A MATRIX AFIM DE NÃO COMPENSAR AS AMOSTRAS MÉDIAS,ESCALAS E GANHOS
        ClearGaussNewtonMatrices(&CalibrationState);

        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            AccSampleToScale[ROLL] = AccSamples[AxisIndex][ROLL] / CALIBRATING_ACC_CYCLES - CALIBRATION.AccelerometerZero[ROLL];
            AccSampleToScale[PITCH] = AccSamples[AxisIndex][PITCH] / CALIBRATING_ACC_CYCLES - CALIBRATION.AccelerometerZero[PITCH];
            AccSampleToScale[YAW] = AccSamples[AxisIndex][YAW] / CALIBRATING_ACC_CYCLES - CALIBRATION.AccelerometerZero[YAW];

            GaussNewtonPushSampleForScaleCalculation(&CalibrationState, AxisIndex / 2, AccSampleToScale, 256);
        }

        //CALCULA A ESCALA
        GaussNewtonSolveForScale(&CalibrationState, AccOffSetAndScaleBeta);

        for (uint8_t AxisIndex = 0; AxisIndex < 3; AxisIndex++)
        {
            CALIBRATION.AccelerometerScale[AxisIndex] = lrintf(AccOffSetAndScaleBeta[AxisIndex] * 2048);
        }

        SaveIMUCalibration();
    }

    CalibratingAccelerometer--;
}

void Accelerometer_Calibration()
{
    if (AccCalibrationRunning())
    {
        PerformAccelerationCalibration();
    }

    //APLICA A ACELERAÇÃO ZERO
    IMU.AccelerometerRead[ROLL] = (((int32_t)(IMU.AccelerometerRead[ROLL] - CALIBRATION.AccelerometerZero[ROLL])) * CALIBRATION.AccelerometerScale[ROLL]) / 1024;
    IMU.AccelerometerRead[PITCH] = (((int32_t)(IMU.AccelerometerRead[PITCH] - CALIBRATION.AccelerometerZero[PITCH])) * CALIBRATION.AccelerometerScale[PITCH]) / 1024;
    IMU.AccelerometerRead[YAW] = (((int32_t)(IMU.AccelerometerRead[YAW] - CALIBRATION.AccelerometerZero[YAW])) * CALIBRATION.AccelerometerScale[YAW]) / 1024;
}