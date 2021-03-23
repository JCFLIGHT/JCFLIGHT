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
#include "LedRGB/LEDRGB.h"
#include "IMU/IMUHEALTH.h"
#include "GaussNewton/GAUSSNEWTON.h"

Calibration_Struct CALIBRATION;
static Jacobian_Struct Jacobian_Matrices_To_Acc;

void StartAccCalibration(void)
{
    CALIBRATION.Accelerometer.Counter = CALIBRATING_ACC_CYCLES;
}

bool AccCalibrationRunning(void)
{
    return CALIBRATION.Accelerometer.Counter > 0;
}

bool FirstAccelerationCalibrationCycle(void)
{
    return CALIBRATION.Accelerometer.Counter == CALIBRATING_ACC_CYCLES;
}

bool FinalAccelerationCalibrationCycle(void)
{
    return CALIBRATION.Accelerometer.Counter == 1;
}

void PerformAccelerationCalibration(void)
{
    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.AccelerometerRead);

    if ((GetActualPositionOfAcc == -1 && AccCalibrationRunning()) ||
        (AccCalibratedPosition[GetActualPositionOfAcc] && AccCalibrationRunning()))
    {
        CALIBRATION.Accelerometer.Counter = 0;
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
            CALIBRATION.Accelerometer.Samples[AxisIndex][ROLL] = 0;
            CALIBRATION.Accelerometer.Samples[AxisIndex][PITCH] = 0;
            CALIBRATION.Accelerometer.Samples[AxisIndex][YAW] = 0;
        }

        CALIBRATION.Accelerometer.PositionCount = 0;
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);
    }

    if (!AccCalibratedPosition[GetActualPositionOfAcc])
    {
        GaussNewtonPushSampleForOffSetCalculation(&Jacobian_Matrices_To_Acc, IMU.AccelerometerRead);
        CALIBRATION.Accelerometer.Samples[GetActualPositionOfAcc][ROLL] += IMU.AccelerometerRead[ROLL];
        CALIBRATION.Accelerometer.Samples[GetActualPositionOfAcc][PITCH] += IMU.AccelerometerRead[PITCH];
        CALIBRATION.Accelerometer.Samples[GetActualPositionOfAcc][YAW] += IMU.AccelerometerRead[YAW];

        if (FinalAccelerationCalibrationCycle())
        {
            AccCalibratedPosition[GetActualPositionOfAcc] = true;
            CALIBRATION.Accelerometer.PositionCount++;

            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }

    if (CALIBRATION.Accelerometer.PositionCount == 6)
    {
        float AccOffSetAndScaleBeta[3];
        int16_t AccSampleToScale[3];

        //CALCULA O OFFSET
        GaussNewtonSolveForOffSet(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta);

        for (uint8_t AxisIndex = 0; AxisIndex < 3; AxisIndex++)
        {
            CALIBRATION.Accelerometer.OffSet[AxisIndex] = lrintf(AccOffSetAndScaleBeta[AxisIndex]);
        }

        //LIMPA A MATRIX AFIM DE NÃO COMPENSAR AS AMOSTRAS MÉDIAS,ESCALAS E GANHOS
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);

        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            AccSampleToScale[ROLL] = CALIBRATION.Accelerometer.Samples[AxisIndex][ROLL] / CALIBRATING_ACC_CYCLES - CALIBRATION.Accelerometer.OffSet[ROLL];
            AccSampleToScale[PITCH] = CALIBRATION.Accelerometer.Samples[AxisIndex][PITCH] / CALIBRATING_ACC_CYCLES - CALIBRATION.Accelerometer.OffSet[PITCH];
            AccSampleToScale[YAW] = CALIBRATION.Accelerometer.Samples[AxisIndex][YAW] / CALIBRATING_ACC_CYCLES - CALIBRATION.Accelerometer.OffSet[YAW];

            GaussNewtonPushSampleForScaleCalculation(&Jacobian_Matrices_To_Acc, AxisIndex / 2, AccSampleToScale, 256);
        }

        //CALCULA A ESCALA
        GaussNewtonSolveForScale(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta);

        for (uint8_t AxisIndex = 0; AxisIndex < 3; AxisIndex++)
        {
            CALIBRATION.Accelerometer.Scale[AxisIndex] = lrintf(AccOffSetAndScaleBeta[AxisIndex] * 2048);
        }

        SaveIMUCalibration();
    }

    CALIBRATION.Accelerometer.Counter--;
}

void Accelerometer_Calibration()
{
    if (AccCalibrationRunning())
    {
        PerformAccelerationCalibration();
    }

    //APLICA A ACELERAÇÃO ZERO
    IMU.AccelerometerRead[ROLL] = (((int32_t)(IMU.AccelerometerRead[ROLL] - CALIBRATION.Accelerometer.OffSet[ROLL])) * CALIBRATION.Accelerometer.Scale[ROLL]) / 1024;
    IMU.AccelerometerRead[PITCH] = (((int32_t)(IMU.AccelerometerRead[PITCH] - CALIBRATION.Accelerometer.OffSet[PITCH])) * CALIBRATION.Accelerometer.Scale[PITCH]) / 1024;
    IMU.AccelerometerRead[YAW] = (((int32_t)(IMU.AccelerometerRead[YAW] - CALIBRATION.Accelerometer.OffSet[YAW])) * CALIBRATION.Accelerometer.Scale[YAW]) / 1024;
}