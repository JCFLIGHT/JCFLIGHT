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
#include "Scheduler/SCHEDULERTIME.h"
#include "GaussNewton/GAUSSNEWTON.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AccCalibClass ACCCALIBRATION;

Calibration_Struct Calibration;
static Jacobian_Struct Jacobian_Matrices_To_Acc;

#define CALIBRATING_ACC_TIME_MS 500 //TEMPO MAXIMO DE CALIBRAÇÃO DO ACC EM MS

void AccCalibClass::Start(void)
{
    Calibration.Accelerometer.Flags.InCalibration = true;
    Calibration.Accelerometer.Time.Previous = Calibration.Accelerometer.Time.Actual;
}

bool AccCalibClass::GetRunning(void)
{
    return Calibration.Accelerometer.Time.Previous > 0;
}

static bool GetAccAxisCalibrationRunning(void)
{
    return Calibration.Accelerometer.Flags.InCalibration;
}

static bool GetFirstAccelerationCalibrationCycle(void)
{
    return (Calibration.Accelerometer.Time.Difference > (CALIBRATING_ACC_TIME_MS * 0.999f)); //99.9% DO TEMPO
}

static bool GetFinalAccelerationCalibrationCycle(void)
{
    return (Calibration.Accelerometer.Time.Difference < (CALIBRATING_ACC_TIME_MS * 0.05f)); //5% DO TEMPO
}

static bool GetCheckOnlyLevelCalibration(void)
{
    return Calibration.Accelerometer.Flags.CalibratedPosition[0] && !Calibration.Accelerometer.Flags.CalibratedPosition[1] &&
           !Calibration.Accelerometer.Flags.CalibratedPosition[2] && !Calibration.Accelerometer.Flags.CalibratedPosition[3] &&
           !Calibration.Accelerometer.Flags.CalibratedPosition[4] && !Calibration.Accelerometer.Flags.CalibratedPosition[5];
}

static bool GetAllOrientationsHaveCalibrationDataCollected(void)
{
    return Calibration.Accelerometer.Flags.CalibratedPosition[0] && Calibration.Accelerometer.Flags.CalibratedPosition[1] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[2] && Calibration.Accelerometer.Flags.CalibratedPosition[3] &&
           Calibration.Accelerometer.Flags.CalibratedPosition[4] && Calibration.Accelerometer.Flags.CalibratedPosition[5];
}

static void PerformAccelerationCalibration(void)
{
    int8_t GetActualPositionOfAcc = GetAxisInclinedToCalibration(IMU.Accelerometer.Read);

    if ((GetActualPositionOfAcc == -1 && GetAccAxisCalibrationRunning()) ||
        (Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc] && GetAccAxisCalibrationRunning()))
    {
        Calibration.Accelerometer.Flags.InCalibration = false;
        Calibration.Accelerometer.Time.Previous = 0;
        BEEPER.Play(BEEPER_ACTION_FAIL);
        return;
    }

    RGB.Function(CALL_LED_ACC_CALIBRATION);

    Calibration.Accelerometer.Time.Difference = Calibration.Accelerometer.Time.Actual - Calibration.Accelerometer.Time.Previous;

    //RESETA TODOS OS PARAMETROS DA CALIBRAÇÃO E A MATRIX DE GAUSS NEWTON
    if (GetActualPositionOfAcc == 0 && GetFirstAccelerationCalibrationCycle())
    {
        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            Calibration.Accelerometer.Flags.CalibratedPosition[AxisIndex] = false;
            Calibration.Accelerometer.Samples.Window[AxisIndex][ROLL] = 0;
            Calibration.Accelerometer.Samples.Window[AxisIndex][PITCH] = 0;
            Calibration.Accelerometer.Samples.Window[AxisIndex][YAW] = 0;
        }
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);
    }

    if (!Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc])
    {
        GaussNewtonPushSampleForOffSetCalculation(&Jacobian_Matrices_To_Acc, IMU.Accelerometer.Read);
        Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][ROLL] += IMU.Accelerometer.Read[ROLL];
        Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][PITCH] += IMU.Accelerometer.Read[PITCH];
        Calibration.Accelerometer.Samples.Window[GetActualPositionOfAcc][YAW] += IMU.Accelerometer.Read[YAW];

        if (GetFinalAccelerationCalibrationCycle())
        {
            Calibration.Accelerometer.Flags.CalibratedPosition[GetActualPositionOfAcc] = true;
            Calibration.Accelerometer.Flags.InCalibration = false;
            BEEPER.Play(BEEPER_CALIBRATION_DONE);
        }
    }

    if (GetAllOrientationsHaveCalibrationDataCollected())
    {
        bool CalibrationFailed = false;
        float AccOffSetAndScaleBeta[3];
        int16_t AccSampleToScale[3];

        //CALCULA O OFFSET E VERIFICA SE A CALIBRAÇÃO FALHOU
        if (!GaussNewtonSolveForOffSet(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta))
        {
            AccOffSetAndScaleBeta[ROLL] = 0.0f;
            AccOffSetAndScaleBeta[PITCH] = 0.0f;
            AccOffSetAndScaleBeta[YAW] = 0.0f;
            CalibrationFailed = true;
        }

        Calibration.Accelerometer.OffSet[ROLL] = lrintf(AccOffSetAndScaleBeta[ROLL]);
        Calibration.Accelerometer.OffSet[PITCH] = lrintf(AccOffSetAndScaleBeta[PITCH]);
        Calibration.Accelerometer.OffSet[YAW] = lrintf(AccOffSetAndScaleBeta[YAW]);

        //LIMPA A MATRIX AFIM DE NÃO COMPENSAR AS AMOSTRAS MÉDIAS,ESCALAS E GANHOS
        ClearGaussNewtonMatrices(&Jacobian_Matrices_To_Acc);

        for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
        {
            AccSampleToScale[ROLL] = Calibration.Accelerometer.Samples.Window[AxisIndex][ROLL] / Calibration.Accelerometer.Samples.Counter - Calibration.Accelerometer.OffSet[ROLL];
            AccSampleToScale[PITCH] = Calibration.Accelerometer.Samples.Window[AxisIndex][PITCH] / Calibration.Accelerometer.Samples.Counter - Calibration.Accelerometer.OffSet[PITCH];
            AccSampleToScale[YAW] = Calibration.Accelerometer.Samples.Window[AxisIndex][YAW] / Calibration.Accelerometer.Samples.Counter - Calibration.Accelerometer.OffSet[YAW];

            GaussNewtonPushSampleForScaleCalculation(&Jacobian_Matrices_To_Acc, AxisIndex / 2, AccSampleToScale, IMU.Accelerometer.GravityForce.OneG);
        }

        //CALCULA A ESCALA E VERIFICA SE A CALIBRAÇÃO FALHOU
        if (!GaussNewtonSolveForScale(&Jacobian_Matrices_To_Acc, AccOffSetAndScaleBeta))
        {
            AccOffSetAndScaleBeta[ROLL] = 1.0f;
            AccOffSetAndScaleBeta[PITCH] = 1.0f;
            AccOffSetAndScaleBeta[YAW] = 1.0f;
            CalibrationFailed = true;
        }

        Calibration.Accelerometer.Scale[ROLL] = lrintf(AccOffSetAndScaleBeta[ROLL] * 4096);
        Calibration.Accelerometer.Scale[PITCH] = lrintf(AccOffSetAndScaleBeta[PITCH] * 4096);
        Calibration.Accelerometer.Scale[YAW] = lrintf(AccOffSetAndScaleBeta[YAW] * 4096);

        if (!CalibrationFailed) //A CALIBRAÇÃO FALHOU?NÃO...
        {
            SaveIMUCalibration();
        }
        else
        {
            for (uint8_t AxisIndex = 0; AxisIndex < 6; AxisIndex++)
            {
                Calibration.Accelerometer.Flags.CalibratedPosition[AxisIndex] = false;
            }
        }

        Calibration.Accelerometer.Samples.Counter = 0;
        Calibration.Accelerometer.Time.Previous = 0;
    }

    if (GetCheckOnlyLevelCalibration())
    {
        Calibration.Accelerometer.Samples.Counter++;
    }
}

void AccCalibClass::Update(void)
{
    Calibration.Accelerometer.Time.Actual = SCHEDULERTIME.GetMillis();

    if (GetAccAxisCalibrationRunning())
    {
        PerformAccelerationCalibration();
    }
    else
    {
        //APLICA A ACELERAÇÃO ZERO
        IMU.Accelerometer.Read[ROLL] = (((int32_t)(IMU.Accelerometer.Read[ROLL] - Calibration.Accelerometer.OffSet[ROLL])) * Calibration.Accelerometer.Scale[ROLL]) / 4096;
        IMU.Accelerometer.Read[PITCH] = (((int32_t)(IMU.Accelerometer.Read[PITCH] - Calibration.Accelerometer.OffSet[PITCH])) * Calibration.Accelerometer.Scale[PITCH]) / 4096;
        IMU.Accelerometer.Read[YAW] = (((int32_t)(IMU.Accelerometer.Read[YAW] - Calibration.Accelerometer.OffSet[YAW])) * Calibration.Accelerometer.Scale[YAW]) / 4096;
    }
}