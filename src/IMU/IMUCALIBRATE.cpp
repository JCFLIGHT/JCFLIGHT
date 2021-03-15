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

#include "IMUCALIBRATE.h"
#include "PID/PIDPARAMS.h"
#include "LedRGB/LEDRGB.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Buzzer/BUZZER.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "BAR/BAR.h"
#include "IMUHEALTH.h"
#include "Scheduler/SCHEDULER.h"
#include "ACCGYROREAD.h"

int16_t StoredValueOfGyro[3] = {0, 0, 0};

uint16_t CalibratingAccelerometer;
uint16_t CalibratingGyroscope = ACC_1G;

int32_t StoredGyroZero[3] = {0, 0, 0};

static void DeviceClear(Device_Struct *Device)
{
  Device->MeasureCount = 0;
}

static void DevicePushValues(Device_Struct *Device, float Value)
{
  Device->MeasureCount++;
  if (Device->MeasureCount == 1)
  {
    Device->OldMeasure = Device->NewMeasure = Value;
    Device->OldValue = 0.0f;
  }
  else
  {
    Device->NewMeasure = Device->OldMeasure + (Value - Device->OldMeasure) / Device->MeasureCount;
    Device->NewValue = Device->OldValue + (Value - Device->OldMeasure) * (Value - Device->NewMeasure);
    Device->OldMeasure = Device->NewMeasure;
    Device->OldValue = Device->NewValue;
  }
}

static float DeviceVariance(Device_Struct *Device)
{
  return ((Device->MeasureCount > 1) ? Device->NewValue / (Device->MeasureCount - 1) : 0.0f);
}

static float DeviceStandardDeviation(Device_Struct *Device)
{
  return Fast_SquareRoot(DeviceVariance(Device));
}

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
      {                                    //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
        CalibratingGyroscope = ACC_1G + 1; //REINICIA A CALIBRAÇÃO
        BEEPER.Play(BEEPER_ACTION_FAIL);   //SINALIZA COM O BUZZER QUE HOUVE UM ERRO
      }
      else
      {
        StoredGyroZero[ROLL] /= ACC_1G;
        StoredGyroZero[PITCH] /= ACC_1G;
        StoredGyroZero[YAW] /= ACC_1G;
        BEEPER.Play(BEEPER_CALIBRATION_DONE); //SINALIZA COM O BUZZER QUE TUDO OCORREU BEM
      }
      break;

    case ACC_1G:
      StoredGyroZero[ROLL] = 0;
      StoredGyroZero[PITCH] = 0;
      StoredGyroZero[YAW] = 0;
      DeviceClear(&GyroDevice[ROLL]);
      DeviceClear(&GyroDevice[PITCH]);
      DeviceClear(&GyroDevice[YAW]);
      break;

    default:
      StoredGyroZero[ROLL] += IMU.GyroscopeRead[ROLL];
      StoredGyroZero[PITCH] += IMU.GyroscopeRead[PITCH];
      StoredGyroZero[YAW] += IMU.GyroscopeRead[YAW];
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
    IMU.GyroscopeRead[ROLL] = (IMU.GyroscopeRead[ROLL] - StoredGyroZero[ROLL]) * GYRO_SCALE;
    IMU.GyroscopeRead[ROLL] = Constrain_16Bits(IMU.GyroscopeRead[ROLL], StoredValueOfGyro[ROLL] - 800, StoredValueOfGyro[ROLL] + 800);
    StoredValueOfGyro[ROLL] = IMU.GyroscopeRead[ROLL];
    //PITCH
    IMU.GyroscopeRead[PITCH] = (IMU.GyroscopeRead[PITCH] - StoredGyroZero[PITCH]) * GYRO_SCALE;
    IMU.GyroscopeRead[PITCH] = Constrain_16Bits(IMU.GyroscopeRead[PITCH], StoredValueOfGyro[PITCH] - 800, StoredValueOfGyro[PITCH] + 800);
    StoredValueOfGyro[PITCH] = IMU.GyroscopeRead[PITCH];
    //YAW
    IMU.GyroscopeRead[YAW] = (IMU.GyroscopeRead[YAW] - StoredGyroZero[YAW]) * GYRO_SCALE;
    IMU.GyroscopeRead[YAW] = Constrain_16Bits(IMU.GyroscopeRead[YAW], StoredValueOfGyro[YAW] - 800, StoredValueOfGyro[YAW] + 800);
    StoredValueOfGyro[YAW] = IMU.GyroscopeRead[YAW];
  }
}

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
  if (Scheduler(&ACC_Calibration_Timer, SCHEDULER_SET_FREQUENCY(200, "Hz")))
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