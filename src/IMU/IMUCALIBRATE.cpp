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
#include "Common/VARIABLES.h"
#include "PID/PIDPARAMS.h"
#include "LedRGB/LEDRGB.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Buzzer/BUZZER.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "BAR/BAR.h"
#include "IMUHEALTH.h"

int16_t StoredValueOfGyro[3] = {0, 0, 0};
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
  return sqrtf(DeviceVariance(Device));
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
      if ((GyroDeviation[ROLL] > 32) ||
          (GyroDeviation[PITCH] > 32) ||
          (GyroDeviation[YAW] > 32))
      {                                  //CHECA SE A IMU FOI MOVIDA DURANTE A CALIBRAÇÃO
        CalibratingGyroscope = 513;      //REINICIA A CALIBRAÇÃO
        BEEPER.Play(BEEPER_ACTION_FAIL); //SINALIZA COM O BUZZER QUE HOUVE UM ERRO
      }
      else
      {
        StoredGyroZero[ROLL] /= 512;
        StoredGyroZero[PITCH] /= 512;
        StoredGyroZero[YAW] /= 512;
        BEEPER.Play(BEEPER_ACTION_SUCCESS); //SINALIZA COM O BUZZER QUE TUDO OCORREU BEM
      }
      break;

    case 512:
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
      DevicePushValues(&GyroDevice[ROLL], IMU.GyroscopeRead[ROLL] * 0.25f);
      DevicePushValues(&GyroDevice[PITCH], IMU.GyroscopeRead[PITCH] * 0.25f);
      DevicePushValues(&GyroDevice[YAW], IMU.GyroscopeRead[YAW] * 0.25f);
      break;
    }
    CalibratingGyroscope--;
  }
  else
  {
    //ROLL
    IMU.GyroscopeRead[ROLL] = (IMU.GyroscopeRead[ROLL] - StoredGyroZero[ROLL]) * 0.25f;
    IMU.GyroscopeRead[ROLL] = Constrain_16Bits(IMU.GyroscopeRead[ROLL], StoredValueOfGyro[ROLL] - 800, StoredValueOfGyro[ROLL] + 800);
    StoredValueOfGyro[ROLL] = IMU.GyroscopeRead[ROLL];
    //PITCH
    IMU.GyroscopeRead[PITCH] = (IMU.GyroscopeRead[PITCH] - StoredGyroZero[PITCH]) * 0.25f;
    IMU.GyroscopeRead[PITCH] = Constrain_16Bits(IMU.GyroscopeRead[PITCH], StoredValueOfGyro[PITCH] - 800, StoredValueOfGyro[PITCH] + 800);
    StoredValueOfGyro[PITCH] = IMU.GyroscopeRead[PITCH];
    //YAW
    IMU.GyroscopeRead[YAW] = (IMU.GyroscopeRead[YAW] - StoredGyroZero[YAW]) * 0.25f;
    IMU.GyroscopeRead[YAW] = Constrain_16Bits(IMU.GyroscopeRead[YAW], StoredValueOfGyro[YAW] - 800, StoredValueOfGyro[YAW] + 800);
    StoredValueOfGyro[YAW] = IMU.GyroscopeRead[YAW];
  }
}

void Accelerometer_Calibration()
{
  int8_t GetPositionActualOfAcc = GetAxisInclinedToCalibration(IMU.AccelerometerRead);
  static uint8_t AxisToCalibration = YAW;
  static int16_t AccMinMaxValue[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  static int32_t AccReadVector[3] = {0, 0, 0};

  if ((GetPositionActualOfAcc == -1 && CalibratingAccelerometer > 0) ||
      (AccCalibratedPosition[GetPositionActualOfAcc] && CalibratingAccelerometer > 0))
  {
    CalibratingAccelerometer = 0;
    BEEPER.Play(BEEPER_ACTION_FAIL);
    return;
  }

  if (CalibratingAccelerometer > 0)
  {
    RGB.Function(ACCLED);
    for (uint8_t AccCalibIndex = 0; AccCalibIndex < 3; AccCalibIndex++)
    {
      if (CalibratingAccelerometer == 512)
      {
        int8_t AccCalibIndexTwo = (AccCalibIndex + 1) % 3;
        int8_t AccCalibIndexThree = (AccCalibIndex + 2) % 3;
        if (ABS_16BITS(IMU.AccelerometerRead[AccCalibIndex] -
                       CALIBRATION.AccelerometerZero[AccCalibIndex]) > ABS_16BITS(IMU.AccelerometerRead[AccCalibIndexTwo] -
                                                                                  CALIBRATION.AccelerometerZero[AccCalibIndexTwo]) &&
            ABS_16BITS(IMU.AccelerometerRead[AccCalibIndex] -
                       CALIBRATION.AccelerometerZero[AccCalibIndex]) > ABS_16BITS(IMU.AccelerometerRead[AccCalibIndexThree] -
                                                                                  CALIBRATION.AccelerometerZero[AccCalibIndexThree]))
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
      int8_t MeasuredLimit = AccReadVector[AxisToCalibration] > 0 ? 0 : 1;
      AccMinMaxValue[AxisToCalibration][MeasuredLimit] = AccReadVector[AxisToCalibration] >> 9;
      if (AccMinMaxValue[AxisToCalibration][0] > 0 && AccMinMaxValue[AxisToCalibration][1] < 0)
      {
        CALIBRATION.AccelerometerZero[AxisToCalibration] = (AccMinMaxValue[AxisToCalibration][0] + AccMinMaxValue[AxisToCalibration][1]) / 2;
        CALIBRATION.AccelerometerScale[AxisToCalibration] = ((int32_t)512) * 2048 / (AccMinMaxValue[AxisToCalibration][0] - AccMinMaxValue[AxisToCalibration][1]);
      }
      else if (AxisToCalibration == YAW && MeasuredLimit == 0)
      {
        CALIBRATION.AccelerometerZero[ROLL] = AccReadVector[ROLL] >> 9;
        CALIBRATION.AccelerometerZero[PITCH] = AccReadVector[PITCH] >> 9;
        CALIBRATION.AccelerometerZero[YAW] = AccReadVector[YAW] >> 9;
        CALIBRATION.AccelerometerZero[YAW] -= 512;
        CALIBRATION.AccelerometerScale[ROLL] = 0;
        CALIBRATION.AccelerometerScale[PITCH] = 0;
        CALIBRATION.AccelerometerScale[YAW] = 0;
      }
      AccCalibratedPosition[GetPositionActualOfAcc] = true;
      STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADDR, CALIBRATION.AccelerometerZero[ROLL]);
      STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADDR, CALIBRATION.AccelerometerZero[PITCH]);
      STORAGEMANAGER.Write_16Bits(ACC_YAW_ADDR, CALIBRATION.AccelerometerZero[YAW]);
      STORAGEMANAGER.Write_16Bits(ACC_ROLL_SCALE_ADDR, CALIBRATION.AccelerometerScale[ROLL]);
      STORAGEMANAGER.Write_16Bits(ACC_PITCH_SCALE_ADDR, CALIBRATION.AccelerometerScale[PITCH]);
      STORAGEMANAGER.Write_16Bits(ACC_YAW_SCALE_ADDR, CALIBRATION.AccelerometerScale[YAW]);
      CheckAndUpdateIMUCalibration();
      BEEPER.Play(BEEPER_CALIBRATION_DONE);
    }
    CalibratingAccelerometer--;
  }
  if (CALIBRATION.AccelerometerScale[ROLL] ||
      CALIBRATION.AccelerometerScale[PITCH] ||
      CALIBRATION.AccelerometerScale[YAW]) //APLICA ACELERAÇÃO ZERO
  {
    IMU.AccelerometerRead[ROLL] = (((int32_t)(IMU.AccelerometerRead[ROLL] -
                                              CALIBRATION.AccelerometerZero[ROLL])) *
                                   CALIBRATION.AccelerometerScale[ROLL]) >>
                                  10;
    IMU.AccelerometerRead[PITCH] = (((int32_t)(IMU.AccelerometerRead[PITCH] -
                                               CALIBRATION.AccelerometerZero[PITCH])) *
                                    CALIBRATION.AccelerometerScale[PITCH]) >>
                                   10;
    IMU.AccelerometerRead[YAW] = (((int32_t)(IMU.AccelerometerRead[YAW] -
                                             CALIBRATION.AccelerometerZero[YAW])) *
                                  CALIBRATION.AccelerometerScale[YAW]) >>
                                 10;
  }
  else
  {
    IMU.AccelerometerRead[ROLL] -= CALIBRATION.AccelerometerZero[ROLL];
    IMU.AccelerometerRead[PITCH] -= CALIBRATION.AccelerometerZero[PITCH];
    IMU.AccelerometerRead[YAW] -= CALIBRATION.AccelerometerZero[YAW];
  }
}