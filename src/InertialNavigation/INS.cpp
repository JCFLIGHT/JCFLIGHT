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

#include "INS.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATION.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Scheduler/SCHEDULER.h"
#include "Barometer/BAROREAD.h"
#include "Math/MATHSUPPORT.h"
#include "GPS/GPSSTATES.h"
#include "AHRS/AHRS.h"
#include "IMU/ACCGYROREAD.h"
#include "Barometer/BAROBACKEND.h"

InertialNavigationClass INERTIALNAVIGATION;
INS_Struct INS;

void InertialNavigationClass::Calculate_AccelerationXYZ_To_EarthFrame()
{
  float Cosine_Roll;
  float Sine_Roll;
  float Cosine_Pitch;
  float Sine_Pitch;
  float SinePitch_CosineYaw_Fusion;
  float SinePitch_SineYaw_Fusion;

  //OBTÉM O SENO E COSSENO DE CADA EIXO DA ESTIMAÇÃO DE ATTITUDE DADO PELO AHRS
  Cosine_Roll = AHRS.CosineRoll();
  Sine_Roll = AHRS.SineRoll();
  Cosine_Pitch = -AHRS.CosinePitch();
  Sine_Pitch = -AHRS.SinePitch();
  INERTIALNAVIGATION.Cosine_Yaw = AHRS.CosineYaw();
  INERTIALNAVIGATION.Sine_Yaw = AHRS.SineYaw();

  //REALIZA A FUSÃO DE ALGUMAS VARIAVEIS PARA OBTER DIFERENTES VALORES DE ACELERAÇÃO EM TORNO DO EIXO Z
  SinePitch_CosineYaw_Fusion = Sine_Pitch * INERTIALNAVIGATION.Cosine_Yaw;
  SinePitch_SineYaw_Fusion = Sine_Pitch * INERTIALNAVIGATION.Sine_Yaw;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = -((Cosine_Pitch * INERTIALNAVIGATION.Cosine_Yaw) * IMU.Accelerometer.Read[PITCH] + (Sine_Roll * SinePitch_CosineYaw_Fusion - Cosine_Roll * Sine_Yaw) * IMU.Accelerometer.Read[ROLL] + (Sine_Roll * INERTIALNAVIGATION.Sine_Yaw + Cosine_Roll * SinePitch_CosineYaw_Fusion) * IMU.Accelerometer.Read[YAW]);

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = -((Cosine_Pitch * INERTIALNAVIGATION.Sine_Yaw) * IMU.Accelerometer.Read[PITCH] + (Cosine_Roll * INERTIALNAVIGATION.Cosine_Yaw + Sine_Roll * SinePitch_SineYaw_Fusion) * IMU.Accelerometer.Read[ROLL] + (-Sine_Roll * INERTIALNAVIGATION.Cosine_Yaw + Cosine_Roll * SinePitch_SineYaw_Fusion) * IMU.Accelerometer.Read[YAW]);

  //YAW
  INS.AccelerationEarthFrame[YAW] = ((-Sine_Pitch) * IMU.Accelerometer.Read[PITCH] + (Sine_Roll * Cosine_Pitch) * IMU.Accelerometer.Read[ROLL] + (Cosine_Roll * Cosine_Pitch) * IMU.Accelerometer.Read[YAW]) - 512;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[ROLL]);
  INERTIALNAVIGATION.AccelerationDifference[ROLL] = INS.AccelerationEarthFrame[ROLL] - INERTIALNAVIGATION.AccelerationAdjustBias[ROLL];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[ROLL] = INERTIALNAVIGATION.AccelerationAdjustBias[ROLL] * 0.985f + INS.AccelerationEarthFrame[ROLL] * 0.015f; //2HZ LPF
  }
  else if (ABS(INERTIALNAVIGATION.AccelerationDifference[ROLL]) <= 80.0f)
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[ROLL] = INERTIALNAVIGATION.AccelerationAdjustBias[ROLL] * 0.9987f + INS.AccelerationEarthFrame[ROLL] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[ROLL] = INERTIALNAVIGATION.AccelerationDifference[ROLL];
  INERTIALNAVIGATION.AccelerationEarthFrame_LPF[ROLL] = INERTIALNAVIGATION.AccelerationEarthFrame_LPF[ROLL] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[ROLL] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[ROLL] += INERTIALNAVIGATION.AccelerationEarthFrame_LPF[ROLL];
  INS.AccelerationEarthFrame_Sum_Count[ROLL]++;

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[PITCH]);
  INERTIALNAVIGATION.AccelerationDifference[PITCH] = INS.AccelerationEarthFrame[PITCH] - INERTIALNAVIGATION.AccelerationAdjustBias[PITCH];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[PITCH] = INERTIALNAVIGATION.AccelerationAdjustBias[PITCH] * 0.985f + INS.AccelerationEarthFrame[PITCH] * 0.015f; //2HZ LPF
  }
  else if (ABS(INERTIALNAVIGATION.AccelerationDifference[PITCH]) <= 80.0f)
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[PITCH] = INERTIALNAVIGATION.AccelerationAdjustBias[PITCH] * 0.9987f + INS.AccelerationEarthFrame[PITCH] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[PITCH] = INERTIALNAVIGATION.AccelerationDifference[PITCH];
  INERTIALNAVIGATION.AccelerationEarthFrame_LPF[PITCH] = INERTIALNAVIGATION.AccelerationEarthFrame_LPF[PITCH] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[PITCH] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[PITCH] += INERTIALNAVIGATION.AccelerationEarthFrame_LPF[PITCH];
  INS.AccelerationEarthFrame_Sum_Count[PITCH]++;

  //YAW
  INS.AccelerationEarthFrame[YAW] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[YAW]);
  INERTIALNAVIGATION.AccelerationDifference[YAW] = INS.AccelerationEarthFrame[YAW] - INERTIALNAVIGATION.AccelerationAdjustBias[YAW];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[YAW] = INERTIALNAVIGATION.AccelerationAdjustBias[YAW] * 0.985f + INS.AccelerationEarthFrame[YAW] * 0.015f; //2HZ LPF
  }
  else if (ABS(INERTIALNAVIGATION.AccelerationDifference[YAW]) <= 80.0f)
  {
    INERTIALNAVIGATION.AccelerationAdjustBias[YAW] = INERTIALNAVIGATION.AccelerationAdjustBias[YAW] * 0.9987f + INS.AccelerationEarthFrame[YAW] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[YAW] = INERTIALNAVIGATION.AccelerationDifference[YAW];
  INERTIALNAVIGATION.AccelerationEarthFrame_LPF[YAW] = INERTIALNAVIGATION.AccelerationEarthFrame_LPF[YAW] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[YAW] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[YAW] += INERTIALNAVIGATION.AccelerationEarthFrame_LPF[YAW];
  INS.AccelerationEarthFrame_Sum_Count[YAW]++;
}

void InertialNavigationClass::UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount)
{
  INS.AccelerationEarthFrame_Filtered[ArrayCount] = INS.AccelerationEarthFrame_Sum[ArrayCount] / INS.AccelerationEarthFrame_Sum_Count[ArrayCount];
  INS.AccelerationEarthFrame_Sum[ArrayCount] = 0.0f;
  INS.AccelerationEarthFrame_Sum_Count[ArrayCount] = 0;
}

void InertialNavigationClass::Calculate_AccelerationXY()
{
  static Scheduler_Struct Calculate_AccelerationXYTimer;
  if (Scheduler(&Calculate_AccelerationXYTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
  {
    static bool ResetAccelerationXY = false;
    INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LATITUDE);
    INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LONGITUDE);
    if (Get_State_Armed_With_GPS())
    {
      if (!ResetAccelerationXY)
      {
        ResetAccelerationXY = true;
        INERTIALNAVIGATION.ResetXYState();
      }
      float DeltaTime = Calculate_AccelerationXYTimer.ActualTime * 1e-6f;
      INERTIALNAVIGATION.CorrectXYStateWithGPS(&DeltaTime);
      INERTIALNAVIGATION.EstimationPredictXY(&DeltaTime);
      INERTIALNAVIGATION.SaveXYPositionToHistory();
      return;
    }
    else
    {
      ResetAccelerationXY = false;
    }
  }
}

void InertialNavigationClass::CorrectXYStateWithGPS(float *DeltaTime)
{
  float PositionError[2];
  PositionError[INS_LATITUDE] = GPSDistanceToHome[INS_LATITUDE] - INERTIALNAVIGATION.HistoryXYPosition[INS_LATITUDE][INERTIALNAVIGATION.HistoryXYCount];
  PositionError[INS_LONGITUDE] = GPSDistanceToHome[INS_LONGITUDE] - INERTIALNAVIGATION.HistoryXYPosition[INS_LONGITUDE][INERTIALNAVIGATION.HistoryXYCount];
  PositionError[INS_LATITUDE] = Constrain_Float(PositionError[INS_LATITUDE], -1000, 1000);
  PositionError[INS_LONGITUDE] = Constrain_Float(PositionError[INS_LONGITUDE], -1000, 1000);
  float StoredDeltaTime;
  StoredDeltaTime = 0.48f * *DeltaTime;
  INS.Velocity_EarthFrame[INS_LATITUDE] += PositionError[INS_LATITUDE] * StoredDeltaTime;
  INS.Velocity_EarthFrame[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * StoredDeltaTime;
  StoredDeltaTime = 1.2f * *DeltaTime;
  INS.Position_EarthFrame[INS_LATITUDE] += PositionError[INS_LATITUDE] * StoredDeltaTime;
  INS.Position_EarthFrame[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * StoredDeltaTime;
}

void InertialNavigationClass::EstimationPredictXY(float *DeltaTime)
{
  float VelocityIncrease[2];
  VelocityIncrease[INS_LATITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LATITUDE] * *DeltaTime;
  VelocityIncrease[INS_LONGITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LONGITUDE] * *DeltaTime;
  INS.Position_EarthFrame[INS_LATITUDE] += (INS.Velocity_EarthFrame[INS_LATITUDE] + VelocityIncrease[INS_LATITUDE] * 0.5f) * *DeltaTime;    //POSIÇÃO FINAL X ESTIMADA PELO INS
  INS.Position_EarthFrame[INS_LONGITUDE] += (INS.Velocity_EarthFrame[INS_LONGITUDE] + VelocityIncrease[INS_LONGITUDE] * 0.5f) * *DeltaTime; //POSIÇÃO FINAL Y ESTIMADA PELO INS
  INS.Velocity_EarthFrame[INS_LATITUDE] += VelocityIncrease[INS_LATITUDE];                                                                  //VELOCIDADE FINAL X ESTIMADA PELO INS
  INS.Velocity_EarthFrame[INS_LONGITUDE] += VelocityIncrease[INS_LONGITUDE];                                                                //VELOCIDADE FINAL Y ESTIMADA PELO INS
}

void InertialNavigationClass::SaveXYPositionToHistory()
{
  INERTIALNAVIGATION.HistoryXYPosition[INS_LATITUDE][INERTIALNAVIGATION.HistoryXYCount] = INS.Position_EarthFrame[INS_LATITUDE];
  INERTIALNAVIGATION.HistoryXYPosition[INS_LONGITUDE][INERTIALNAVIGATION.HistoryXYCount] = INS.Position_EarthFrame[INS_LONGITUDE];
  INERTIALNAVIGATION.HistoryXYCount++;
  if (INERTIALNAVIGATION.HistoryXYCount >= 10)
  {
    INERTIALNAVIGATION.HistoryXYCount = 0;
  }
}

void InertialNavigationClass::ResetXYState()
{
  INERTIALNAVIGATION.HistoryXYCount = 0;
  for (uint8_t i = 0; i < 2; i++)
  {
    INS.Velocity_EarthFrame[i] = (ABS(GPSActualSpeed[i]) > 50) ? GPSActualSpeed[i] : 0.0f;
    INS.Position_EarthFrame[i] = GPSDistanceToHome[i];
    for (uint8_t j = 0; j < 10; j++)
    {
      INERTIALNAVIGATION.HistoryXYPosition[i][j] = GPSDistanceToHome[i];
    }
  }
}

void InertialNavigationClass::Calculate_AccelerationZ()
{
  static Scheduler_Struct Calculate_AccelerationZTimer;
  if (Scheduler(&Calculate_AccelerationZTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
  {
    static bool ResetAccelerationZ = false;
    CalculateBaroAltitudeForFlight();
    INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_VERTICAL_Z);
    if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      if (!ResetAccelerationZ)
      {
        ResetAccelerationZ = true;
        INERTIALNAVIGATION.ResetZState();
      }
      float DeltaTime = Calculate_AccelerationZTimer.ActualTime * 1e-6f;
      INERTIALNAVIGATION.CorrectZStateWithBaro(&DeltaTime);
      INERTIALNAVIGATION.EstimationPredictZ(&DeltaTime);
      INERTIALNAVIGATION.SaveZPositionToHistory();
      return;
    }
    else
    {
      ResetAccelerationZ = false;
    }
  }
}

void InertialNavigationClass::CorrectZStateWithBaro(float *DeltaTime)
{
  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (Barometer.Altitude.Actual < Barometer.Altitude.GroundOffSet);
  float PositionError = (AirCushionEffectDetected ? Barometer.Altitude.GroundOffSet : Barometer.Altitude.Actual) - INERTIALNAVIGATION.HistoryZPosition[INERTIALNAVIGATION.HistoryZCount];
  INS.Velocity_EarthFrame[INS_VERTICAL_Z] += PositionError * (0.19753086419753086419753086419753f * *DeltaTime);
  INS.Position_EarthFrame[INS_VERTICAL_Z] += PositionError * (0.66666666666666666666666666666667f * *DeltaTime);
}

void InertialNavigationClass::EstimationPredictZ(float *DeltaTime)
{
  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * *DeltaTime;
  INS.Position_EarthFrame[INS_VERTICAL_Z] += (INS.Velocity_EarthFrame[INS_VERTICAL_Z] + VelocityIncrease * 0.5f) * *DeltaTime;
  INS.Velocity_EarthFrame[INS_VERTICAL_Z] += VelocityIncrease;
  Barometer.INS.Altitude.Estimated = INS.Position_EarthFrame[INS_VERTICAL_Z]; //ALTITUDE FINAL ESTIMADA PELO INS
  Barometer.INS.Velocity.Vertical = INS.Velocity_EarthFrame[INS_VERTICAL_Z];  //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS
}

void InertialNavigationClass::SaveZPositionToHistory()
{
  INERTIALNAVIGATION.HistoryZPosition[INERTIALNAVIGATION.HistoryZCount] = Barometer.INS.Altitude.Estimated;
  INERTIALNAVIGATION.HistoryZCount++;
  if (INERTIALNAVIGATION.HistoryZCount >= 10)
  {
    INERTIALNAVIGATION.HistoryZCount = 0;
  }
}

void InertialNavigationClass::ResetZState()
{
  INS.Position_EarthFrame[INS_VERTICAL_Z] = 0.0f;
  INS.Velocity_EarthFrame[INS_VERTICAL_Z] = 0.0f;
  INERTIALNAVIGATION.HistoryZCount = 0;
  INERTIALNAVIGATION.HistoryZPosition[0] = 0;
  INERTIALNAVIGATION.HistoryZPosition[1] = 0;
  INERTIALNAVIGATION.HistoryZPosition[2] = 0;
  INERTIALNAVIGATION.HistoryZPosition[3] = 0;
  INERTIALNAVIGATION.HistoryZPosition[4] = 0;
  INERTIALNAVIGATION.HistoryZPosition[5] = 0;
  INERTIALNAVIGATION.HistoryZPosition[6] = 0;
  INERTIALNAVIGATION.HistoryZPosition[7] = 0;
  INERTIALNAVIGATION.HistoryZPosition[8] = 0;
  INERTIALNAVIGATION.HistoryZPosition[9] = 0;
}
