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
#include "Common/VARIABLES.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "Scheduler/SCHEDULER.h"
#include "Barometer/BAROREAD.h"
#include "Math/MATHSUPPORT.h"
#include "GPS/GPSSTATES.h"
#include "AHRS/AHRS.h"

InertialNavigationClass INERTIALNAVIGATION;

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
  Cosine_Yaw = AHRS.CosineYaw();
  Sine_Yaw = AHRS.SineYaw();

  //REALIZA A FUSÃO DE ALGUMAS VARIAVEIS PARA OBTER DIFERENTES VALORES DE ACELERAÇÃO EM TORNO DO EIXO Z
  SinePitch_CosineYaw_Fusion = Sine_Pitch * Cosine_Yaw;
  SinePitch_SineYaw_Fusion = Sine_Pitch * Sine_Yaw;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = -((Cosine_Pitch * Cosine_Yaw) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * SinePitch_CosineYaw_Fusion - Cosine_Roll * Sine_Yaw) * IMU.AccelerometerRead[ROLL] + (Sine_Roll * Sine_Yaw + Cosine_Roll * SinePitch_CosineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = -((Cosine_Pitch * Sine_Yaw) * IMU.AccelerometerRead[PITCH] + (Cosine_Roll * Cosine_Yaw + Sine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[ROLL] + (-Sine_Roll * Cosine_Yaw + Cosine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //YAW
  INS.AccelerationEarthFrame[YAW] = ((-Sine_Pitch) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[ROLL] + (Cosine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[YAW]) - 512;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[ROLL]);
  AccelerationDifference[ROLL] = INS.AccelerationEarthFrame[ROLL] - AccelerationAdjustBias[ROLL];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    AccelerationAdjustBias[ROLL] = AccelerationAdjustBias[ROLL] * 0.985f + INS.AccelerationEarthFrame[ROLL] * 0.015f; //2HZ LPF
  }
  else if (ABS(AccelerationDifference[ROLL]) <= 80.0f)
  {
    AccelerationAdjustBias[ROLL] = AccelerationAdjustBias[ROLL] * 0.9987f + INS.AccelerationEarthFrame[ROLL] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[ROLL] = AccelerationDifference[ROLL];
  AccelerationEarthFrame_LPF[ROLL] = AccelerationEarthFrame_LPF[ROLL] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[ROLL] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[ROLL] += AccelerationEarthFrame_LPF[ROLL];
  INS.AccelerationEarthFrame_Sum_Count[ROLL]++;

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[PITCH]);
  AccelerationDifference[PITCH] = INS.AccelerationEarthFrame[PITCH] - AccelerationAdjustBias[PITCH];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    AccelerationAdjustBias[PITCH] = AccelerationAdjustBias[PITCH] * 0.985f + INS.AccelerationEarthFrame[PITCH] * 0.015f; //2HZ LPF
  }
  else if (ABS(AccelerationDifference[PITCH]) <= 80.0f)
  {
    AccelerationAdjustBias[PITCH] = AccelerationAdjustBias[PITCH] * 0.9987f + INS.AccelerationEarthFrame[PITCH] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[PITCH] = AccelerationDifference[PITCH];
  AccelerationEarthFrame_LPF[PITCH] = AccelerationEarthFrame_LPF[PITCH] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[PITCH] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[PITCH] += AccelerationEarthFrame_LPF[PITCH];
  INS.AccelerationEarthFrame_Sum_Count[PITCH]++;

  //YAW
  INS.AccelerationEarthFrame[YAW] = ConvertAccelerationEarthFrameToCMSS(INS.AccelerationEarthFrame[YAW]);
  AccelerationDifference[YAW] = INS.AccelerationEarthFrame[YAW] - AccelerationAdjustBias[YAW];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    AccelerationAdjustBias[YAW] = AccelerationAdjustBias[YAW] * 0.985f + INS.AccelerationEarthFrame[YAW] * 0.015f; //2HZ LPF
  }
  else if (ABS(AccelerationDifference[YAW]) <= 80.0f)
  {
    AccelerationAdjustBias[YAW] = AccelerationAdjustBias[YAW] * 0.9987f + INS.AccelerationEarthFrame[YAW] * 0.0013f; //1HZ LPF
  }
  INS.AccelerationEarthFrame[YAW] = AccelerationDifference[YAW];
  AccelerationEarthFrame_LPF[YAW] = AccelerationEarthFrame_LPF[YAW] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[YAW] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[YAW] += AccelerationEarthFrame_LPF[YAW];
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
    UpdateAccelerationEarthFrame_Filtered(0);
    UpdateAccelerationEarthFrame_Filtered(1);
    if (Get_State_Armed_With_GPS())
    {
      if (!ResetAccelerationXY)
      {
        ResetAccelerationXY = true;
        ResetXYState();
      }
      float DeltaTime = Calculate_AccelerationXYTimer.ActualTime * 1e-6f;
      CorrectXYStateWithGPS(&DeltaTime);
      EstimationPredictXY(&DeltaTime);
      SaveXYPositionToHistory();
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
  PositionError[0] = GPSDistanceToHome[0] - HistoryXYPosition[0][HistoryXYCount];
  PositionError[1] = GPSDistanceToHome[1] - HistoryXYPosition[1][HistoryXYCount];
  PositionError[0] = Constrain_Float(PositionError[0], -1000, 1000);
  PositionError[1] = Constrain_Float(PositionError[1], -1000, 1000);
  float StoredDeltaTime;
  StoredDeltaTime = 0.48f * *DeltaTime;
  INS.Velocity_EarthFrame[0] += PositionError[0] * StoredDeltaTime;
  INS.Velocity_EarthFrame[1] += PositionError[1] * StoredDeltaTime;
  StoredDeltaTime = 1.2f * *DeltaTime;
  INS.Position_EarthFrame[0] += PositionError[0] * StoredDeltaTime;
  INS.Position_EarthFrame[1] += PositionError[1] * StoredDeltaTime;
}

void InertialNavigationClass::EstimationPredictXY(float *DeltaTime)
{
  float VelocityIncrease[2];
  VelocityIncrease[0] = INS.AccelerationEarthFrame_Filtered[0] * *DeltaTime;
  VelocityIncrease[1] = INS.AccelerationEarthFrame_Filtered[1] * *DeltaTime;
  INS.Position_EarthFrame[0] += (INS.Velocity_EarthFrame[0] + VelocityIncrease[0] * 0.5f) * *DeltaTime; //POSIÇÃO FINAL X ESTIMADA PELO INS
  INS.Position_EarthFrame[1] += (INS.Velocity_EarthFrame[1] + VelocityIncrease[1] * 0.5f) * *DeltaTime; //POSIÇÃO FINAL Y ESTIMADA PELO INS
  INS.Velocity_EarthFrame[0] += VelocityIncrease[0];                                                    //VALOCIDADE FINAL X ESTIMADA PELO INS
  INS.Velocity_EarthFrame[1] += VelocityIncrease[1];                                                    //VELOCIDADE FINAL Y ESTIMADA PELO INS
}

void InertialNavigationClass::SaveXYPositionToHistory()
{
  HistoryXYPosition[0][HistoryXYCount] = INS.Position_EarthFrame[0];
  HistoryXYPosition[1][HistoryXYCount] = INS.Position_EarthFrame[1];
  HistoryXYCount++;
  if (HistoryXYCount >= 10)
  {
    HistoryXYCount = 0;
  }
}

void InertialNavigationClass::ResetXYState()
{
  HistoryXYCount = 0;
  for (uint8_t i = 0; i < 2; i++)
  {
    INS.Velocity_EarthFrame[i] = (ABS(GPSActualSpeed[i]) > 50) ? GPSActualSpeed[i] : 0.0f;
    INS.Position_EarthFrame[i] = GPSDistanceToHome[i];
    for (uint8_t j = 0; j < 10; j++)
    {
      HistoryXYPosition[i][j] = GPSDistanceToHome[i];
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
    UpdateAccelerationEarthFrame_Filtered(2);
    if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
    {
      if (!ResetAccelerationZ)
      {
        ResetAccelerationZ = true;
        ResetZState();
      }
      float DeltaTime = Calculate_AccelerationZTimer.ActualTime * 1e-6f;
      CorrectZStateWithBaro(&DeltaTime);
      EstimationPredictZ(&DeltaTime);
      SaveZPositionToHistory();
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
  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (ALTITUDE.RealBaroAltitude < ALTITUDE.GroundAltitude);
  float PositionError = (AirCushionEffectDetected ? ALTITUDE.GroundAltitude : ALTITUDE.RealBaroAltitude) - HistoryZPosition[HistoryZCount];
  INS.Velocity_EarthFrame[2] += PositionError * (0.19753086419753086419753086419753f * *DeltaTime);
  INS.Position_EarthFrame[2] += PositionError * (0.66666666666666666666666666666667f * *DeltaTime);
}

void InertialNavigationClass::EstimationPredictZ(float *DeltaTime)
{
  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[2] * *DeltaTime;
  INS.Position_EarthFrame[2] += (INS.Velocity_EarthFrame[2] + VelocityIncrease * 0.5f) * *DeltaTime;
  INS.Velocity_EarthFrame[2] += VelocityIncrease;
  ALTITUDE.EstimatedAltitude = INS.Position_EarthFrame[2];   //ALTITUDE FINAL ESTIMADA APÓS O INS
  ALTITUDE.EstimatedVariometer = INS.Velocity_EarthFrame[2]; //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA APÓS O INS
}

void InertialNavigationClass::SaveZPositionToHistory()
{
  HistoryZPosition[HistoryZCount] = ALTITUDE.EstimatedAltitude;
  HistoryZCount++;
  if (HistoryZCount >= 10)
  {
    HistoryZCount = 0;
  }
}

void InertialNavigationClass::ResetZState()
{
  INS.Position_EarthFrame[2] = 0.0f;
  INS.Velocity_EarthFrame[2] = 0.0f;
  HistoryZCount = 0;
  HistoryZPosition[0] = 0;
  HistoryZPosition[1] = 0;
  HistoryZPosition[2] = 0;
  HistoryZPosition[3] = 0;
  HistoryZPosition[4] = 0;
  HistoryZPosition[5] = 0;
  HistoryZPosition[6] = 0;
  HistoryZPosition[7] = 0;
  HistoryZPosition[8] = 0;
  HistoryZPosition[9] = 0;
}
