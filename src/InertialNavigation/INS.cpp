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
  //OBTÉM O SENO E COSSENO DE CADA EIXO DADO PELO AHRS
  INS.Math.Cosine_Roll = AHRS.GetCosineRoll();
  INS.Math.Sine_Roll = AHRS.GetSineRoll();
  INS.Math.Cosine_Pitch = -AHRS.GetCosinePitch();
  INS.Math.Sine_Pitch = -AHRS.GetSinePitch();
  INS.Math.Cosine_Yaw = AHRS.GetCosineYaw();
  INS.Math.Sine_Yaw = AHRS.GetSineYaw();

  //REALIZA A FUSÃO DE ALGUMAS VARIAVEIS PARA OBTER DIFERENTES VALORES DE ACELERAÇÃO EM TORNO DO EIXO Z
  INS.Math.Sine_Pitch_Cosine_Yaw_Fusion = INS.Math.Sine_Pitch * INS.Math.Cosine_Yaw;
  INS.Math.Sine_Pitch_Sine_Yaw_Fusion = INS.Math.Sine_Pitch * INS.Math.Sine_Yaw;

  //ROLL
  INS.EarthFrame.Acceleration[ROLL] = -((INS.Math.Cosine_Pitch * INS.Math.Cosine_Yaw) * IMU.Accelerometer.Read[PITCH] + (INS.Math.Sine_Roll * INS.Math.Sine_Pitch_Cosine_Yaw_Fusion - INS.Math.Cosine_Roll * INS.Math.Sine_Yaw) * IMU.Accelerometer.Read[ROLL] + (INS.Math.Sine_Roll * INS.Math.Sine_Yaw + INS.Math.Cosine_Roll * INS.Math.Sine_Pitch_Cosine_Yaw_Fusion) * IMU.Accelerometer.Read[YAW]);

  //PITCH
  INS.EarthFrame.Acceleration[PITCH] = -((INS.Math.Cosine_Pitch * INS.Math.Sine_Yaw) * IMU.Accelerometer.Read[PITCH] + (INS.Math.Cosine_Roll * INS.Math.Cosine_Yaw + INS.Math.Sine_Roll * INS.Math.Sine_Pitch_Sine_Yaw_Fusion) * IMU.Accelerometer.Read[ROLL] + (-INS.Math.Sine_Roll * INS.Math.Cosine_Yaw + INS.Math.Cosine_Roll * INS.Math.Sine_Pitch_Sine_Yaw_Fusion) * IMU.Accelerometer.Read[YAW]);

  //YAW
  INS.EarthFrame.Acceleration[YAW] = ((-INS.Math.Sine_Pitch) * IMU.Accelerometer.Read[PITCH] + (INS.Math.Sine_Roll * INS.Math.Cosine_Pitch) * IMU.Accelerometer.Read[ROLL] + (INS.Math.Cosine_Roll * INS.Math.Cosine_Pitch) * IMU.Accelerometer.Read[YAW]) - 512;

  //ROLL
  INS.EarthFrame.Acceleration[ROLL] = ConvertAccelerationEarthFrameToCMSS(INS.EarthFrame.Acceleration[ROLL]);
  INS.Bias.Difference[ROLL] = INS.EarthFrame.Acceleration[ROLL] - INS.Bias.Adjust[ROLL];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[ROLL] = INS.Bias.Adjust[ROLL] * 0.985f + INS.EarthFrame.Acceleration[ROLL] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[ROLL]) <= 80.0f)
  {
    INS.Bias.Adjust[ROLL] = INS.Bias.Adjust[ROLL] * 0.9987f + INS.EarthFrame.Acceleration[ROLL] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.Acceleration[ROLL] = INS.Bias.Difference[ROLL];
  INS.AccelerationEarthFrame_LPF[ROLL] = INS.AccelerationEarthFrame_LPF[ROLL] * 0.85714285714285714285714285714286f + INS.EarthFrame.Acceleration[ROLL] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[ROLL] += INS.AccelerationEarthFrame_LPF[ROLL];
  INS.AccelerationEarthFrame_Sum_Count[ROLL]++;

  //PITCH
  INS.EarthFrame.Acceleration[PITCH] = ConvertAccelerationEarthFrameToCMSS(INS.EarthFrame.Acceleration[PITCH]);
  INS.Bias.Difference[PITCH] = INS.EarthFrame.Acceleration[PITCH] - INS.Bias.Adjust[PITCH];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[PITCH] = INS.Bias.Adjust[PITCH] * 0.985f + INS.EarthFrame.Acceleration[PITCH] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[PITCH]) <= 80.0f)
  {
    INS.Bias.Adjust[PITCH] = INS.Bias.Adjust[PITCH] * 0.9987f + INS.EarthFrame.Acceleration[PITCH] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.Acceleration[PITCH] = INS.Bias.Difference[PITCH];
  INS.AccelerationEarthFrame_LPF[PITCH] = INS.AccelerationEarthFrame_LPF[PITCH] * 0.85714285714285714285714285714286f + INS.EarthFrame.Acceleration[PITCH] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[PITCH] += INS.AccelerationEarthFrame_LPF[PITCH];
  INS.AccelerationEarthFrame_Sum_Count[PITCH]++;

  //YAW
  INS.EarthFrame.Acceleration[YAW] = ConvertAccelerationEarthFrameToCMSS(INS.EarthFrame.Acceleration[YAW]);
  INS.Bias.Difference[YAW] = INS.EarthFrame.Acceleration[YAW] - INS.Bias.Adjust[YAW];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[YAW] = INS.Bias.Adjust[YAW] * 0.985f + INS.EarthFrame.Acceleration[YAW] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[YAW]) <= 80.0f)
  {
    INS.Bias.Adjust[YAW] = INS.Bias.Adjust[YAW] * 0.9987f + INS.EarthFrame.Acceleration[YAW] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.Acceleration[YAW] = INS.Bias.Difference[YAW];
  INS.AccelerationEarthFrame_LPF[YAW] = INS.AccelerationEarthFrame_LPF[YAW] * 0.85714285714285714285714285714286f + INS.EarthFrame.Acceleration[YAW] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[YAW] += INS.AccelerationEarthFrame_LPF[YAW];
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
      INERTIALNAVIGATION.CorrectXYStateWithGPS(DeltaTime);
      INERTIALNAVIGATION.EstimationPredictXY(DeltaTime);
      INERTIALNAVIGATION.SaveXYPositionToHistory();
      return;
    }
    else
    {
      ResetAccelerationXY = false;
    }
  }
}

void InertialNavigationClass::CorrectXYStateWithGPS(float DeltaTime)
{
  float PositionError[2];
  PositionError[INS_LATITUDE] = GPSDistanceToHome[INS_LATITUDE] - INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount];
  PositionError[INS_LONGITUDE] = GPSDistanceToHome[INS_LONGITUDE] - INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount];
  PositionError[INS_LATITUDE] = Constrain_Float(PositionError[INS_LATITUDE], -1000, 1000);
  PositionError[INS_LONGITUDE] = Constrain_Float(PositionError[INS_LONGITUDE], -1000, 1000);
  INS.EarthFrame.Velocity[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Velocity[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Position[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 1.2f);
  INS.EarthFrame.Position[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 1.2f);
}

void InertialNavigationClass::EstimationPredictXY(float DeltaTime)
{
  float VelocityIncrease[2];
  VelocityIncrease[INS_LATITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LATITUDE] * DeltaTime;
  VelocityIncrease[INS_LONGITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LONGITUDE] * DeltaTime;
  INS.EarthFrame.Position[INS_LATITUDE] += (INS.EarthFrame.Velocity[INS_LATITUDE] + VelocityIncrease[INS_LATITUDE] * 0.5f) * DeltaTime;    //POSIÇÃO FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Position[INS_LONGITUDE] += (INS.EarthFrame.Velocity[INS_LONGITUDE] + VelocityIncrease[INS_LONGITUDE] * 0.5f) * DeltaTime; //POSIÇÃO FINAL Y ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LATITUDE] += VelocityIncrease[INS_LATITUDE];                                                                 //VELOCIDADE FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LONGITUDE] += VelocityIncrease[INS_LONGITUDE];                                                               //VELOCIDADE FINAL Y ESTIMADA PELO INS
}

void InertialNavigationClass::SaveXYPositionToHistory()
{
  INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LATITUDE];
  INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LONGITUDE];
  INS.History.XYCount++;
  if (INS.History.XYCount >= 10)
  {
    INS.History.XYCount = 0;
  }
}

void InertialNavigationClass::ResetXYState()
{
  INS.History.XYCount = 0;
  INS.EarthFrame.Velocity[INS_LATITUDE] = (ABS(GPSActualSpeed[INS_LATITUDE]) > 50) ? GPSActualSpeed[INS_LATITUDE] : 0.0f;
  INS.EarthFrame.Position[INS_LATITUDE] = GPSDistanceToHome[INS_LATITUDE];
  INS.EarthFrame.Velocity[INS_LONGITUDE] = (ABS(GPSActualSpeed[INS_LONGITUDE]) > 50) ? GPSActualSpeed[INS_LONGITUDE] : 0.0f;
  INS.EarthFrame.Position[INS_LONGITUDE] = GPSDistanceToHome[INS_LONGITUDE];
  INS.History.XYPosition[INS_LATITUDE][0] = GPSDistanceToHome[0];
  INS.History.XYPosition[INS_LATITUDE][1] = GPSDistanceToHome[1];
  INS.History.XYPosition[INS_LATITUDE][2] = GPSDistanceToHome[2];
  INS.History.XYPosition[INS_LATITUDE][3] = GPSDistanceToHome[3];
  INS.History.XYPosition[INS_LATITUDE][4] = GPSDistanceToHome[4];
  INS.History.XYPosition[INS_LATITUDE][5] = GPSDistanceToHome[5];
  INS.History.XYPosition[INS_LATITUDE][6] = GPSDistanceToHome[6];
  INS.History.XYPosition[INS_LATITUDE][7] = GPSDistanceToHome[7];
  INS.History.XYPosition[INS_LATITUDE][8] = GPSDistanceToHome[8];
  INS.History.XYPosition[INS_LATITUDE][9] = GPSDistanceToHome[9];
  INS.History.XYPosition[INS_LONGITUDE][0] = GPSDistanceToHome[0];
  INS.History.XYPosition[INS_LONGITUDE][1] = GPSDistanceToHome[1];
  INS.History.XYPosition[INS_LONGITUDE][2] = GPSDistanceToHome[2];
  INS.History.XYPosition[INS_LONGITUDE][3] = GPSDistanceToHome[3];
  INS.History.XYPosition[INS_LONGITUDE][4] = GPSDistanceToHome[4];
  INS.History.XYPosition[INS_LONGITUDE][5] = GPSDistanceToHome[5];
  INS.History.XYPosition[INS_LONGITUDE][6] = GPSDistanceToHome[6];
  INS.History.XYPosition[INS_LONGITUDE][7] = GPSDistanceToHome[7];
  INS.History.XYPosition[INS_LONGITUDE][8] = GPSDistanceToHome[8];
  INS.History.XYPosition[INS_LONGITUDE][9] = GPSDistanceToHome[9];
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
      INERTIALNAVIGATION.CorrectZStateWithBaro(DeltaTime);
      INERTIALNAVIGATION.EstimationPredictZ(DeltaTime);
      INERTIALNAVIGATION.SaveZPositionToHistory();
      return;
    }
    else
    {
      ResetAccelerationZ = false;
    }
  }
}

void InertialNavigationClass::CorrectZStateWithBaro(float DeltaTime)
{
  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (Barometer.Altitude.Actual < Barometer.Altitude.GroundOffSet);
  float PositionError = (AirCushionEffectDetected ? Barometer.Altitude.GroundOffSet : Barometer.Altitude.Actual) - INS.History.ZPosition[INS.History.ZCount];
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += PositionError * (0.19753086419753086419753086419753f * DeltaTime);
  INS.EarthFrame.Position[INS_VERTICAL_Z] += PositionError * (0.66666666666666666666666666666667f * DeltaTime);
}

void InertialNavigationClass::EstimationPredictZ(float DeltaTime)
{
  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * DeltaTime;
  INS.EarthFrame.Position[INS_VERTICAL_Z] += (INS.EarthFrame.Velocity[INS_VERTICAL_Z] + VelocityIncrease * 0.5f) * DeltaTime;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += VelocityIncrease;
  Barometer.INS.Altitude.Estimated = INS.EarthFrame.Position[INS_VERTICAL_Z]; //ALTITUDE FINAL ESTIMADA PELO INS
  Barometer.INS.Velocity.Vertical = INS.EarthFrame.Velocity[INS_VERTICAL_Z];  //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS
}

void InertialNavigationClass::SaveZPositionToHistory()
{
  INS.History.ZPosition[INS.History.ZCount] = Barometer.INS.Altitude.Estimated;
  INS.History.ZCount++;
  if (INS.History.ZCount >= 10)
  {
    INS.History.ZCount = 0;
  }
}

void InertialNavigationClass::ResetZState()
{
  INS.EarthFrame.Position[INS_VERTICAL_Z] = 0.0f;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] = 0.0f;
  INS.History.ZCount = 0;
  INS.History.ZPosition[0] = 0;
  INS.History.ZPosition[1] = 0;
  INS.History.ZPosition[2] = 0;
  INS.History.ZPosition[3] = 0;
  INS.History.ZPosition[4] = 0;
  INS.History.ZPosition[5] = 0;
  INS.History.ZPosition[6] = 0;
  INS.History.ZPosition[7] = 0;
  INS.History.ZPosition[8] = 0;
  INS.History.ZPosition[9] = 0;
}
