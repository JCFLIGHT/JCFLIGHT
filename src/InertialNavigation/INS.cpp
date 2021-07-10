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
#include "Math/MATHSUPPORT.h"
#include "GPS/GPSSTATES.h"
#include "AHRS/AHRS.h"
#include "IMU/ACCGYROREAD.h"
#include "Barometer/BAROBACKEND.h"
#include "PerformanceCalibration/PERFORMGRAVITY.h"
#include "BitArray/BITARRAY.h"
#include "FastSerial/PRINTF.h"

InertialNavigationClass INERTIALNAVIGATION;
INS_Struct INS;

#define MAX_INS_POSITION_ERROR 1000.0f //ERRO MAXIMO DE POSIÇÃO QUE SE PODE ACUMULAR NO INS (EM CM)

//DEBUG
//#define PRINTLN_INS_COS_SIN
//#define PRINTLN_INS_ACC_NEU
//#define PRINTLN_INS_POS_VEL_XY
//#define PRINTLN_INS_POS_VEL_Z

void InertialNavigationClass::Calculate_AccelerationXYZ_To_EarthFrame(void)
{
  Vector3x3_Struct EarthFrameAcceleration;

  //OBTÉM O SENO E COSSENO DO YAW DADO PELO AHRS
  INS.Math.Cosine.Yaw = AHRS.GetCosineYaw();
  INS.Math.Sine.Yaw = AHRS.GetSineYaw();

#ifdef PRINTLN_INS_COS_SIN

  //COM O COMPASS CALIBRADO E APONTANTO PARA 1000:COSSENO DEVE SER NEGATIVO E SENO DEVE SER POSITIVO

  DEBUG("%d %.2f %.2f",
        Attitude.EulerAngles.YawDecidegrees,
        INS.Math.Cosine.Yaw,
        INS.Math.Sine.Yaw);

#endif

  EarthFrameAcceleration.Roll = BodyFrameAcceleration.Roll;
  EarthFrameAcceleration.Pitch = BodyFrameAcceleration.Pitch;
  EarthFrameAcceleration.Yaw = BodyFrameAcceleration.Yaw;

  AHRS.TransformVectorBodyFrameToEarthFrame(&EarthFrameAcceleration);

  GRAVITYCALIBRATION.Update(&EarthFrameAcceleration);

  INS.EarthFrame.AccelerationNEU[NORTH] = EarthFrameAcceleration.Roll;
  INS.EarthFrame.AccelerationNEU[EAST] = EarthFrameAcceleration.Pitch;
  INS.EarthFrame.AccelerationNEU[UP] = EarthFrameAcceleration.Yaw;

  //NORTH
  INS.Bias.Difference[NORTH] = INS.EarthFrame.AccelerationNEU[NORTH] - INS.Bias.Adjust[NORTH];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[NORTH] = INS.Bias.Adjust[NORTH] * 0.985f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[NORTH]) <= 80.0f)
  {
    INS.Bias.Adjust[NORTH] = INS.Bias.Adjust[NORTH] * 0.9987f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[NORTH] = INS.Bias.Difference[NORTH];
  INS.AccelerationEarthFrame_LPF[NORTH] = INS.AccelerationEarthFrame_LPF[NORTH] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[NORTH] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[NORTH] += INS.AccelerationEarthFrame_LPF[NORTH];
  INS.AccelerationEarthFrame_Sum_Count[NORTH]++;

  //EAST
  INS.Bias.Difference[EAST] = INS.EarthFrame.AccelerationNEU[EAST] - INS.Bias.Adjust[EAST];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[EAST] = INS.Bias.Adjust[EAST] * 0.985f + INS.EarthFrame.AccelerationNEU[EAST] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[EAST]) <= 80.0f)
  {
    INS.Bias.Adjust[EAST] = INS.Bias.Adjust[EAST] * 0.9987f + INS.EarthFrame.AccelerationNEU[EAST] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[EAST] = INS.Bias.Difference[EAST];
  INS.AccelerationEarthFrame_LPF[EAST] = INS.AccelerationEarthFrame_LPF[EAST] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[EAST] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[EAST] += INS.AccelerationEarthFrame_LPF[EAST];
  INS.AccelerationEarthFrame_Sum_Count[EAST]++;

  //UP
  INS.Bias.Difference[UP] = INS.EarthFrame.AccelerationNEU[UP] - INS.Bias.Adjust[UP];
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INS.Bias.Adjust[UP] = INS.Bias.Adjust[UP] * 0.985f + INS.EarthFrame.AccelerationNEU[UP] * 0.015f; //2HZ LPF
  }
  else if (ABS(INS.Bias.Difference[UP]) <= 80.0f)
  {
    INS.Bias.Adjust[UP] = INS.Bias.Adjust[UP] * 0.9987f + INS.EarthFrame.AccelerationNEU[UP] * 0.0013f; //1HZ LPF
  }
  INS.EarthFrame.AccelerationNEU[UP] = INS.Bias.Difference[UP];
  INS.AccelerationEarthFrame_LPF[UP] = INS.AccelerationEarthFrame_LPF[UP] * 0.85714285714285714285714285714286f + INS.EarthFrame.AccelerationNEU[UP] * 0.14285714285714285714285714285714f; //25HZ LPF
  INS.AccelerationEarthFrame_Sum[UP] += INS.AccelerationEarthFrame_LPF[UP];
  INS.AccelerationEarthFrame_Sum_Count[UP]++;

#ifdef PRINTLN_INS_ACC_NEU

  //INS.EarthFrame.AccelerationNEU[NORTH] -> POSITIVO MOVENDO PARA O NORTE
  //INS.EarthFrame.AccelerationNEU[EAST]  -> POSITIVO MOVENDO PARA O OESTE
  //INS.EarthFrame.AccelerationNEU[UP]    -> POSITIVO MOVENDO PARA CIMA

  DEBUG("NORTH:%.4f EAST:%.4f UP:%.4f",
        INS.EarthFrame.AccelerationNEU[NORTH],
        INS.EarthFrame.AccelerationNEU[EAST],
        INS.EarthFrame.AccelerationNEU[UP]);

#endif
}

void InertialNavigationClass::UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount)
{
  INS.AccelerationEarthFrame_Filtered[ArrayCount] = INS.AccelerationEarthFrame_Sum[ArrayCount] / INS.AccelerationEarthFrame_Sum_Count[ArrayCount];
  INS.AccelerationEarthFrame_Sum[ArrayCount] = 0.0f;
  INS.AccelerationEarthFrame_Sum_Count[ArrayCount] = 0;
}

bool InertialNavigationClass::WaitForSample(void)
{
#define SAMPLE_DELAY 2.0f //SEGUNDOS

  static uint16_t SampleCount = 0;

  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    SampleCount = 0;
  }

  if (SampleCount >= (50 * SAMPLE_DELAY))
  {
    return true;
  }
  else
  {
    SampleCount++;
  }

  return false;
}

void InertialNavigationClass::Calculate_AccelerationXY(float DeltaTime)
{
  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LATITUDE);
  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_LONGITUDE);
  if (Get_State_Armed_With_GPS() && INERTIALNAVIGATION.WaitForSample())
  {
    INERTIALNAVIGATION.CorrectXYStateWithGPS(DeltaTime);
    INERTIALNAVIGATION.EstimationPredictXY(DeltaTime);
    INERTIALNAVIGATION.SaveXYPositionToHistory();
  }
  else
  {
    INERTIALNAVIGATION.ResetXYState();
  }
}

void InertialNavigationClass::CorrectXYStateWithGPS(float DeltaTime)
{
  float PositionError[2] = {0, 0};
  PositionError[INS_LATITUDE] = GPS_Resources.Home.INS.Distance[COORD_LATITUDE] - INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount];
  PositionError[INS_LONGITUDE] = GPS_Resources.Home.INS.Distance[COORD_LONGITUDE] - INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount];
  PositionError[INS_LATITUDE] = Constrain_Float(PositionError[INS_LATITUDE], -MAX_INS_POSITION_ERROR, MAX_INS_POSITION_ERROR);
  PositionError[INS_LONGITUDE] = Constrain_Float(PositionError[INS_LONGITUDE], -MAX_INS_POSITION_ERROR, MAX_INS_POSITION_ERROR);
  INS.EarthFrame.Velocity[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Velocity[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 0.48f);
  INS.EarthFrame.Position[INS_LATITUDE] += PositionError[INS_LATITUDE] * (DeltaTime * 1.2f);
  INS.EarthFrame.Position[INS_LONGITUDE] += PositionError[INS_LONGITUDE] * (DeltaTime * 1.2f);
}

void InertialNavigationClass::EstimationPredictXY(float DeltaTime)
{
  float VelocityIncrease[2] = {0, 0};
  VelocityIncrease[INS_LATITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LATITUDE] * DeltaTime;
  VelocityIncrease[INS_LONGITUDE] = INS.AccelerationEarthFrame_Filtered[INS_LONGITUDE] * DeltaTime;
  INS.EarthFrame.Position[INS_LATITUDE] += (INS.EarthFrame.Velocity[INS_LATITUDE] + VelocityIncrease[INS_LATITUDE] * 0.5f) * DeltaTime;    //POSIÇÃO FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Position[INS_LONGITUDE] += (INS.EarthFrame.Velocity[INS_LONGITUDE] + VelocityIncrease[INS_LONGITUDE] * 0.5f) * DeltaTime; //POSIÇÃO FINAL Y ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LATITUDE] += VelocityIncrease[INS_LATITUDE];                                                                 //VELOCIDADE FINAL X ESTIMADA PELO INS
  INS.EarthFrame.Velocity[INS_LONGITUDE] += VelocityIncrease[INS_LONGITUDE];                                                               //VELOCIDADE FINAL Y ESTIMADA PELO INS

#ifdef PRINTLN_INS_POS_VEL_XY

  DEBUG("%.2f %.2f %.2f %.2f",
        INS.EarthFrame.Position[INS_LATITUDE],
        INS.EarthFrame.Position[INS_LONGITUDE],
        INS.EarthFrame.Velocity[INS_LATITUDE],
        INS.EarthFrame.Velocity[INS_LONGITUDE]);

#endif
}

void InertialNavigationClass::SaveXYPositionToHistory(void)
{
  INS.History.XYPosition[INS_LATITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LATITUDE];
  INS.History.XYPosition[INS_LONGITUDE][INS.History.XYCount] = INS.EarthFrame.Position[INS_LONGITUDE];
  INS.History.XYCount++;
  if (INS.History.XYCount >= 10)
  {
    INS.History.XYCount = 0;
  }
}

void InertialNavigationClass::ResetXYState(void)
{
  INS.History.XYCount = 0;
  for (uint8_t IndexCount = 0; IndexCount < 2; IndexCount++)
  {
    INS.EarthFrame.Velocity[IndexCount] = (ABS(GPS_Resources.Navigation.Speed[IndexCount]) > 50) ? GPS_Resources.Navigation.Speed[IndexCount] : 0.0f;
    INS.EarthFrame.Position[IndexCount] = GPS_Resources.Home.INS.Distance[IndexCount];
    for (uint8_t SecondIndexCount = 0; SecondIndexCount < 10; SecondIndexCount++)
    {
      INS.History.XYPosition[IndexCount][SecondIndexCount] = GPS_Resources.Home.INS.Distance[IndexCount];
    }
  }
}

void InertialNavigationClass::Calculate_AccelerationZ(float DeltaTime)
{
  INERTIALNAVIGATION.UpdateAccelerationEarthFrame_Filtered(INS_VERTICAL_Z);
  if (IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
  {
    INERTIALNAVIGATION.CorrectZStateWithBaro(DeltaTime);
    INERTIALNAVIGATION.EstimationPredictZ(DeltaTime);
    INERTIALNAVIGATION.SaveZPositionToHistory();
  }
  else
  {
    INERTIALNAVIGATION.ResetZState();
  }
}

void InertialNavigationClass::CorrectZStateWithBaro(float DeltaTime)
{
  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (Barometer.Altitude.Actual < Barometer.Altitude.GroundOffSet);
  float BaroAltitudeResidual = (AirCushionEffectDetected ? Barometer.Altitude.GroundOffSet : Barometer.Altitude.Actual) - INS.History.ZPosition[INS.History.ZCount];
  INS.EarthFrame.Position[INS_VERTICAL_Z] += BaroAltitudeResidual * (0.66666666666666666666666666666667f * DeltaTime);
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += BaroAltitudeResidual * (0.19753086419753086419753086419753f * DeltaTime);
}

void InertialNavigationClass::EstimationPredictZ(float DeltaTime)
{
  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * DeltaTime;
  INS.EarthFrame.Position[INS_VERTICAL_Z] += (INS.EarthFrame.Velocity[INS_VERTICAL_Z] + VelocityIncrease * 0.5f) * DeltaTime;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] += VelocityIncrease;
  Barometer.INS.Altitude.Estimated = INS.EarthFrame.Position[INS_VERTICAL_Z]; //ALTITUDE FINAL ESTIMADA PELO INS
  Barometer.INS.Velocity.Vertical = INS.EarthFrame.Velocity[INS_VERTICAL_Z];  //VELOCIDADE VERTICAL(Z) FINAL ESTIMADA PELO INS

#ifdef PRINTLN_INS_POS_VEL_Z

  //Barometer.INS.Altitude.Estimated -> POSITIVO SUBINDO E NEGATIVO DESCENDO
  //Barometer.INS.Velocity.Vertical  -> POSITIVO SUBINDO E NEGATIVO DESCENDO

  DEBUG("%ld %d",
        Barometer.INS.Altitude.Estimated,
        Barometer.INS.Velocity.Vertical);

#endif
}

void InertialNavigationClass::SaveZPositionToHistory(void)
{
  INS.History.ZPosition[INS.History.ZCount] = Barometer.INS.Altitude.Estimated;
  INS.History.ZCount++;
  if (INS.History.ZCount >= 10)
  {
    INS.History.ZCount = 0;
  }
}

void InertialNavigationClass::ResetZState(void)
{
  INS.EarthFrame.Position[INS_VERTICAL_Z] = 0.0f;
  INS.EarthFrame.Velocity[INS_VERTICAL_Z] = 0.0f;
  INS.History.ZCount = 0;
  for (uint8_t IndexCount = 0; IndexCount < 10; IndexCount++)
  {
    INS.History.ZPosition[IndexCount] = 0;
  }
}
