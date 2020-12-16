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
#include "Math/AVRMATH.h"

uint8_t HistoryZCount;
int32_t HistoryZPosition[10];

void INS_Calculate_AccelerationZ()
{
  static Scheduler_Struct INS_Calculate_AccelerationZTimer;
  if (SchedulerTimer(&INS_Calculate_AccelerationZTimer, 20000))
  {
    static bool Activated = false;
    CalculateBaroAltitude();
    UpdateAccelerationEarthFrame_Filtered(2);
    if (COMMAND_ARM_DISARM)
    {
      if (!Activated)
      {
        Activated = true;
        ResetZState();
      }
      float DeltaTime = INS_Calculate_AccelerationZTimer.ActualTime * 1e-6f;
      CorrectZStateWithBaro(&DeltaTime);
      UpdateZState(&DeltaTime);
      saveZPositionToHistory();
      return;
    }
    else
    {
      if (Activated)
        Activated = false;
    }
  }
}

void CorrectZStateWithBaro(float *DeltaTime)
{
  bool AirCushionEffectDetected = (GetTakeOffInProgress() || GetGroundDetected()) && (ALTITUDE.RealBaroAltitude < ALTITUDE.GroundAltitude);
  float PositionError = (AirCushionEffectDetected ? ALTITUDE.GroundAltitude : ALTITUDE.RealBaroAltitude) - HistoryZPosition[HistoryZCount];
  INS.Velocity_EarthFrame[2] += PositionError * (0.19753086419753086419753086419753f * *DeltaTime);
  INS.Position_EarthFrame[2] += PositionError * (0.66666666666666666666666666666667f * *DeltaTime);
}

void UpdateZState(float *DeltaTime)
{
  float VelocityIncrease = INS.AccelerationEarthFrame_Filtered[2] * *DeltaTime;
  INS.Position_EarthFrame[2] += (INS.Velocity_EarthFrame[2] + VelocityIncrease * 0.5) * *DeltaTime;
  ALTITUDE.EstimateAltitude = INS.Position_EarthFrame[2];
  INS.Velocity_EarthFrame[2] += VelocityIncrease;
  ALTITUDE.EstimateVariometer = INS.Velocity_EarthFrame[2];
}

void saveZPositionToHistory()
{
  HistoryZPosition[HistoryZCount] = ALTITUDE.EstimateAltitude;
  HistoryZCount++;
  if (HistoryZCount >= 10)
    HistoryZCount = 0;
}

void ResetZState()
{
  INS.Position_EarthFrame[2] = 0.0f;
  INS.Velocity_EarthFrame[2] = 0.0f;
  HistoryZCount = 0;
  for (uint8_t i = 0; i < 10; i++)
    HistoryZPosition[i] = 0;
}

uint8_t HistoryXYCount;
int32_t HistoryXYPosition[2][10];

void CalculateXY_INS()
{
  static Scheduler_Struct CalculateXY_INSTimer;
  if (SchedulerTimer(&CalculateXY_INSTimer, 20000))
  {
    static bool Activated = false;
    UpdateAccelerationEarthFrame_Filtered(0);
    UpdateAccelerationEarthFrame_Filtered(1);
    if (COMMAND_ARM_DISARM && GPS_3DFIX && (GPS_NumberOfSatellites >= 4) && Home_Point)
    {
      if (!Activated)
      {
        Activated = true;
        ResetXYState();
      }
      float DeltaTime = CalculateXY_INSTimer.ActualTime * 1e-6f;
      CorrectXYStateWithGPS(&DeltaTime);
      updateXYState(&DeltaTime);
      SaveXYPositionToHistory();
      return;
    }
    else
    {
      if (Activated)
        Activated = false;
    }
  }
}

void CorrectXYStateWithGPS(float *DeltaTime)
{
  float PositionError[2];
  PositionError[0] = GPSDistanceToHome[0] - HistoryXYPosition[0][HistoryXYCount];
  PositionError[0] = Constrain_Float(PositionError[0], -1000, 1000);
  PositionError[1] = GPSDistanceToHome[1] - HistoryXYPosition[1][HistoryXYCount];
  PositionError[1] = Constrain_Float(PositionError[1], -1000, 1000);
  float StoredDeltaTime;
  StoredDeltaTime = 0.48f * *DeltaTime;
  INS.Velocity_EarthFrame[0] += PositionError[0] * StoredDeltaTime;
  INS.Velocity_EarthFrame[1] += PositionError[1] * StoredDeltaTime;
  StoredDeltaTime = 1.2f * *DeltaTime;
  INS.Position_EarthFrame[0] += PositionError[0] * StoredDeltaTime;
  INS.Position_EarthFrame[1] += PositionError[1] * StoredDeltaTime;
}

void updateXYState(float *DeltaTime)
{
  float VelocityIncrease[2];
  VelocityIncrease[0] = INS.AccelerationEarthFrame_Filtered[0] * *DeltaTime;
  VelocityIncrease[1] = INS.AccelerationEarthFrame_Filtered[1] * *DeltaTime;
  INS.Position_EarthFrame[0] += (INS.Velocity_EarthFrame[0] + VelocityIncrease[0] * 0.5) * *DeltaTime;
  INS.Position_EarthFrame[1] += (INS.Velocity_EarthFrame[1] + VelocityIncrease[1] * 0.5) * *DeltaTime;
  INS.Velocity_EarthFrame[0] += VelocityIncrease[0];
  INS.Velocity_EarthFrame[1] += VelocityIncrease[1];
}

void SaveXYPositionToHistory()
{
  HistoryXYPosition[0][HistoryXYCount] = INS.Position_EarthFrame[0];
  HistoryXYPosition[1][HistoryXYCount] = INS.Position_EarthFrame[1];
  HistoryXYCount++;
  if (HistoryXYCount >= 10)
    HistoryXYCount = 0;
}

void ResetXYState()
{
  HistoryXYCount = 0;
  for (uint8_t i = 0; i < 2; i++)
  {
    INS.Velocity_EarthFrame[i] = (ABS_16BITS(GPSActualSpeed[i]) > 50) ? GPSActualSpeed[i] : 0.0f;
    INS.Position_EarthFrame[i] = GPSDistanceToHome[i];
    for (uint8_t j = 0; j < 10; j++)
      HistoryXYPosition[i][j] = GPSDistanceToHome[i];
  }
}

void UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount)
{
  INS.AccelerationEarthFrame_Filtered[ArrayCount] = INS.AccelerationEarthFrame_Sum[ArrayCount] / INS.AccelerationEarthFrame_Sum_Count[ArrayCount];
  INS.AccelerationEarthFrame_Sum[ArrayCount] = 0.0f;
  INS.AccelerationEarthFrame_Sum_Count[ArrayCount] = 0;
}
