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

#include "ALTITUDEHOLD.h"
#include "Common/VARIABLES.h"
#include "Scheduler/SCHEDULER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"

bool TakeOffInProgress = false;
int16_t HoveringThrottle = 0;
int32_t AltitudeToHold;
int32_t TargetVariometer = 0;

bool ApplyAltitudeHoldControl()
{
  static Scheduler_Struct AltitudeHoldControlTimer;
  if (SchedulerTimer(&AltitudeHoldControlTimer, 20000))
  {
    static bool BaroModeActivated = false;
    static bool HoveringState = false;
    if ((Do_AltitudeHold_Mode || Do_GPS_Altitude) && COMMAND_ARM_DISARM)
    {
      if (!BaroModeActivated)
      {
        BaroModeActivated = true;
        HoveringState = false;
        TakeOffInProgress = false;
        InitializeHoveringThrottle();
        ResetLandDetector();
      }
      RunLandDetector();
      if (Do_GPS_Altitude)
      {
        if (TakeOffInProgress)
          TakeOffInProgress = false;
        if (NavigationMode == Do_LandInProgress || NavigationMode == Do_Land_Detected || NavigationMode == Do_Landed)
        {
          if (HoveringState)
            HoveringState = false;
          SetAltitudeHold(ALTITUDE.EstimateAltitude);
          TargetVariometer = 50;
          if (ALTITUDE.EstimateAltitude > 500)
            TargetVariometer += (int32_t)200 * (ALTITUDE.EstimateAltitude - 500) / (RTH_Altitude * 100 - 500);
          TargetVariometer = -TargetVariometer;
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
          }
          TargetVariometer = ((AltitudeToHold - ALTITUDE.EstimateAltitude) * 3) / 2;
          if (ALTITUDE.EstimateAltitude > 500)
            TargetVariometer = Constrain_32Bits(TargetVariometer, -250, 250);
          else
            TargetVariometer = Constrain_32Bits(TargetVariometer, -83, 83);
        }
      }
      else
      {
        int16_t ThrottleDifference = RadioControllOutput[THROTTLE] - 1500;
        if (!TakeOffInProgress)
        {
          if ((RadioControllOutput[THROTTLE] < 1100))
          {
            if (GetGroundDetectedFor100ms())
            {
              TakeOffInProgress = true;
            }
          }
        }
        else
        {
          if ((ThrottleDifference > 70) && (ALTITUDE.EstimateVariometer >= 15))
            TakeOffInProgress = false;
        }
        if (TakeOffInProgress || (ABS_16BITS(ThrottleDifference) > 70))
        {
          if (HoveringState)
            HoveringState = false;
          if (ABS_16BITS(ThrottleDifference) <= 70)
            TargetVariometer = 0;
          else
            TargetVariometer = ((ThrottleDifference - ((ThrottleDifference > 0) ? 70 : -70)) * 3) / 4;
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
            AltitudeToHold = ALTITUDE.EstimateAltitude;
          }
          TargetVariometer = ((AltitudeToHold - ALTITUDE.EstimateAltitude) * 3) / 2;
        }
      }
      ApplyAltitudeHoldPIDControl(AltitudeHoldControlTimer.ActualTime, HoveringState);
      return true;
    }
    else
    {
      if (BaroModeActivated)
      {
        BaroModeActivated = false;
        TakeOffInProgress = false;
        TargetVariometer = 0;
        ResetIntegralOfVariometerError();
        ResetLandDetector();
      }
    }
  }
  return false;
}

bool GetTakeOffInProgress()
{
  return TakeOffInProgress;
}

int32_t VariometerErrorISum = 0;
int16_t VariometerErrorIPart = 0;

void ApplyAltitudeHoldPIDControl(uint16_t DeltaTime, bool HoveringState)
{
  TargetVariometer = Constrain_32Bits(TargetVariometer, -350, 350);
  int32_t VariometerError = TargetVariometer - ALTITUDE.EstimateVariometer;
  VariometerError = Constrain_32Bits(VariometerError, -600, 600);
  VariometerErrorISum += ((VariometerError * PID[PIDALTITUDE].IntegratorVector * DeltaTime) >> 7) / ((HoveringState && ABS_32BITS(TargetVariometer) < 100) ? 2 : 1);
  VariometerErrorISum = Constrain_32Bits(VariometerErrorISum, -16384000, 16384000);
  VariometerErrorIPart = (VariometerErrorISum >> 16);
  VariometerErrorIPart = Constrain_16Bits(VariometerErrorIPart, -250, 250);
  int16_t VarioPIDControl = ((VariometerError * PID[PIDALTITUDE].ProportionalVector) >> 5) + VariometerErrorIPart -
                            (((int32_t)INS.AccelerationEarthFrame_Filtered[2] * PID[PIDALTITUDE].DerivativeVector) >> 6);
  RCController[THROTTLE] = HoveringThrottle + VarioPIDControl;
  RCController[THROTTLE] = Constrain_16Bits(RCController[THROTTLE], MotorSpeed + 50, 1900 - 50);
}

void ResetIntegralOfVariometerError()
{
  VariometerErrorISum = Constrain_32Bits(VariometerErrorISum, -16384000 / 2, 16384000 / 2);
  VariometerErrorIPart = Constrain_16Bits(VariometerErrorIPart, -250 / 2, 250 / 2);
}

void InitializeHoveringThrottle()
{
  if ((HoveringThrottle < 1250) || (HoveringThrottle > 1750))
  {
    HoveringThrottle = 1500;
    HoveringThrottle = Constrain_16Bits(HoveringThrottle, 1250, 1750);
  }
}

void SetAltitudeHold(int32_t ValueOfNewAltitudeHold)
{
  if (ValueOfNewAltitudeHold > 15000)
    ValueOfNewAltitudeHold = 15000;
  AltitudeToHold = ValueOfNewAltitudeHold;
}

bool GetAltitudeReached()
{
  return ABS_32BITS(AltitudeToHold - ALTITUDE.EstimateAltitude) < 50;
}

bool GroundAltitudeSet = false;
uint32_t LandDetectorStartTime;
uint32_t TimeOnLand;

void RunLandDetector()
{
  if (GetGroundDetected())
    TimeOnLand = AVRTIME.SchedulerMillis() - LandDetectorStartTime;
  else
    ResetLandDetector();
  if (!GroundAltitudeSet && (TimeOnLand >= 100))
  {
    GroundAltitudeSet = true;
    ALTITUDE.GroundAltitude = ALTITUDE.RealBaroAltitude;
  }
}

void ResetLandDetector()
{
  LandDetectorStartTime = AVRTIME.SchedulerMillis();
  TimeOnLand = 0;
  GroundAltitudeSet = false;
}

bool GetGroundDetected()
{
  return (ABS_16BITS(ALTITUDE.EstimateVariometer) < 15) && (VariometerErrorIPart <= -185) && (ALTITUDE.EstimateAltitude < 500);
}

bool GetGroundDetectedFor100ms()
{
  return GroundAltitudeSet;
}

bool GetLanded()
{
  return GroundAltitudeSet && (TimeOnLand >= 4000);
}
