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
#include "Scheduler/SCHEDULER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "RadioControl/CURVESRC.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "RadioControl/RCSTATES.h"
#include "RadioControl/DECODE.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "InertialNavigation/INS.h"
#include "PID/PIDPARAMS.h"
#include "Barometer/BAROBACKEND.h"

bool TakeOffInProgress = false;
bool GroundAltitudeSet = false;

uint8_t MinVariometer = 50;
uint8_t SafeZoneToCompleteTakeOff = 70;
uint8_t SafeAltitude = 5;
int16_t ThrottleMiddleValue = 1500;
int16_t HoveringThrottle = 0;
int16_t VariometerErrorIPart = 0;

uint32_t LandDetectorStartTime;
uint32_t TimeOnLand;

int32_t AltitudeToHold = 0;
int32_t TargetVariometer = 0;
int32_t VariometerErrorISum = 0;

void AltitudeHold_Update_Params()
{
  MinVariometer = STORAGEMANAGER.Read_8Bits(AH_MIN_VEL_VERT_ADDR);
  SafeZoneToCompleteTakeOff = STORAGEMANAGER.Read_8Bits(AH_DEADZONE_ADDR);
  SafeAltitude = STORAGEMANAGER.Read_8Bits(AH_SAFE_ALT_ADDR);
  ThrottleMiddleValue = RCLookupThrottleMiddle();
}

bool ApplyAltitudeHoldControl()
{
  static Scheduler_Struct AltitudeHoldControlTimer;
  if (Scheduler(&AltitudeHoldControlTimer, SCHEDULER_SET_FREQUENCY(50, "Hz")))
  {
    static bool BaroModeActivated = false;
    static bool HoveringState = false;
    if ((Do_Altitude_Hold || Do_RTH_Or_Land_Call_Alt_Hold) && IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
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
      if (Do_RTH_Or_Land_Call_Alt_Hold)
      {
        if (TakeOffInProgress)
        {
          TakeOffInProgress = false;
        }
        if (GPS_Parameters.Mode.Navigation == DO_LAND_IN_PROGRESS || GPS_Parameters.Mode.Navigation == DO_LAND_DETECTED || GPS_Parameters.Mode.Navigation == DO_LANDED)
        {
          if (HoveringState)
          {
            HoveringState = false;
          }
          SetAltitudeToHold(Barometer.INS.Altitude.Estimated);
          TargetVariometer = Constrain_8Bits(MinVariometer, 30, 100);
          if (Barometer.INS.Altitude.Estimated > ConvertCMToMeters(SafeAltitude))
          {
            TargetVariometer += (int32_t)(250 - MinVariometer) * (Barometer.INS.Altitude.Estimated - ConvertCMToMeters(SafeAltitude)) / (ConvertCMToMeters(GPS_Parameters.Home.Altitude) - ConvertCMToMeters(SafeAltitude));
          }
          TargetVariometer = -TargetVariometer;
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
          }
          TargetVariometer = ((AltitudeToHold - Barometer.INS.Altitude.Estimated) * 3) / 2;
          if (Barometer.INS.Altitude.Estimated > ConvertCMToMeters(SafeAltitude))
          {
            TargetVariometer = Constrain_32Bits(TargetVariometer, -250, 250);
          }
          else
          {
            TargetVariometer = Constrain_32Bits(TargetVariometer, -83, 83);
          }
        }
      }
      else
      {
        int16_t ThrottleDifference = DECODE.GetRxChannelOutput(THROTTLE) - ThrottleMiddleValue;
        if (!TakeOffInProgress)
        {
          if (GetActualThrottleStatus(THROTTLE_LOW))
          {
            if (GetGroundDetectedFor100ms())
            {
              TakeOffInProgress = true;
            }
          }
        }
        else
        {
          if ((ThrottleDifference > SafeZoneToCompleteTakeOff) && (Barometer.INS.Velocity.Vertical >= 15))
          {
            TakeOffInProgress = false;
          }
        }
        if (TakeOffInProgress || (ABS(ThrottleDifference) > SafeZoneToCompleteTakeOff))
        {
          if (HoveringState)
          {
            HoveringState = false;
          }
          if (ABS(ThrottleDifference) <= SafeZoneToCompleteTakeOff)
          {
            TargetVariometer = 0;
          }
          else
          {
            TargetVariometer = ((ThrottleDifference - ((ThrottleDifference > 0) ? SafeZoneToCompleteTakeOff : -SafeZoneToCompleteTakeOff)) * 3) / 4;
          }
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
            AltitudeToHold = Barometer.INS.Altitude.Estimated;
          }
          TargetVariometer = ((AltitudeToHold - Barometer.INS.Altitude.Estimated) * 3) / 2;
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

//#define THR_SMOOTH_TEST

#ifdef THR_SMOOTH_TEST

#include "FastSerial/PRINTF.h"
#include "Build/BOARDDEFS.h"
#include "Filters/PT1.h"

PT1_Filter_Struct Smooth_ThrottleHover;

#define ALT_HOLD_LPF_CUTOFF 4 //HZ

#endif

void ApplyAltitudeHoldPIDControl(uint16_t DeltaTime, bool HoveringState)
{
  TargetVariometer = Constrain_32Bits(TargetVariometer, -350, 350);
  int32_t VariometerError = TargetVariometer - Barometer.INS.Velocity.Vertical;
  VariometerError = Constrain_32Bits(VariometerError, -600, 600);
  VariometerErrorISum += ((VariometerError * GET_SET[PID_ALTITUDE].IntegralVector * DeltaTime) >> 7) / ((HoveringState && ABS(TargetVariometer) < 100) ? 2 : 1);
  VariometerErrorISum = Constrain_32Bits(VariometerErrorISum, -16384000, 16384000);
  VariometerErrorIPart = (VariometerErrorISum >> 16);
  VariometerErrorIPart = Constrain_16Bits(VariometerErrorIPart, -250, 250);
  int16_t VarioPIDControl = ((VariometerError * GET_SET[PID_ALTITUDE].ProportionalVector) >> 5) + VariometerErrorIPart -
                            (((int32_t)INS.AccelerationEarthFrame_Filtered[2] * GET_SET[PID_ALTITUDE].DerivativeVector) >> 6);

  if (GetFrameStateOfMultirotor())
  {
    RCController[THROTTLE] = Constrain_16Bits(HoveringThrottle + VarioPIDControl, AttitudeThrottleMin + 50, AttitudeThrottleMax - 50);
  }

#ifdef THR_SMOOTH_TEST

  if (GetFrameStateOfMultirotor())
  {
    PRINTF.SendToConsole(ProgramMemoryString("RCController[THROTTLE]:%d PT1RCController[THROTTLE]:%d\n"),
                         RCController[THROTTLE],
                         (int16_t)PT1FilterApply(&Smooth_ThrottleHover, RCController[THROTTLE], ALT_HOLD_LPF_CUTOFF, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY) * 1e-6f));
  }

#endif
}

void ResetIntegralOfVariometerError()
{
  VariometerErrorISum = Constrain_32Bits(VariometerErrorISum, -8192000, 8192000);
  VariometerErrorIPart = Constrain_16Bits(VariometerErrorIPart, -125, 125);
}

void InitializeHoveringThrottle()
{
  if ((HoveringThrottle < (ThrottleMiddleValue - 250)) || (HoveringThrottle > (ThrottleMiddleValue + 250)))
  {
    HoveringThrottle = ThrottleMiddleValue;
    HoveringThrottle = Constrain_16Bits(HoveringThrottle, ThrottleMiddleValue - 250, ThrottleMiddleValue + 250);
  }
}

void SetAltitudeToHold(int32_t ValueOfNewAltitudeHold)
{
  if (ValueOfNewAltitudeHold > 15000)
  {
    ValueOfNewAltitudeHold = 15000;
  }
  AltitudeToHold = ValueOfNewAltitudeHold;
}

bool GetAltitudeReached()
{
  return ABS(AltitudeToHold - Barometer.INS.Altitude.Estimated) < 50;
}

void RunLandDetector()
{
  if (GetGroundDetected())
  {
    TimeOnLand = SCHEDULERTIME.GetMillis() - LandDetectorStartTime;
  }
  else
  {
    ResetLandDetector();
  }
  if (!GroundAltitudeSet && (TimeOnLand >= 100))
  {
    GroundAltitudeSet = true;
    Barometer.Altitude.GroundOffSet = Barometer.Altitude.Actual;
  }
}

void ResetLandDetector()
{
  LandDetectorStartTime = SCHEDULERTIME.GetMillis();
  TimeOnLand = 0;
  GroundAltitudeSet = false;
}

bool GetGroundDetected()
{
  return (ABS(Barometer.INS.Velocity.Vertical) < 15) && (VariometerErrorIPart <= -185) && (Barometer.INS.Altitude.Estimated < ConvertCMToMeters(SafeAltitude));
}

bool GetGroundDetectedFor100ms()
{
  return GroundAltitudeSet;
}

bool GetLanded()
{
  return GroundAltitudeSet && (TimeOnLand >= 4000);
}
