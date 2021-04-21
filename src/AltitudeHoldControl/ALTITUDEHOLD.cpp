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
#include "RadioControl/RCSTATES.h"
#include "RadioControl/DECODE.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PID/RCPID.h"
#include "FlightModes/FLIGHTMODES.h"
#include "InertialNavigation/INS.h"
#include "PID/PIDPARAMS.h"
#include "Barometer/BAROBACKEND.h"
#include "GPS/GPSSTATES.h"

AH_Controller_Struct AHController;

#define ALT_HOLD_DEADBAND 50         //GANHO PARA EVITAR BAIXO OU ALTO VALOR DE THROTTLE
#define SAFE_ALTITUDE 5              //ALTITUDE SEGURA PARA O LAND E TAKE-OFF EM METROS
#define MIN_TARGET_POS_Z 50          //GANHO DE ALTITUDE DURANTE O LAND E ACIMA DA ALTITUDE DADA PELO PARAM "SAFE_ALTITUDE" EM CM DE 30 ~ 100
#define VALID_ALT_REACHED 50         //RESULTADO DA SUBTRAÇÃO ENTRE A ALTITUDE MARCADA COMO HOLD E A ALTIUDE ATUAL PARA VALIDAR QUE ALTITUDE FOI ALCANÇADA EM CM
#define MIN_VEL_Z_TO_VALID_GROUND 15 //VELOCIDADE VERTICAL MINIMA PARA INDICAR QUE A VEL Z DO INS ESTÁ EM REPOUSO
#define LANDED_TIME 4000             //ESTOURO DE TEMPO EM MS PARA INDICAR QUE REALMENTE O SOLO FOI DETECTADO

//-------------------------------------
//PARAMS DO USUARIO
uint8_t AH_Percent_Complete_TakeOff = 70;
int16_t AH_Hover_Throttle = 1500;
//-------------------------------------

static void ResetIntegralOfVelZError()
{
  AHController.PID.IntegratorSum = Constrain_32Bits(AHController.PID.IntegratorSum, -8192000, 8192000);
  AHController.PID.IntegratorError = Constrain_16Bits(AHController.PID.IntegratorError, -125, 125);
}

static void ResetLandDetector()
{
  AHController.Time.LandDetectorStart = SCHEDULERTIME.GetMillis();
  AHController.Time.OnLand = 0;
  AHController.Flags.GroundAltitudeSet = false;
}

//#define THR_SMOOTH_TEST

#ifdef THR_SMOOTH_TEST

#include "FastSerial/PRINTF.h"
#include "Filters/PT1.h"

PT1_Filter_Struct Smooth_ThrottleHover;

#define ALT_HOLD_LPF_CUTOFF 4 //HZ

#endif

static void ApplyAltitudeHoldPIDControl(uint16_t DeltaTime, bool HoveringState)
{
  AHController.Target.Position.Z = Constrain_32Bits(AHController.Target.Position.Z, -350, 350);
  AHController.Target.Velocity.Z = Constrain_32Bits(AHController.Target.Position.Z - Barometer.INS.Velocity.Vertical, -600, 600);
  AHController.PID.IntegratorSum += Constrain_32Bits(((AHController.Target.Velocity.Z * GET_SET[PID_VELOCITY_Z].kI * DeltaTime) / 128) / ((HoveringState && ABS(AHController.Target.Position.Z) < 100) ? 2 : 1), -16384000, 16384000);
  AHController.PID.IntegratorError = Constrain_16Bits((AHController.PID.IntegratorSum / UINT16_MAX), -250, 250);
  AHController.PID.Control = ((AHController.Target.Velocity.Z * GET_SET[PID_VELOCITY_Z].kP) / 32) + AHController.PID.IntegratorError - (((int32_t)INS.AccelerationEarthFrame_Filtered[INS_VERTICAL_Z] * GET_SET[PID_VELOCITY_Z].kD) / 64);

  RCController[THROTTLE] = Constrain_16Bits(AHController.Throttle.Hovering + AHController.PID.Control, AttitudeThrottleMin + ALT_HOLD_DEADBAND, AttitudeThrottleMax - ALT_HOLD_DEADBAND);

#ifdef THR_SMOOTH_TEST

  RCController[THROTTLE] = (int16_t)PT1FilterApply(&Smooth_ThrottleHover, RCController[THROTTLE], ALT_HOLD_LPF_CUTOFF, DeltaTime * 1e-6f);

  DEBUG("RCController[THROTTLE]:%d", RCController[THROTTLE]);

#endif
}

static void RunLandDetector()
{
  if (GetGroundDetected())
  {
    AHController.Time.OnLand = SCHEDULERTIME.GetMillis() - AHController.Time.LandDetectorStart;
  }
  else
  {
    ResetLandDetector();
  }
  if (!AHController.Flags.GroundAltitudeSet && (AHController.Time.OnLand >= 100))
  {
    AHController.Flags.GroundAltitudeSet = true;
    Barometer.Altitude.GroundOffSet = Barometer.Altitude.Actual;
  }
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
        AHController.Flags.TakeOffInProgress = false;
        if ((AHController.Throttle.Hovering < (MIDDLE_STICKS_PULSE - 250)) ||
            (AHController.Throttle.Hovering > (MIDDLE_STICKS_PULSE + 250)))
        {
          AHController.Throttle.Hovering = Constrain_16Bits(AH_Hover_Throttle, MIDDLE_STICKS_PULSE - 250, MIDDLE_STICKS_PULSE + 250);
        }
        ResetLandDetector();
      }
      RunLandDetector();
      if (Do_RTH_Or_Land_Call_Alt_Hold)
      {
        if (AHController.Flags.TakeOffInProgress)
        {
          AHController.Flags.TakeOffInProgress = false;
        }
        if (Get_GPS_Used_To_Land())
        {
          if (HoveringState)
          {
            HoveringState = false;
          }
          SetNewAltitudeToHold(Barometer.INS.Altitude.Estimated);
          if (Barometer.INS.Altitude.Estimated > ConvertCMToMeters(SAFE_ALTITUDE))
          {
            AHController.Target.Position.Z = MIN_TARGET_POS_Z + ((int32_t)(250 - MIN_TARGET_POS_Z) * (Barometer.INS.Altitude.Estimated - ConvertCMToMeters(SAFE_ALTITUDE)) / (ConvertCMToMeters(GPSParameters.Home.Altitude) - ConvertCMToMeters(SAFE_ALTITUDE)));
          }
          AHController.Target.Position.Z = -AHController.Target.Position.Z;
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
          }
          AHController.Target.Position.Z = ((AHController.Target.Altitude - Barometer.INS.Altitude.Estimated) * GET_SET[PID_ALTITUDE_HOLD].kP) / 2;
          if (Barometer.INS.Altitude.Estimated > ConvertCMToMeters(SAFE_ALTITUDE))
          {
            AHController.Target.Position.Z = Constrain_32Bits(AHController.Target.Position.Z, -250, 250);
          }
          else
          {
            AHController.Target.Position.Z = Constrain_32Bits(AHController.Target.Position.Z, -83, 83);
          }
        }
      }
      else
      {
        AHController.Throttle.Difference = DECODE.GetRxChannelOutput(THROTTLE) - MIDDLE_STICKS_PULSE;
        if (!AHController.Flags.TakeOffInProgress)
        {
          if (GetActualThrottleStatus(THROTTLE_LOW))
          {
            if (GetGroundDetectedFor100ms())
            {
              AHController.Flags.TakeOffInProgress = true;
            }
          }
        }
        else
        {
          if ((AHController.Throttle.Difference > AH_Percent_Complete_TakeOff) && (Barometer.INS.Velocity.Vertical >= 15))
          {
            AHController.Flags.TakeOffInProgress = false;
          }
        }
        if (AHController.Flags.TakeOffInProgress || (ABS(AHController.Throttle.Difference) > AH_Percent_Complete_TakeOff))
        {
          if (HoveringState)
          {
            HoveringState = false;
          }
          if (ABS(AHController.Throttle.Difference) <= AH_Percent_Complete_TakeOff)
          {
            AHController.Target.Position.Z = 0;
          }
          else
          {
            AHController.Target.Position.Z = ((AHController.Throttle.Difference - ((AHController.Throttle.Difference > 0) ? AH_Percent_Complete_TakeOff : -AH_Percent_Complete_TakeOff)) * GET_SET[PID_ALTITUDE_HOLD].kP) / 4;
          }
        }
        else
        {
          if (!HoveringState)
          {
            HoveringState = true;
            AHController.Target.Altitude = Barometer.INS.Altitude.Estimated;
          }
          AHController.Target.Position.Z = ((AHController.Target.Altitude - Barometer.INS.Altitude.Estimated) * GET_SET[PID_ALTITUDE_HOLD].kP) / 2;
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
        AHController.Flags.TakeOffInProgress = false;
        AHController.Target.Position.Z = 0;
        ResetIntegralOfVelZError();
        ResetLandDetector();
      }
    }
  }
  return false;
}

void SetNewAltitudeToHold(int32_t ValueOfNewAltitudeHold)
{
  if (ValueOfNewAltitudeHold > 15000)
  {
    ValueOfNewAltitudeHold = 15000;
  }
  AHController.Target.Altitude = ValueOfNewAltitudeHold;
}

bool GetTakeOffInProgress()
{
  return AHController.Flags.TakeOffInProgress;
}

bool GetAltitudeReached()
{
  return ABS(AHController.Target.Altitude - Barometer.INS.Altitude.Estimated) < VALID_ALT_REACHED;
}

bool GetGroundDetected()
{
  return (ABS(Barometer.INS.Velocity.Vertical) < MIN_VEL_Z_TO_VALID_GROUND) && (AHController.PID.IntegratorError <= -185) && (Barometer.INS.Altitude.Estimated < ConvertCMToMeters(SAFE_ALTITUDE));
}

bool GetGroundDetectedFor100ms()
{
  return AHController.Flags.GroundAltitudeSet;
}

bool GetLanded()
{
  return AHController.Flags.GroundAltitudeSet && (AHController.Time.OnLand >= LANDED_TIME);
}