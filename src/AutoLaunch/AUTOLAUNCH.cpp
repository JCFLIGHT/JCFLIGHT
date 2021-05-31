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

#include "AUTOLAUNCH.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "AHRS/VECTOR.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "PID/RCPID.h"
#include "Barometer/BAROBACKEND.h"
#include "GPS/GPSUBLOX.h"
#include "GPS/GPSSTATES.h"
#include "Param/PARAM.h"
#include "I2C/I2C.h"
#include "GPSNavigation/NAVIGATION.h"
#include "RadioControl/DECODE.h"
#include "BitArray/BITARRAY.h"

AutoLaunchClass AUTOLAUNCH;

#define AHRS_BANKED_ANGLE 25                                 //25 GRAUS MAXIMO DE BANK ANGLE (CONSIDERANDO EM RADIANOS = 436)
#define IMU_BANKED_ANGLE -450.0f                             //-45 GRAUS DE INCLINAÇÃO NA IMU
#define LAUNCH_MOTOR_IDLE_SPINUP_TIME 1500                   //ARMA O MOTOR DEPOIS DE 1.5 SEGUNDO APÓS DETECTAR O AUTO-LAUNCH
#define AUTO_LAUNCH_ANGLE 18                                 //VALOR DO PITCH (ELEVATOR) AO FAZER O AUTO-LAUNCH (VALOR EM GRAUS)
#define SWING_LAUNCH_MIN_ROTATION_RATE ConvertToRadians(100) //NO MINIMO UM RATE DE 100DPS NO GYRO
#define LAUNCH_VELOCITY_THRESH 3                             //METROS/S
#define MOTOR_SPINUP_VALUE 100                               //VALOR DA INCREMENTAÇÃO DO THROTTLE PARA PLANES COM RODAS
#define MOTOR_SPINUP_TIME 300                                //VAI SUBINDO O THROTTLE AOS POUCOS,BOM PARA AERO COM RODAS (TEMPO EM MS)
#define AUTO_LAUCH_EXIT_FUNCTION 5000                        //TEMPO DE PARA SAIR DO MODO AUTO-LAUCH APÓS A DETECÇÃO (TEMPO EM MS)
#define AUTO_LAUNCH_THROTTLE_MAX 1700                        //VALOR MAXIMO DE ACELERAÇÃO
#define AUTO_LAUCH_MAX_ALTITUDE 0                            //ALTITUDE MAXIMA PARA VALIDAR O AUTO-LAUNCH (VALOR EM METROS)

bool AutoLaunchState = false;
bool StateLaunched = false;
bool LaunchedDetect = false;
bool IgnoreFirstPeak = false;
bool IgnoreFirstPeakOverFlow = false;
uint16_t ThrottleIteration = 1000;
uint32_t ThrottleStart = 0;
uint32_t AutoLaunchDetectorPreviousTime = 0;
uint32_t AutoLaunchDetectorSum = 0;
uint32_t AbortAutoLaunch = 0;

const float GetPitchAccelerationInMSS(void)
{
  return BodyFrameAcceleration.Pitch;
}

const float GetRollAccelerationInMSS(void)
{
  return BodyFrameAcceleration.Roll;
}

const float GetYawRotationInRadians(void)
{
  return BodyFrameRotation.Yaw;
}

bool AutoLaunchClass::GetSwingVelocityState(void)
{
  const float SwingVelocity = (ABS(GetYawRotationInRadians()) > SWING_LAUNCH_MIN_ROTATION_RATE) ? (GetRollAccelerationInMSS() / GetYawRotationInRadians()) : 0;
  return (SwingVelocity > ConverMetersToCM(LAUNCH_VELOCITY_THRESH)) && (GetPitchAccelerationInMSS() > 0);
}

bool AutoLaunchClass::GetForwardState(void)
{
  return Get_GPS_Heading_Is_Valid() && (GetRollAccelerationInMSS() > 0) && (GPS_Resources.Navigation.Misc.Get.GroundSpeed > ConverMetersToCM(LAUNCH_VELOCITY_THRESH));
}

void AutoLaunchClass::AutomaticDetector()
{
  if (AUTOLAUNCH.GetIMUAngleBanked(GetPitchAccelerationInMSS(), AHRS.CheckAnglesInclination(AHRS_BANKED_ANGLE)) ||
      AUTOLAUNCH.GetSwingVelocityState() ||
      AUTOLAUNCH.GetForwardState())
  {
    AutoLaunchDetectorSum += (SCHEDULERTIME.GetMillis() - AutoLaunchDetectorPreviousTime);
    AutoLaunchDetectorPreviousTime = SCHEDULERTIME.GetMillis();
    if (AutoLaunchDetectorSum >= 40)
    {
      AutoLaunchState = true;
    }
  }
  else
  {
    AutoLaunchDetectorPreviousTime = SCHEDULERTIME.GetMillis();
    AutoLaunchDetectorSum = 0;
  }
}

void AutoLaunchClass::Update(void)
{
  if (!GetAirPlaneEnabled())
  {
    return;
  }
  if (IS_FLIGHT_MODE_ACTIVE(LAUNCH_MODE))
  {
    if (AUTOLAUNCH.GetValidStateToRunLaunch() && !LaunchedDetect)
    {
      if (!AutoLaunchState)
      {
        BEEPER.Play(BEEPER_AUTO_LAUNCH);
      }
      else
      {
        BEEPER.Play(BEEPER_LAUNCHED);
      }
      if (AUTOLAUNCH.GetPlaneType() == WITH_WHEELS)
      {
        AutoLaunchState = true;
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          ENABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        }
        AUTOLAUNCH.RCControllerThrottle_Apply_Logic(true); //TRUE PARA PLANES COM TREM DE POUSO
      }
      if (AutoLaunchState)
      {
        if (AUTOLAUNCH.GetPlaneType() == WITHOUT_WHEELS)
        {
          AUTOLAUNCH.RCControllerThrottle_Apply_Logic(false); //FALSE PARA PLANES SEM TREM DE POUSO
        }
        AUTOLAUNCH.RCControllerYawPitchRoll_Apply_Logic();
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
          ENABLE_THIS_STATE(PRIMARY_ARM_DISARM);
        }
        if (AUTOLAUNCH.GetStatusCompleted())
        {
          LaunchedDetect = true;
          AutoLaunchState = false;
        }
        StateLaunched = true;
      }
      else if (!StateLaunched && AUTOLAUNCH.GetPlaneType() == WITHOUT_WHEELS) //NÃO VAMOS USAR A IMU PARA ATIVAR O AUTO-LAUCH EM AEROMODELOS COM RODAS
      {
        AUTOLAUNCH.AutomaticDetector();
      }
      if (!LaunchedDetect)
      {
        AUTOLAUNCH.RCControllerYawPitchRoll_Apply_Logic(); //A FUNÇÃO FICA SEMPRE ATIVA PARA PLANES SEM TREM DE POUSO
      }
    }
  }
  if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && !AUTOLAUNCH.GetValidStateToRunLaunch())
  {
    AUTOLAUNCH.ResetParameters();
  }
}

void AutoLaunchClass::RCControllerThrottle_Apply_Logic(bool SlowThr)
{
  if (SlowThr)
  {
    RC_Resources.Attitude.Controller[THROTTLE] = ThrottleIteration;
    if (SCHEDULERTIME.GetMillis() - ThrottleStart >= MOTOR_SPINUP_TIME)
    {
      if (IgnoreFirstPeak)
      {
        if (ThrottleIteration < AUTO_LAUNCH_THROTTLE_MAX)
        {
          ThrottleIteration += MOTOR_SPINUP_VALUE;
        }
        else
        {
          ThrottleIteration = AUTO_LAUNCH_THROTTLE_MAX;
        }
      }
      IgnoreFirstPeak = true;
      ThrottleStart = SCHEDULERTIME.GetMillis();
    }
  }
  else
  {
    if (SCHEDULERTIME.GetMillis() - ThrottleStart >= LAUNCH_MOTOR_IDLE_SPINUP_TIME)
    {
      if (IgnoreFirstPeak)
      {
        RC_Resources.Attitude.Controller[THROTTLE] = AUTO_LAUNCH_THROTTLE_MAX;
        BEEPER.Silence();
      }
      else
      {
        ThrottleStart = SCHEDULERTIME.GetMillis();
      }
      IgnoreFirstPeak = true;
    }
    else
    {
      RC_Resources.Attitude.Controller[THROTTLE] = RC_Resources.Attitude.ThrottleMin; //VAMOS MANTER A VELOCIDADE MINIMA DEFINIDA PELO USUARIO
    }
  }
}

int16_t AutoLaunchClass::CalculeControllToPitch(float AngleInDegrees, int16_t InclinationMaxOfStabilize)
{
  AngleInDegrees *= 10;
  AngleInDegrees = Constrain_Float(AngleInDegrees, (float)-InclinationMaxOfStabilize, (float)InclinationMaxOfStabilize);
  float CalcValueA = (500.0f - (-500.0f)) * (((float)AngleInDegrees) - ((float)-InclinationMaxOfStabilize));
  float CalcValueB = ((float)InclinationMaxOfStabilize) - ((float)-InclinationMaxOfStabilize);
  return ((CalcValueA / CalcValueB) + (-500.0f));
}

void AutoLaunchClass::RCControllerYawPitchRoll_Apply_Logic(void)
{
  if (AUTOLAUNCH.GetPlaneType() == WITH_WHEELS)
  {
    if (AUTOLAUNCH.GetStateOfThrottle())
    {
      //PARA PLANES COM RODAS O PITCH INCLINA QUANDO O THROTTLE CHEGAR EM UM DETERMINADO VALOR
      RC_Resources.Attitude.Controller[ROLL] = 0;
      RC_Resources.Attitude.Controller[PITCH] = AUTOLAUNCH.CalculeControllToPitch(-AUTO_LAUNCH_ANGLE, 300);
      RC_Resources.Attitude.Controller[YAW] = 0;
    }
  }
  else
  {
    //PARA PLANES SEM RODAS O PITCH FICA SEMPRE INCLINADO
    RC_Resources.Attitude.Controller[ROLL] = 0;
    RC_Resources.Attitude.Controller[PITCH] = AUTOLAUNCH.CalculeControllToPitch(-AUTO_LAUNCH_ANGLE, 300);
    RC_Resources.Attitude.Controller[YAW] = 0;
  }
}

bool AutoLaunchClass::GetStateOfThrottle(void)
{
  return (DECODE.GetRxChannelOutput(THROTTLE) >= 1400) && AutoLaunchState;
}

bool AutoLaunchClass::GetValidStateToRunLaunch(void)
{
  return GetActualThrottleStatus(THROTTLE_MIDDLE);
}

bool AutoLaunchClass::GetIMUAngleBanked(float VectorPitch, bool CheckIMUInclination)
{
  return ((VectorPitch < (IMU_BANKED_ANGLE)) && CheckIMUInclination);
}

bool AutoLaunchClass::GetTimerOverFlow(void)
{
  if (SCHEDULERTIME.GetMillis() - AbortAutoLaunch >= AUTO_LAUCH_EXIT_FUNCTION)
  {
    if (!IgnoreFirstPeakOverFlow)
    {
      AbortAutoLaunch = SCHEDULERTIME.GetMillis();
    }
    if (IgnoreFirstPeakOverFlow)
    {
      return true;
    }
    IgnoreFirstPeakOverFlow = true;
  }
  return false;
}

bool AutoLaunchClass::GetMaxAltitudeReached(void)
{
  if (!I2CResources.Found.Barometer)
  {
    return false;
  }
  return ((AUTO_LAUCH_MAX_ALTITUDE * 100) > 0) && (Barometer.INS.Altitude.Estimated >= (AUTO_LAUCH_MAX_ALTITUDE * 100));
}

bool AutoLaunchClass::GetStatusCompleted(void)
{
  //VERIFIQUE APENAS SE OS STICK'S FORAM MANIPULADOS OU SE A ALTITUDE DEFINIDA FOI ATINGIDA
  if (AUTO_LAUCH_EXIT_FUNCTION == NONE)
  {
    return (GetSticksDeflected(15)) || (AUTOLAUNCH.GetMaxAltitudeReached());
  }
  //FAÇA A MESMA VERIFICAÇÃO DE CIMA,PORÉM COM O ESTOURO DO TEMPO MAXIMO DE LAUNCH
  return (AUTOLAUNCH.GetTimerOverFlow()) || (GetSticksDeflected(15)) || (AUTOLAUNCH.GetMaxAltitudeReached());
}

uint8_t AutoLaunchClass::GetPlaneType(void)
{
#ifndef __AVR_ATmega2560__
  if (JCF_Param.AirPlane_Wheels == WITH_WHEELS)
  {
    return WITH_WHEELS;
  }
#endif
  return WITHOUT_WHEELS;
}

void AutoLaunchClass::ResetParameters(void)
{
  //RESETA OS PARAMETROS QUANDO ESTIVER DESARMADO
  StateLaunched = false;
  LaunchedDetect = false;
  IgnoreFirstPeak = false;
  IgnoreFirstPeakOverFlow = false;
  ThrottleIteration = 1000;
  ThrottleStart = 0;
  AbortAutoLaunch = 0;
}