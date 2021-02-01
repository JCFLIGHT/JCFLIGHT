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
#include "Common/VARIABLES.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/STATES.h"
#include "AHRS/VECTOR.h"
#include "Buzzer/BUZZER.h"
#include "FrameStatus/FRAMESTATUS.h"

#define AHRS_BANKED_ANGLE 25                                 //25 GRAUS MAXIMO DE BANK ANGLE (CONSIDERANDO EM RADIANOS = 436)
#define IMU_BANKED_ANGLE -450.0f                             //-45 GRAUS DE INCLINAÇÃO NA IMU
#define LAUNCH_MOTOR_IDLE_SPINUP_TIME 1500                   //ARMA O MOTOR DEPOIS DE 1.5 SEGUNDO APÓS DETECTAR O AUTO-LAUNCH
#define AUTO_LAUNCH_ANGLE 18                                 //VALOR DO PITCH (ELEVATOR) AO FAZER O AUTO-LAUNCH (VALOR EM GRAUS)
#define SWING_LAUNCH_MIN_ROTATION_RATE ConvertToRadians(100) //NO MINIMO UM RATE DE 100DPS NO GYRO
#define LAUNCH_VELOCITY_THRESH 300                           //3 METROS/S
#define MOTOR_SPINUP_VALUE 100                               //VALOR DA INCREMENTAÇÃO DO THROTTLE PARA PLANES COM RODAS
#define MOTOR_SPINUP_TIME 300                                //VAI SUBINDO O THROTTLE AOS POUCOS,BOM PARA AERO COM RODAS (TEMPO EM MS)
#define AUTO_LAUCH_EXIT_FUNCTION 5000                        //TEMPO DE PARA SAIR DO MODO AUTO-LAUCH APÓS A DETECÇÃO (TEMPO EM MS)
#define AUTO_LAUNCH_THROTTLE_MAX 1700                        //VALOR MAXIMO DE ACELERAÇÃO
#define AUTO_LAUCH_MAX_ALTITUDE 0                            //ALTITUDE MAXIMA PARA VALIDAR O AUTO-LAUNCH (VALOR EM METROS)

bool PlaneType = 0;
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

const float GetPitchAccelerationInMSS()
{
  return BodyFrameAcceleration.Roll;
}

const float GetRollAccelerationInMSS()
{
  return BodyFrameAcceleration.Pitch;
}

const bool GetSwingVelocityState()
{
  const float SwingVelocity = (ABS_FLOAT(BodyFrameRotation.Yaw) * 10 > SWING_LAUNCH_MIN_ROTATION_RATE) ? (GetRollAccelerationInMSS() / BodyFrameRotation.Yaw) : 0;
  return (SwingVelocity > LAUNCH_VELOCITY_THRESH) && (GetPitchAccelerationInMSS() > 0);
}

void AutoLaunchDetector()
{
  if (GetIMUAngleBanked(GetPitchAccelerationInMSS(), CheckAnglesInclination(AHRS_BANKED_ANGLE)) || GetSwingVelocityState())
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

void Auto_Launch_Update()
{
  if (!SetFlightModes[ATACK_MODE] || GetFrameStateOfMultirotor())
  {
    return;
  }
  else
  {
    if (GetValidStateToRunLaunch() && !LaunchedDetect)
    {
      if (!AutoLaunchState)
      {
        BEEPER.Play(BEEPER_AUTOLAUNCH);
      }
      else
      {
        BEEPER.Play(BEEPER_LAUNCHED);
      }
      if (PlaneType == WITH_WHEELS)
      {
        AutoLaunchState = true;
        if (!COMMAND_ARM_DISARM)
        {
          COMMAND_ARM_DISARM = true;
        }
        RCControllerThrottle_Apply_Logic(true); //TRUE PARA PLANES COM TREM DE POUSO
      }
      if (AutoLaunchState)
      {
        if (PlaneType == WITHOUT_WHEELS)
        {
          RCControllerThrottle_Apply_Logic(false); //FALSE PARA PLANES SEM TREM DE POUSO
        }
        if (PlaneType == WITH_WHEELS)
        {
          RCControllerYawPitchRoll_Apply_Logic(true); //TRUE PARA PLANES COM TREM DE POUSO
        }
        if (!COMMAND_ARM_DISARM)
        {
          COMMAND_ARM_DISARM = true;
        }
        if (AutoLaunchCompleted())
        {
          LaunchedDetect = true;
          AutoLaunchState = false;
        }
        StateLaunched = true;
      }
      else if (!StateLaunched && PlaneType == WITHOUT_WHEELS) //NÃO VAMOS USAR A IMU PARA ATIVAR O AUTO-LAUCH EM AEROMODELOS COM RODAS
      {
        AutoLaunchDetector();
      }
      if (PlaneType == WITHOUT_WHEELS && !LaunchedDetect)
      {
        RCControllerYawPitchRoll_Apply_Logic(false); //A FUNÇÃO FICA SEMPRE ATIVA PARA PLANES SEM TREM DE POUSO
      }
    }
  }
  if (!COMMAND_ARM_DISARM && !GetValidStateToRunLaunch())
  {
    ResetParameters();
  }
}

void RCControllerThrottle_Apply_Logic(bool SlowThr)
{
  if (SlowThr)
  {
    RCController[THROTTLE] = ThrottleIteration;
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
        RCController[THROTTLE] = AUTO_LAUNCH_THROTTLE_MAX;
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
      RCController[THROTTLE] = MotorSpeed; //VAMOS MANTER A VELOCIDADE MINIMA DEFINIDA PELO USUARIO
    }
  }
}

int16_t CalculeControllToPitch(float AngleInDegrees, int16_t InclinationMaxOfStabilize)
{
  AngleInDegrees *= 10;
  AngleInDegrees = Constrain_Float(AngleInDegrees, (float)-InclinationMaxOfStabilize, (float)InclinationMaxOfStabilize);
  float CalcValueA = (500.0f - (-500.0f)) * (((float)AngleInDegrees) - ((float)-InclinationMaxOfStabilize));
  float CalcValueB = ((float)InclinationMaxOfStabilize) - ((float)-InclinationMaxOfStabilize);
  return ((CalcValueA / CalcValueB) + (-500.0f));
}

void RCControllerYawPitchRoll_Apply_Logic(bool SlowControll)
{
  if (SlowControll)
  {
    if (GetStateOfThrottle())
    {
      //PARA PLANES COM RODAS O PITCH INCLINA QUANDO O THROTTLE CHEGAR EM UM DETERMINADO VALOR
      RCController[ROLL] = 0;
      RCController[PITCH] = CalculeControllToPitch(-AUTO_LAUNCH_ANGLE, 300);
      RCController[YAW] = 0;
    }
  }
  else
  {
    //PARA PLANES SEM RODAS O PITCH FICA SEMPRE INCLINADO
    RCController[ROLL] = 0;
    RCController[PITCH] = CalculeControllToPitch(-AUTO_LAUNCH_ANGLE, 300);
    RCController[YAW] = 0;
  }
}

bool GetStateOfThrottle()
{
  return (RCController[THROTTLE] >= 1400) && AutoLaunchState;
}

bool GetValidStateToRunLaunch()
{
  return (RadioControllOutput[THROTTLE] >= 1400);
}

bool GetIMUAngleBanked(float VectorPitch, bool CheckIMUInclination)
{
  return (VectorPitch < (IMU_BANKED_ANGLE) && CheckIMUInclination);
}

bool AutoLaunchTimerOverFlow()
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

bool AutoLaunchMaxAltitudeReached(void)
{
  return ((AUTO_LAUCH_MAX_ALTITUDE * 100) > 0) && (ALTITUDE.EstimatedAltitude >= (AUTO_LAUCH_MAX_ALTITUDE * 100));
}

bool AutoLaunchCompleted()
{
  //VERIFIQUE APENAS SE OS STICK'S FORAM MANIPULADOS OU SE A ALTITUDE DEFINIDA FOI ATINGIDA
  if (AUTO_LAUCH_EXIT_FUNCTION == 0)
  {
    return (SticksDeflected(15)) || (AutoLaunchMaxAltitudeReached());
  }
  //FAÇA A MESMA VERIFICAÇÃO DE CIMA,PORÉM COM O ESTOURO DO TEMPO MAXIMO DE LAUNCH
  return (AutoLaunchTimerOverFlow()) || (SticksDeflected(15)) || (AutoLaunchMaxAltitudeReached());
}

void ResetParameters()
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