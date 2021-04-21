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

#include "AIRPLANE.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/RCDEFINES.h"
#include "PID/RCPID.h"
#include "PID/PIDXYZ.h"
#include "Common/ENUM.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AirPlaneClass AIRPLANE;

void AirPlaneClass::Update_Conventional_AirPlane(void)
{
  if (!GetActualPlatformEnabledUsingName(AIR_PLANE))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    Servo.Signal.UnFiltered[SERVO1] = RCController[ROLL] * Servo.Direction.GetAndSet[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = RCController[ROLL] * Servo.Direction.GetAndSet[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    Servo.Signal.UnFiltered[SERVO3] = RCController[YAW] * Servo.Direction.GetAndSet[SERVO3];   //RUDDER   (LEME)
    Servo.Signal.UnFiltered[SERVO4] = RCController[PITCH] * Servo.Direction.GetAndSet[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    Servo.Signal.UnFiltered[SERVO1] = PID_Resources.Controller.Output.Calced[ROLL] * Servo.Direction.GetAndSet[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = PID_Resources.Controller.Output.Calced[ROLL] * Servo.Direction.GetAndSet[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    Servo.Signal.UnFiltered[SERVO3] = PID_Resources.Controller.Output.Calced[YAW] * Servo.Direction.GetAndSet[SERVO3];   //RUDDER   (LEME)
    Servo.Signal.UnFiltered[SERVO4] = PID_Resources.Controller.Output.Calced[PITCH] * Servo.Direction.GetAndSet[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
}

void AirPlaneClass::Update_FixedWing(void)
{
  if (!GetActualPlatformEnabledUsingName(FIXED_WING))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    Servo.Signal.UnFiltered[SERVO1] = (RCController[ROLL] * Servo.Direction.GetAndSet[SERVO1]) + (RCController[PITCH] * Servo.Direction.GetAndSet[SERVO1]); //AILERON (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = (RCController[ROLL] * Servo.Direction.GetAndSet[SERVO1]) - (RCController[PITCH] * Servo.Direction.GetAndSet[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    Servo.Signal.UnFiltered[SERVO1] = (PID_Resources.Controller.Output.Calced[ROLL] * Servo.Direction.GetAndSet[SERVO1]) + (PID_Resources.Controller.Output.Calced[PITCH] * Servo.Direction.GetAndSet[SERVO1]); //AILERON (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = (PID_Resources.Controller.Output.Calced[ROLL] * Servo.Direction.GetAndSet[SERVO1]) - (PID_Resources.Controller.Output.Calced[PITCH] * Servo.Direction.GetAndSet[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
}

void AirPlaneClass::Update_AirPlaneVTail(void)
{
  if (!GetActualPlatformEnabledUsingName(PLANE_VTAIL))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    Servo.Signal.UnFiltered[SERVO1] = RCController[PITCH] * Servo.Direction.GetAndSet[SERVO1];                      //AILERON  (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = RCController[PITCH] * Servo.Direction.GetAndSet[SERVO2];                      //AILERON  (SERVO 2 DA ASA)
    Servo.Signal.UnFiltered[SERVO3] = (RCController[ROLL] + RCController[YAW]) * Servo.Direction.GetAndSet[SERVO3]; //V-TAIL   (CAUDA)
    Servo.Signal.UnFiltered[SERVO4] = (RCController[ROLL] - RCController[YAW]) * Servo.Direction.GetAndSet[SERVO4]; //V-TAIL   (CAUDA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    Servo.Signal.UnFiltered[SERVO1] = PID_Resources.Controller.Output.Calced[PITCH] * Servo.Direction.GetAndSet[SERVO1];                                   //AILERON  (SERVO 1 DA ASA)
    Servo.Signal.UnFiltered[SERVO2] = PID_Resources.Controller.Output.Calced[PITCH] * Servo.Direction.GetAndSet[SERVO2];                                   //AILERON  (SERVO 2 DA ASA)
    Servo.Signal.UnFiltered[SERVO3] = (PID_Resources.Controller.Output.Calced[ROLL] + PID_Resources.Controller.Output.Calced[YAW]) * Servo.Direction.GetAndSet[SERVO3]; //V-TAIL   (CAUDA)
    Servo.Signal.UnFiltered[SERVO4] = (PID_Resources.Controller.Output.Calced[ROLL] - PID_Resources.Controller.Output.Calced[YAW]) * Servo.Direction.GetAndSet[SERVO4]; //V-TAIL   (CAUDA)
  }
}
