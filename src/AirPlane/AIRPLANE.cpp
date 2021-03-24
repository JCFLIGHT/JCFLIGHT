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
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AirPlaneClass AIR_PLANE;

void AirPlaneClass::Mode_ConventionalPlane_Run()
{
  if (!GetActualFrameState(AIRPLANE))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = RCController[YAW] * AIR_PLANE.ServoDirection[SERVO3];   //RUDDER   (LEME)
    AIR_PLANE.ServoToFilter[SERVO4] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1];  //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO2];  //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = PIDXYZ.PIDControllerApply[YAW] * AIR_PLANE.ServoDirection[SERVO3];   //RUDDER   (LEME)
    AIR_PLANE.ServoToFilter[SERVO4] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO4]; //ELEVATOR (PROFUNDOR)
  }
}

void AirPlaneClass::Mode_FixedWing_Run()
{
  if (!GetActualFrameState(FIXED_WING))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = (RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) + (RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO1]); //AILERON (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = (RCController[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) - (RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = (PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) + (PIDXYZ.PIDControllerApply[PITCH] * ServoDirection[SERVO1]); //AILERON (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = (PIDXYZ.PIDControllerApply[ROLL] * AIR_PLANE.ServoDirection[SERVO1]) - (PIDXYZ.PIDControllerApply[PITCH] * ServoDirection[SERVO2]); //AILERON (SERVO 2 DA ASA)
  }
}

void AirPlaneClass::Mode_PlaneVTail_Run()
{
  if (!GetActualFrameState(PLANE_VTAIL))
  {
    return;
  }

  if (IS_FLIGHT_MODE_ACTIVE(MANUAL_MODE)) //MODO MANUAL
  {
    AIR_PLANE.ServoToFilter[SERVO1] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO1];                      //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = RCController[PITCH] * AIR_PLANE.ServoDirection[SERVO2];                      //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = (RCController[ROLL] + RCController[YAW]) * AIR_PLANE.ServoDirection[SERVO3]; //V-TAIL   (CAUDA)
    AIR_PLANE.ServoToFilter[SERVO4] = (RCController[ROLL] - RCController[YAW]) * AIR_PLANE.ServoDirection[SERVO4]; //V-TAIL   (CAUDA)
  }
  else //STABILIZE OU ACRO
  {
    //CONTROLE DOS SERVOS DEPENDENTES DO PID E DO RADIO CONTROLE
    AIR_PLANE.ServoToFilter[SERVO1] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO1];                                   //AILERON  (SERVO 1 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO2] = PIDXYZ.PIDControllerApply[PITCH] * AIR_PLANE.ServoDirection[SERVO2];                                   //AILERON  (SERVO 2 DA ASA)
    AIR_PLANE.ServoToFilter[SERVO3] = (PIDXYZ.PIDControllerApply[ROLL] + PIDXYZ.PIDControllerApply[YAW]) * AIR_PLANE.ServoDirection[SERVO3]; //V-TAIL   (CAUDA)
    AIR_PLANE.ServoToFilter[SERVO4] = (PIDXYZ.PIDControllerApply[ROLL] - PIDXYZ.PIDControllerApply[YAW]) * AIR_PLANE.ServoDirection[SERVO4]; //V-TAIL   (CAUDA)
  }
}
