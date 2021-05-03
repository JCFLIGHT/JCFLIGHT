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

#include "GFORCE.h"
#include "Math/MATHSUPPORT.h"
#include "AHRS/AHRS.h"
#include "Filters/PT1.h"
#include "Scheduler/SCHEDULER.h"
#include "Build/BOARDDEFS.h"
#include "IMU/ACCGYROREAD.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

PT1_Filter_Struct GravityForce_Smooth;

void IMU_GForce_Update(void)
{
  IMU.Accelerometer.GravityForce.Value = Fast_SquareRoot(VectorNormSquared(&BodyFrameAcceleration)) / GRAVITY_MSS;

  if (IMU.Accelerometer.GravityForce.Initialization)
  {
    IMU.Accelerometer.GravityForce.Value = PT1FilterApply2(&GravityForce_Smooth, IMU.Accelerometer.GravityForce.Value, SCHEDULER_SET_PERIOD_US(THIS_LOOP_RATE_IN_US) * 1e-6f);
  }
  else
  {
    GravityForce_Smooth.RC = 0.2f;
    GravityForce_Smooth.State = IMU.Accelerometer.GravityForce.Value;
    IMU.Accelerometer.GravityForce.Initialization = true;
  }

  //DEBUG("IMU.Accelerometer.GForce:%.4f", IMU.Accelerometer.GForce);
}