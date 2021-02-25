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
#include "Scheduler/SCHEDULERTIME.h"
#include "FastSerial/PRINTF.h"

PT1_Filter_Struct GForce_Smooth;

#define GRAVITY_MSS 9.80665f //M/S^2

void IMU_GForce_Update()
{
  static uint32_t PreviousDeltaTime = 0;

#ifndef __AVR_ATmega2560__
  float CalcedDeltaTime = (SCHEDULERTIME.GetMicros() - PreviousDeltaTime) * 1e-6;
#endif

  IMU.CalcedGForce = Fast_SquareRoot(VectorNormSquared(&BodyFrameAcceleration)) / GRAVITY_MSS;

  if (PreviousDeltaTime)
  {
#ifndef __AVR_ATmega2560__
    IMU.CalcedGForce = PT1FilterApply2(&GForce_Smooth, IMU.CalcedGForce, CalcedDeltaTime);
#else
    IMU.CalcedGForce = PT1FilterApply2(&GForce_Smooth, IMU.CalcedGForce, 1.0f / 1000.0f);
#endif
  }
  else
  {
    GForce_Smooth.RC = 0.2f;
    GForce_Smooth.State = IMU.CalcedGForce;
  }

  PreviousDeltaTime = SCHEDULERTIME.GetMicros();

  //DEBUG("IMU.CalcedGForce:%.4f", IMU.CalcedGForce);
}