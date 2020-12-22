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

#include "IOCMODE.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "FrameStatus/FRAMESTATUS.h"

void IOC_Mode_Update()
{
  if (GetFrameStateOfMultirotor())
  {
    if (Do_IOC_Mode)
    {
      int16_t HeadingDifference = ATTITUDE.CompassHeading - (IOC_Initial_Compass * 10);
      float CosineDifference = Calculate_Cosine_Approx(HeadingDifference);
      float SineDifference = Calculate_Sine_Approx(HeadingDifference);
      int16_t RCController_PITCH = RCController[PITCH] * CosineDifference + RCController[ROLL] * SineDifference;
      RCController[ROLL] = RCController[ROLL] * CosineDifference - RCController[PITCH] * SineDifference;
      RCController[PITCH] = RCController_PITCH;
    }
  }
}
