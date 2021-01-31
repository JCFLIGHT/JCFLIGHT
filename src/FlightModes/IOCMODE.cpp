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
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//#define DEBUG_IOC

void IOC_Mode_Update()
{
  if (Do_IOC_Mode && GetFrameStateOfMultirotor())
  {
    const float HeadingDifference = ConvertToRadians(ConvertDeciDegreesToDegrees(ATTITUDE.CompassHeading) - ConvertDeciDegreesToDegrees(IOC_Initial_Compass));
    const float CosineDifference = Fast_Cosine(HeadingDifference);
    const float SineDifference = Fast_Sine(HeadingDifference);
    const int16_t RCController_PITCH = RCController[PITCH] * CosineDifference + RCController[ROLL] * SineDifference;
    RCController[ROLL] = RCController[ROLL] * CosineDifference - RCController[PITCH] * SineDifference;
    RCController[PITCH] = RCController_PITCH;
  }
#ifdef DEBUG_IOC
  FastSerialPrintln(PSTR("RCController[ROLL]:%d RCController[PITCH]:%d RCController_PITCH:%d HeadingDiff:%.3f CosineDiff:%.3f SineDiff:%.3f\n"),
                    RCController[ROLL],
                    RCController[PITCH],
                    RCController_PITCH,
                    HeadingDifference,
                    CosineDifference,
                    SineDifference);
#endif
}
