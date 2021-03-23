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

#include "SIMPLEMODE.h"
#include "Math/MATHSUPPORT.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Compass/COMPASSREAD.h"
#include "PID/RCPID.h"
#include "Common/ENUM.h"
#include "BitArray/BITARRAY.h"
#include "AHRS/AHRS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//DEBUG
//#define PRINTLN_SIMPLE_MODE

void Simple_Mode_Update()
{
  if (IS_FLIGHT_MODE_ACTIVE(SIMPLE_MODE) && GetFrameStateOfMultirotor())
  {
    const float HeadingDifference = ConvertToRadians(ATTITUDE.AngleOut[YAW] - CALIBRATION.Magnetometer.SimpleMode_Initial_Value);
    const float CosineDifference = Fast_Cosine(HeadingDifference);
    const float SineDifference = Fast_Sine(HeadingDifference);
    const int16_t CalcedRCControllerPITCH = RCController[PITCH] * CosineDifference + RCController[ROLL] * SineDifference;
    RCController[ROLL] = RCController[ROLL] * CosineDifference - RCController[PITCH] * SineDifference;
    RCController[PITCH] = CalcedRCControllerPITCH;
  }
#ifdef PRINTLN_SIMPLE_MODE
  PRINTF.SendToConsole(ProgramMemoryString("RCController[ROLL]:%d RCController[PITCH]:%d CalcedRCControllerPITCH:%d HeadingDiff:%.3f CosineDiff:%.3f SineDiff:%.3f\n"),
                       RCController[ROLL],
                       RCController[PITCH],
                       CalcedRCControllerPITCH,
                       HeadingDifference,
                       CosineDifference,
                       SineDifference);
#endif
}
