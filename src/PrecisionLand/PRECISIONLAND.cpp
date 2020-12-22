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

#include "PRECISIONLAND.h"
#include "Filters/LPFACCEF.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "FastSerial/PRINTF.h"

#define LOOP_RATE_IN_HZ 500          //HZ
#define LAND_CHECK_ACCEL_MOVING 3.0f //M/S^2
#define LPF_CUTOFF_IN_HZ 1.0f        //HZ

//DEBUG
//#define PRINTLN_PRECISIONLAND
//#define PRINTLN_LPFINROLL

LowPassFilter AccelerationEarthFrameFilteredRoll;
LowPassFilter AccelerationEarthFrameFilteredPitch;
LowPassFilter AccelerationEarthFrameFilteredYaw;

void Update_PrecisionLand()
{
  AccelerationEarthFrameFilteredRoll.Apply(INS.AccelerationEarthFrame[ROLL], LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);
  AccelerationEarthFrameFilteredPitch.Apply(INS.AccelerationEarthFrame[PITCH], LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);
  AccelerationEarthFrameFilteredYaw.Apply(INS.AccelerationEarthFrame[YAW], LPF_CUTOFF_IN_HZ, 1.0f / LOOP_RATE_IN_HZ);

#ifdef PRINTLN_PRECISIONLAND

  FastSerialPrintln(PSTR("GetAccelerationTotal:%f GetLandSuccess:%d\n"),
                    GetAccelerationTotal(), GetLandSuccess());

#endif

#ifdef PRINTLN_LPFINROLL

  FastSerialPrintln(PSTR("INS.AccelerationEarthFrame[ROLL]:%f AccelerationEarthFrameFilteredRoll.GetOutputFiltered():%f\n"),
                    INS.AccelerationEarthFrame[ROLL], AccelerationEarthFrameFilteredRoll.GetOutputFiltered());

#endif
}

//CALCULA A RAIZ QUADRADA DE TODAS AS ACELERAÇÕES PRESENTES NO EARTH FRAME COM 1G SUBTRAIDO NO EIXO Z
float GetAccelerationTotal()
{
  return sqrtf(SquareFloat(AccelerationEarthFrameFilteredRoll.GetOutputFiltered()) +
               SquareFloat(AccelerationEarthFrameFilteredPitch.GetOutputFiltered()) +
               SquareFloat(AccelerationEarthFrameFilteredYaw.GetOutputFiltered()));
}

//SE A VELOCIDADE FOR MAIOR OU IGUAL AO PARAMETRO "LAND_CHECK_ACCEL_MOVING" ISSO QUER DIZER QUE O UAV NÃO ESTÁ NO CHÃO
bool GetLandSuccess()
{
  if (GetAccelerationTotal() >= LAND_CHECK_ACCEL_MOVING)
    return false;
  return true;
}
