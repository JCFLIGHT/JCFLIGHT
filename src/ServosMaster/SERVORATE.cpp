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

#include "SERVORATE.h"
#include "AirPlane/AIRPLANE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "BAR/BAR.h"
#include "Math/MATHSUPPORT.h"
#include "Common/ENUM.h"

int16_t ServoScaleMin[MAX_SUPPORTED_SERVOS];
int16_t ServoScaleMax[MAX_SUPPORTED_SERVOS];

void Servo_Rate_Update()
{
  if (GetFrameStateOfMultirotor())
  {
    return;
  }

  //OBTÉM O RATE DOS SERVOS
  AIR_PLANE.ServoRate[SERVO1] = GET_SERVO_RATE(SERVO1_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO2] = GET_SERVO_RATE(SERVO2_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO3] = GET_SERVO_RATE(SERVO3_RATE_ADDR);
  AIR_PLANE.ServoRate[SERVO4] = GET_SERVO_RATE(SERVO4_RATE_ADDR);
}

void Servo_Rate_Apply()
{
  //CALCULA O RATE PARA OS SERVOS
  AIR_PLANE.ServoToFilter[SERVO1] = (((int32_t)AIR_PLANE.ServoRate[SERVO1] * AIR_PLANE.ServoToFilter[SERVO1]) / 100L); //AJUSTA O RATE DO SERVO 1
  AIR_PLANE.ServoToFilter[SERVO2] = (((int32_t)AIR_PLANE.ServoRate[SERVO2] * AIR_PLANE.ServoToFilter[SERVO2]) / 100L); //AJUSTA O RATE DO SERVO 2
  AIR_PLANE.ServoToFilter[SERVO3] = (((int32_t)AIR_PLANE.ServoRate[SERVO3] * AIR_PLANE.ServoToFilter[SERVO3]) / 100L); //AJUSTA O RATE DO SERVO 3
  AIR_PLANE.ServoToFilter[SERVO4] = (((int32_t)AIR_PLANE.ServoRate[SERVO4] * AIR_PLANE.ServoToFilter[SERVO4]) / 100L); //AJUSTA O RATE DO SERVO 4

  //AJUSTA O PONTO MÉDIO DOS SERVOS
  AIR_PLANE.ServoToFilter[SERVO1] += AIR_PLANE.ServoMiddle[SERVO1];
  AIR_PLANE.ServoToFilter[SERVO2] += AIR_PLANE.ServoMiddle[SERVO2];
  AIR_PLANE.ServoToFilter[SERVO3] += AIR_PLANE.ServoMiddle[SERVO3];
  AIR_PLANE.ServoToFilter[SERVO4] += AIR_PLANE.ServoMiddle[SERVO4];
}
