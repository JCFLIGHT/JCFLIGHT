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

#include "GIMBAL.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/DECODE.h"
#include "MotorsControl/MOTORS.h"
#include "Common/RCDEFINES.h"

void Gimbal_Controll()
{
  //CONTROLE DO GIMBAL
  if (GimbalControlAux == NONE)
  {
    MotorControl[GIMBAL] = MIDDLE_STICKS_PULSE;
  }
  else if (GimbalControlAux == RCAUX1)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX1);
  }
  else if (GimbalControlAux == RCAUX2)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX2);
  }
  else if (GimbalControlAux == RCAUX3)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX3);
  }
  else if (GimbalControlAux == RCAUX4)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX4);
  }
  else if (GimbalControlAux == RCAUX5)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX5);
  }
  else if (GimbalControlAux == RCAUX6)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX6);
  }
  else if (GimbalControlAux == RCAUX7)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX7);
  }
  else if (GimbalControlAux == RCAUX8)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX8);
  }
}
