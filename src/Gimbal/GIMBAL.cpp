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
#include "Common/ENUM.h"
#include "Math/MATHSUPPORT.h"
#include "Param/PARAM.h"

void Gimbal_Controll(void)
{
  //CONTROLE DO GIMBAL
  if (GimbalConfig == NONE)
  {
    MotorControl[GIMBAL] = MIDDLE_STICKS_PULSE;
  }
  else if (GimbalConfig == RCAUX1)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX1);
  }
  else if (GimbalConfig == RCAUX2)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX2);
  }
  else if (GimbalConfig == RCAUX3)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX3);
  }
  else if (GimbalConfig == RCAUX4)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX4);
  }
  else if (GimbalConfig == RCAUX5)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX5);
  }
  else if (GimbalConfig == RCAUX6)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX6);
  }
  else if (GimbalConfig == RCAUX7)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX7);
  }
  else if (GimbalConfig == RCAUX8)
  {
    MotorControl[GIMBAL] = DECODE.GetRxChannelOutput(AUX8);
  }
#ifndef __AVR_ATmega2560__
  Constrain_16Bits(MotorControl[GIMBAL], JCF_Param.GimbalMinValue, JCF_Param.GimbalMaxValue);
#endif
}
