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

#ifndef PIDXYZ_H_
#define PIDXYZ_H_
#include "Arduino.h"
extern int16_t IntegralAccError[2];
extern int16_t IntegralGyroError[2];
extern int32_t IntegralGyroError_Yaw;
extern int16_t CalcedRateTargetRoll;
extern int16_t CalcedRateTargetPitch;
extern int16_t CalcedRateTargetYaw;
void PID_DerivativeLPF_Update();
void PID_Update();
void PID_Controll_Pitch(int16_t RateTargetInput);
void PID_Controll_Roll(int16_t RateTargetInput);
void PID_Controll_Yaw(int16_t RateTargetInput);
int16_t TurnControllerForAirPlane(int16_t RadioControlToTurn);
void PID_Reset_Integral_Accumulators();
#endif
