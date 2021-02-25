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

#ifndef DYNAMICPID_H_
#define DYNAMICPID_H_
#include "Arduino.h"
extern uint8_t RCRate;
extern uint8_t RCExpo;
extern uint8_t YawRate;
extern uint8_t ThrottleMiddle;
extern uint8_t ThrottleExpo;
extern int16_t RCController[4];
extern int16_t AttitudeThrottleMin;
extern int16_t AttitudeThrottleMax;
void RC_PID_Update();
#endif
