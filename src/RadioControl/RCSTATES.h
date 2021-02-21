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

#ifndef RCSTATES_H_
#define RCSTATES_H_
#include "Arduino.h"
bool CheckInclinationForArm(void);
bool ArmDelayedState(void);
void ResetArmDelayed(void);
bool DisarmDelayedState(void);
void ResetDisarmDelayed(void);
bool SticksStateToArm(void);
bool SticksStateToDisarm(void);
bool SticksInAutoPilotPosition(int16_t AutoPilotValue);
bool SticksDeflected(int16_t MinDeflectionValue);
bool GetActualThrottleStatus(uint8_t ThrottleStatus);
#endif