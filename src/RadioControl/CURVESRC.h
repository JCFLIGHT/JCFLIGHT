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

#ifndef CURVESRC_H_
#define CURVESRC_H_
#include "Arduino.h"
void CurvesRC_SetValues();
void CurvesRC_CalculeValue();
int16_t RCControllerToRate(int16_t StickData, uint8_t Rate);
int16_t CalcedAttitudeRC(int16_t Data, int16_t RcExpo);
uint16_t CalcedLookupThrottle(uint16_t CalcedDeflection);
int16_t RCLookupThrottleMiddle(void);
#endif
