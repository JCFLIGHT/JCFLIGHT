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

#ifndef ALTITUDEHOLD_H_
#define ALTITUDEHOLD_H_
#include "Build/LIBDEPENDENCIES.h"
void AltitudeHold_Update_Params();
bool ApplyAltitudeHoldControl();
void SetAltitudeHold(int32_t ValueOfNewAltitudeHold);
bool GetAltitudeReached();
bool GetTakeOffInProgress();
bool isLandDetected();
void ResetIntegralOfVariometerError();
void RunLandDetector();
void ResetLandDetector();
bool GetGroundDetected();
bool GetGroundDetectedFor100ms();
bool GetLanded();
void InitializeHoveringThrottle();
void ApplyAltitudeHoldPIDControl(uint16_t DeltaTime, bool HoveringState);
#endif
