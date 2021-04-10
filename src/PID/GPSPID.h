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

#ifndef GPSPID_H_
#define GPSPID_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern PID_Terms_Float_Struct PositionHoldPID;
extern PID_Terms_Float_Struct PositionHoldRatePID;
extern PID_Terms_Float_Struct NavigationPID;
extern PID_Terms_Float_Struct PositionHoldPIDArray[2];
extern PID_Terms_Float_Struct PositionHoldRatePIDArray[2];
extern PID_Terms_Float_Struct NavigationPIDArray[2];
int32_t GPSGetProportional(int32_t Error, PID_Terms_Float_Struct *PID);
int32_t GPSGetIntegral(int32_t Error, float DeltaTime, PID_Terms_Float_Struct *PID, PID_Terms_Float_Struct *GPS_PID_Param);
int32_t GPSGetDerivative(int32_t Input, float DeltaTime, PID_Terms_Float_Struct *PID, PID_Terms_Float_Struct *GPS_PID_Param);
void GPSResetPID(PID_Terms_Float_Struct *PID);
void ResetAllGPSPID(void);
#endif