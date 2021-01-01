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
#include "Arduino.h"
#include "Common/STRUCTS.h"
int32_t GPSGetProportional(int32_t Error, struct _PID_PARAM *PID);
int32_t GPSGetIntegral(int32_t Error, float *DeltaTime, struct _GPS_PID *PID, struct _PID_PARAM *GPS_PID_Param);
int32_t GPSGetDerivative(int32_t Input, float *DeltaTime, struct _GPS_PID *PID, struct _PID_PARAM *GPS_PID_Param);
void GPSResetPID(struct _GPS_PID *PID);
extern PID_PARAM PositionHoldPID;
extern PID_PARAM PositionHoldRatePID;
extern PID_PARAM NavigationPID;
extern GPS_PID PositionHoldPIDArray[2];
extern GPS_PID PositionHoldRatePIDArray[2];
extern GPS_PID NavigationPIDArray[2];
#endif