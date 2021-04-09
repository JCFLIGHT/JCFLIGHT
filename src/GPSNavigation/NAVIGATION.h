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

#ifndef NAVIGATION_H_
#define NAVIGATION_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern GPS_Parameters_Struct GPS_Parameters;
int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity);
void GPS_Process_FlightModes(float DeltaTime);
void Do_Mode_RTH_Now(void);
void Set_Next_Point_To_Navigation(int32_t Latitude_Destiny, int32_t Longitude_Destiny);
bool Point_Reached(void);
void SetThisPointToPositionHold();
void ApplyPosHoldPIDControl(float DeltaTime);
bool Get_Safe_State_For_Pos_Hold(void);
void GPSCalculateNavigationRate(uint16_t Maximum_Velocity);
void Reset_Home_Point(void);
void GPS_Reset_Navigation(void);
void LoadGPSParameters(void);
#endif
