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
extern bool Home_Point;
extern float ScaleDownOfLongitude;
extern uint8_t GPS_Flight_Mode;
extern uint8_t GPS_Navigation_Mode;
extern uint8_t RTH_Altitude;
extern int16_t GPSActualSpeed[2];
extern int16_t GPS_Navigation_Array[2];
extern uint16_t DistanceToHome;
extern int32_t Stored_Coordinates_Home_Point[2];
extern int32_t Two_Points_Distance;
extern int32_t Target_Bearing;
extern int32_t GPSDistanceToHome[2];
extern int32_t Original_Target_Bearing;
extern int32_t Coordinates_To_Navigation[2];
void GPS_Process_FlightModes(float DeltaTime);
void Do_Mode_RTH_Now(void);
void GPS_Adjust_Heading();
void Set_Next_Point_To_Navigation(int32_t *Latitude_Destiny, int32_t *Longitude_Destiny);
bool Point_Reached(void);
void SetThisPointToPositionHold();
void ApplyPosHoldPIDControl(float *DeltaTime);
bool NavStateForPosHold(void);
void GPSCalculateNavigationRate(uint16_t Maximum_Velocity);
int16_t Calculate_Navigation_Speed(int16_t Maximum_Velocity);
void Reset_Home_Point(void);
void GPS_Reset_Navigation(void);
void LoadGPSParameters(void);
#endif
