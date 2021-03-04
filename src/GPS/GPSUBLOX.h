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

#ifndef GPSUBLOX_H_
#define GPSUBLOX_H_
#include "Build/LIBDEPENDENCIES.h"
extern uint8_t GPS_NumberOfSatellites;
extern int16_t GPSVelNED[3];
extern uint16_t GPS_Ground_Course;
extern uint16_t GPS_Altitude;
extern uint16_t GPS_Ground_Speed;
extern uint16_t GPS_HDOP;
extern int32_t GPS_Coordinates_Vector[2];
void GPS_SerialInit(uint32_t Get_BaudRate);
void GPS_SerialRead(uint8_t ReadData);
void UBLOX_GetAllGPSData(void);
#endif
