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

#ifndef GPSSTATES_H_
#define GPSSTATES_H_
#include "Build/LIBDEPENDENCIES.h"
bool Get_State_Armed_With_GPS(void);
bool Get_GPS_In_Good_Condition(void);
bool Get_GPS_In_Bad_Condition(void);
bool Get_GPS_In_Eight_Or_Plus_Satellites(void);
bool Get_GPS_Type(uint8_t GPS_Type);
bool Get_GPS_Heading_Is_Valid(void);
bool Get_GPS_Flight_Modes_And_Navigation_In_Use(void);
bool Get_GPS_Only_Flight_Modes_In_Use(void);
bool Get_GPS_Used_To_Land(void);
#endif