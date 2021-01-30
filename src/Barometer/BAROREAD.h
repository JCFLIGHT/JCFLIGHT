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

#ifndef BAROREAD_H_
#define BAROREAD_H_
#include "Arduino.h"
extern int16_t BaroTemperatureRaw;
extern int32_t BaroPressureRaw;
void Baro_AverageFilter();
void CalculateBaroAltitudeForFlight();
void DoBaroCalibrationForFlight();
float Get_Altitude_Difference(float Base_Pressure, int32_t Pressure, int16_t BaroTemperature);
int32_t GetAltitudeForGCS();
#endif
