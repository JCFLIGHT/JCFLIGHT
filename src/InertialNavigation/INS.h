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

#ifndef INS_H_
#define INS_H_
#include "Arduino.h"
class InertialNavigationClass
{
public:
  void Calculate_AccelerationXY();
  void Calculate_AccelerationZ();

private:
  void ResetXYState();
  void CorrectXYStateWithGPS(float *DeltaTime);
  void UpdateXYState(float *DeltaTime);
  void SaveXYPositionToHistory();
  void ResetZState();
  void CorrectZStateWithBaro(float *DeltaTime);
  void UpdateZState(float *DeltaTime);
  void SaveZPositionToHistory();
  void UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount);
};
extern InertialNavigationClass INERTIALNAVIGATION;
#endif
