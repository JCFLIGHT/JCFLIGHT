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
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern INS_Struct INS;
class InertialNavigationClass
{
public:
  void Calculate_AccelerationXYZ_To_EarthFrame();
  void Calculate_AccelerationXY(float DeltaTime);
  void Calculate_AccelerationZ(float DeltaTime);

private:
  void UpdateAccelerationEarthFrame_Filtered(uint8_t ArrayCount);
  void CorrectXYStateWithGPS(float DeltaTime);
  void EstimationPredictXY(float DeltaTime);
  void SaveXYPositionToHistory();
  void ResetXYState();
  void CorrectZStateWithBaro(float DeltaTime);
  void EstimationPredictZ(float DeltaTime);
  void SaveZPositionToHistory();
  void ResetZState();
};
extern InertialNavigationClass INERTIALNAVIGATION;
#endif
