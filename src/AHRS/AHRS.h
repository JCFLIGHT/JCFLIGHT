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

#ifndef AHRS_H_
#define AHRS_H_
#include "Build/LIBDEPENDENCIES.h"
#include "VECTOR.h"
#include "Common/STRUCTS.h"
extern Attitude_Struct ATTITUDE;
extern Struct_Vector3x3 BodyFrameAcceleration;
extern Struct_Vector3x3 BodyFrameRotation;
extern Quaternion_Struct Orientation;
extern float RotationMatrix[3][3];
class AHRSClass
{
public:
  void Initialization();
  void Update(float DeltaTime);
  bool CheckAnglesInclination(int16_t Angle);
  float SineRoll();
  float CosineRoll();
  float SinePitch();
  float CosinePitch();
  float SineYaw();
  float CosineYaw();

private:
  float CosineTiltAngle(void);
};
extern AHRSClass AHRS;
#endif
