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

#ifndef AUTODECLINATION_H_
#define AUTODECLINATION_H_
#include "Build/LIBDEPENDENCIES.h"
class AutoDeclinationClass
{
public:
  void Set_Initial_Location(int32_t LocationLatitude, int32_t LocationLongitude);
  float GetDeclination();

private:
  float ReturnDeclination;
  float GetDeclinationCalced(float GPSLatitude, float GPSLongitude);
};
extern AutoDeclinationClass AUTODECLINATION;
#endif
