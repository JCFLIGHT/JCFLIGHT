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

#include "FRAMESTATUS.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"

bool GetFrameStateOfMultirotor()
{
    if (FrameType == QUAD_X ||
        FrameType == HEXA_X ||
        FrameType == HEXA_I ||
        FrameType == ZMR250 ||
        FrameType == TBS)
        return true;
    return false;
}

bool GetFrameStateOfAirPlane()
{
    if (FrameType == AIRPLANE ||
        FrameType == FIXED_WING ||
        FrameType == PLANE_VTAIL)
        return true;
    return false;
}