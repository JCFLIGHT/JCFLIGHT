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

#include "CHECK2D.h"
#include "Math/MATHSUPPORT.h"

bool CrashCheck2D(int16_t AngleRoll, int16_t AnglePitch, float CrashAngle)
{
  CrashAngle = ConvertToRadians(CrashAngle) * 1000.0f;
  AngleRoll = Constrain_16Bits(AngleRoll, -1000, 1000);   //ANGULO MAXIMO DE -1000 A +1000
  AnglePitch = Constrain_16Bits(AnglePitch, -1000, 1000); //ANGULO MAXIMO DE -1000 A +1000
  if ((ABS_16BITS(AngleRoll) > CrashAngle) || (ABS_16BITS(AnglePitch) > CrashAngle))
  {
    return true;
  }
  return false;
}
