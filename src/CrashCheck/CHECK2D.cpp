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

bool CrashCheck2D(int16_t ValueA, int16_t ValueB, float CrashAngle)
{
  //NÃO ERA NECESSARIO ESSAS DUAS VARIAVIES,MAS EU COLOQUEI POR QUE SOU UM BOBÃO
  static int16_t ConstrainAngleRoll = 0;
  static int16_t ConstrainAnglePitch = 0;
  CrashAngle = CrashAngle * (3.14f / 180.0f) * 1000.0f;
  ConstrainAngleRoll = Constrain_16Bits(ValueA, -1000, 1000);  //ANGULO MAXIMO DE -1000 A +1000
  ConstrainAnglePitch = Constrain_16Bits(ValueB, -1000, 1000); //ANGULO MAXIMO DE -1000 A +1000
  if ((ConstrainAngleRoll > CrashAngle) || (ConstrainAngleRoll < (-CrashAngle)) ||
      (ConstrainAnglePitch > CrashAngle) || (ConstrainAnglePitch < (-CrashAngle)))
  {
    return true;
  }
  return false;
}
