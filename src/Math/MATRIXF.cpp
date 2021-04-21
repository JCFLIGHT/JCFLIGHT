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

#include "MATRIXF.h"

template <typename T>
Vector3x3<T> Matrix3x3<T>::operator*(const Vector3x3<T> &Vector) const
{
    return Vector3x3<T>(A.X * Vector.X + A.Y * Vector.Y + A.Z * Vector.Z,
                        B.X * Vector.X + B.Y * Vector.Y + B.Z * Vector.Z,
                        C.X * Vector.X + C.Y * Vector.Y + C.Z * Vector.Z);
}

template Vector3x3<float> Matrix3x3<float>::operator*(const Vector3x3<float> &Vector) const;