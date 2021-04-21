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

#ifndef MATRIXF_H_
#define MATRIXF_H_
#include "VECTORF.h"
template <typename T>
class Matrix3x3
{
public:
    Vector3x3<T> A, B, C;

    Matrix3x3<T>(const Vector3x3<T> &A_0, const Vector3x3<T> &B_0, const Vector3x3<T> &C_0) : A(A_0), B(B_0), C(C_0)
    {
    }

    Matrix3x3<T>(const T AX, const T AY, const T AZ, const T BX, const T BY, const T BZ, const T CX, const T CY, const T CZ) : A(AX, AY, AZ), B(BX, BY, BZ), C(CX, CY, CZ)
    {
    }

    Matrix3x3<T> operator-(const Matrix3x3<T> &Matrix) const
    {
        return Matrix3x3<T>(A - Matrix.A, B - Matrix.B, C - Matrix.C);
    }

    Matrix3x3<T> &operator-=(const Matrix3x3<T> &Matrix)
    {
        return *this = *this - Matrix;
    }

    Vector3x3<T> operator*(const Vector3x3<T> &Vector) const;

    Vector3x3<T> ColumnX(void) const
    {
        return Vector3x3<T>(A.X, B.X, C.X);
    }

    Vector3x3<T> ColumnY(void) const
    {
        return Vector3x3<T>(A.Y, B.Y, C.Y);
    }

    Vector3x3<T> ColumnZ(void) const
    {
        return Vector3x3<T>(A.Z, B.Z, C.Z);
    }
};
typedef Matrix3x3<float> Matrix3x3Float;
#endif