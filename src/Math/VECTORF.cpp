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

#include "VECTORF.h"
#include "MATRIXF.h"

template <typename T>
T Vector3x3<T>::operator*(const Vector3x3<T> &Vector) const
{
    return X * Vector.X + Y * Vector.Y + Z * Vector.Z;
}

template <typename T>
Vector3x3<T> &Vector3x3<T>::operator+=(const Vector3x3<T> &Vector)
{
    X += Vector.X;
    Y += Vector.Y;
    Z += Vector.Z;
    return *this;
}

template <typename T>
Vector3x3<T> Vector3x3<T>::operator/(const T Number) const
{
    return Vector3x3<T>(X / Number, Y / Number, Z / Number);
}

template <typename T>
Vector3x3<T> Vector3x3<T>::operator*(const T Number) const
{
    return Vector3x3<T>(X * Number, Y * Number, Z * Number);
}

template <typename T>
Vector3x3<T> Vector3x3<T>::operator-(const Vector3x3<T> &Vector) const
{
    return Vector3x3<T>(X - Vector.X, Y - Vector.Y, Z - Vector.Z);
}

template <typename T>
Vector3x3<T> Vector3x3<T>::operator*(const Matrix3x3<T> &Matrix) const
{
    return Vector3x3<T>(*this * Matrix.ColumnX(), *this * Matrix.ColumnY(), *this * Matrix.ColumnZ());
}

template <typename T>
Matrix3x3<T> Vector3x3<T>::Multiply_Row_Column(const Vector3x3<T> &Vector2) const
{
    const Vector3x3<T> Vector1 = *this;
    return Matrix3x3<T>(Vector1.X * Vector2.X, Vector1.X * Vector2.Y, Vector1.X * Vector2.Z,
                        Vector1.Y * Vector2.X, Vector1.Y * Vector2.Y, Vector1.Y * Vector2.Z,
                        Vector1.Z * Vector2.X, Vector1.Z * Vector2.Y, Vector1.Z * Vector2.Z);
}

template float Vector3x3<float>::operator*(const Vector3x3<float> &Vector) const;
template Vector3x3<float> Vector3x3<float>::operator*(const Matrix3x3<float> &Matrix) const;
template Matrix3x3<float> Vector3x3<float>::Multiply_Row_Column(const Vector3x3<float> &Vector2) const;
template Vector3x3<float> &Vector3x3<float>::operator+=(const Vector3x3<float> &Vector);
template Vector3x3<float> Vector3x3<float>::operator/(const float Number) const;
template Vector3x3<float> Vector3x3<float>::operator*(const float Number) const;
template Vector3x3<float> Vector3x3<float>::operator-(const Vector3x3<float> &Vector) const;
