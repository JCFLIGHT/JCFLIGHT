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

#ifndef VECTORF_H_
#define VECTORF_H_
template <typename T>
class Matrix3x3;
template <typename T>
class Vector3x3
{

public:
    T X, Y, Z;

    Vector3x3<T>(const T X0, const T Y0, const T Z0) : X(X0), Y(Y0), Z(Z0)
    {
    }

    Vector3x3<T> operator*(const T Number) const;
    Vector3x3<T> &operator+=(const Vector3x3<T> &Vector);
    Vector3x3<T> operator/(const T Number) const;
    T operator*(const Vector3x3<T> &Vector) const;
    Vector3x3<T> operator*(const Matrix3x3<T> &Matrix) const;
    Vector3x3<T> operator-(const Vector3x3<T> &Vector) const;
    Matrix3x3<T> Multiply_Row_Column(const Vector3x3<T> &Vector) const;
};
typedef Vector3x3<float> Vector3x3Float;
#endif