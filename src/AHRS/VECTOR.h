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

#ifndef VECTOR_H_
#define VECTOR_H_

#include <stdint.h>
#include "Common/STRUCTS.h"
#include "Math/MATHSUPPORT.h"

static inline float VectorNormSquared(const Struct_Vector3x3 *Vector)
{
    return SquareFloat(Vector->Roll) + SquareFloat(Vector->Pitch) + SquareFloat(Vector->Yaw);
}

static inline Struct_Vector3x3 *VectorNormalize(Struct_Vector3x3 *Result,
                                                const Struct_Vector3x3 *Vector)
{
    float Length = sqrtf(VectorNormSquared(Vector));
    if (Length != 0)
    {
        Result->Roll = Vector->Roll / Length;
        Result->Pitch = Vector->Pitch / Length;
        Result->Yaw = Vector->Yaw / Length;
    }
    else
    {
        Result->Roll = 0;
        Result->Pitch = 0;
        Result->Yaw = 0;
    }
    return Result;
}

static inline Struct_Vector3x3 *VectorCrossProduct(Struct_Vector3x3 *Result,
                                                   const Struct_Vector3x3 *VectorA,
                                                   const Struct_Vector3x3 *VectorB)
{
    Struct_Vector3x3 CalcedVector;
    CalcedVector.Roll = VectorA->Pitch * VectorB->Yaw - VectorA->Yaw * VectorB->Pitch;
    CalcedVector.Pitch = VectorA->Yaw * VectorB->Roll - VectorA->Roll * VectorB->Yaw;
    CalcedVector.Yaw = VectorA->Roll * VectorB->Pitch - VectorA->Pitch * VectorB->Roll;
    *Result = CalcedVector;
    return Result;
}

static inline Struct_Vector3x3 *VectorAdd(Struct_Vector3x3 *Result,
                                          const Struct_Vector3x3 *VectorA,
                                          const Struct_Vector3x3 *VectorB)
{
    Struct_Vector3x3 CalcedVector;
    CalcedVector.Roll = VectorA->Roll + VectorB->Roll;
    CalcedVector.Pitch = VectorA->Pitch + VectorB->Pitch;
    CalcedVector.Yaw = VectorA->Yaw + VectorB->Yaw;
    *Result = CalcedVector;
    return Result;
}

static inline Struct_Vector3x3 *VectorScale(Struct_Vector3x3 *Result,
                                            const Struct_Vector3x3 *VectorA,
                                            const float VectorB)
{
    Struct_Vector3x3 CalcedVector;
    CalcedVector.Roll = VectorA->Roll * VectorB;
    CalcedVector.Pitch = VectorA->Pitch * VectorB;
    CalcedVector.Yaw = VectorA->Yaw * VectorB;
    *Result = CalcedVector;
    return Result;
}
#endif