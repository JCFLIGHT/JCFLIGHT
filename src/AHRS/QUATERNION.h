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

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "VECTOR.h"

static inline Struct_Quaternion *QuaternionInitUnit(Struct_Quaternion *Result)
{
    Result->q0 = 1.0f;
    Result->q1 = 0.0f;
    Result->q2 = 0.0f;
    Result->q3 = 0.0f;
    return Result;
}

static inline float QuaternionNormalizedSquared(const Struct_Quaternion *Quaternion)
{
    return SquareFloat(Quaternion->q0) + SquareFloat(Quaternion->q1) + SquareFloat(Quaternion->q2) + SquareFloat(Quaternion->q3);
}

static inline Struct_Quaternion *QuaternionNormalize(Struct_Quaternion *Result,
                                                     const Struct_Quaternion *Quaternion)
{
    float CheckSquare = Fast_SquareRoot(QuaternionNormalizedSquared(Quaternion));
    if (CheckSquare < 1e-6f)
    {
        Result->q0 = 1;
        Result->q1 = 0;
        Result->q2 = 0;
        Result->q3 = 0;
    }
    else
    {
        Result->q0 = Quaternion->q0 / CheckSquare;
        Result->q1 = Quaternion->q1 / CheckSquare;
        Result->q2 = Quaternion->q2 / CheckSquare;
        Result->q3 = Quaternion->q3 / CheckSquare;
    }
    return Result;
}

static inline Struct_Quaternion *QuaternionMultiply(Struct_Quaternion *Result,
                                                    const Struct_Quaternion *VectorA,
                                                    const Struct_Quaternion *VectorB)
{
    Struct_Quaternion CalcedResult;
    CalcedResult.q0 = VectorA->q0 * VectorB->q0 - VectorA->q1 * VectorB->q1 - VectorA->q2 * VectorB->q2 - VectorA->q3 * VectorB->q3;
    CalcedResult.q1 = VectorA->q0 * VectorB->q1 + VectorA->q1 * VectorB->q0 + VectorA->q2 * VectorB->q3 - VectorA->q3 * VectorB->q2;
    CalcedResult.q2 = VectorA->q0 * VectorB->q2 - VectorA->q1 * VectorB->q3 + VectorA->q2 * VectorB->q0 + VectorA->q3 * VectorB->q1;
    CalcedResult.q3 = VectorA->q0 * VectorB->q3 + VectorA->q1 * VectorB->q2 - VectorA->q2 * VectorB->q1 + VectorA->q3 * VectorB->q0;
    *Result = CalcedResult;
    return Result;
}

static inline Struct_Quaternion *QuaternionConjugate(Struct_Quaternion *Result,
                                                     const Struct_Quaternion *Quaternion)
{
    Result->q0 = Quaternion->q0;
    Result->q1 = -Quaternion->q1;
    Result->q2 = -Quaternion->q2;
    Result->q3 = -Quaternion->q3;
    return Result;
}

static inline Struct_Vector3x3 *QuaternionRotateVector(Struct_Vector3x3 *Result,
                                                       const Struct_Vector3x3 *Vector,
                                                       const Struct_Quaternion *Reference)
{
    Struct_Quaternion QuaternionVector, ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->Roll;
    QuaternionVector.q2 = Vector->Pitch;
    QuaternionVector.q3 = Vector->Yaw;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, &ReferenceConjugate, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, Reference);
    Result->Roll = QuaternionVector.q1;
    Result->Pitch = QuaternionVector.q2;
    Result->Yaw = QuaternionVector.q3;
    return Result;
}

static inline Struct_Vector3x3 *QuaternionRotateVectorInverse(Struct_Vector3x3 *Result,
                                                              const Struct_Vector3x3 *Vector,
                                                              const Struct_Quaternion *Reference)
{
    Struct_Quaternion QuaternionVector,
        ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->Roll;
    QuaternionVector.q2 = Vector->Pitch;
    QuaternionVector.q3 = Vector->Yaw;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, Reference, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, &ReferenceConjugate);
    Result->Roll = QuaternionVector.q1;
    Result->Pitch = QuaternionVector.q2;
    Result->Yaw = QuaternionVector.q3;
    return Result;
}

static inline Struct_Quaternion *QuaternionInitFromVector(Struct_Quaternion *Result,
                                                          const Struct_Vector3x3 *Vector)
{
    Result->q0 = 0.0f;
    Result->q1 = Vector->Roll;
    Result->q2 = Vector->Pitch;
    Result->q3 = Vector->Yaw;
    return Result;
}

static inline Struct_Quaternion *QuaternionScale(Struct_Quaternion *Result,
                                                 const Struct_Quaternion *VectorA,
                                                 const float VectorB)
{
    Struct_Quaternion CalcedResult;
    CalcedResult.q0 = VectorA->q0 * VectorB;
    CalcedResult.q1 = VectorA->q1 * VectorB;
    CalcedResult.q2 = VectorA->q2 * VectorB;
    CalcedResult.q3 = VectorA->q3 * VectorB;
    *Result = CalcedResult;
    return Result;
}
#endif