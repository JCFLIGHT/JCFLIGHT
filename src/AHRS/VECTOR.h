#ifndef VECTOR_H_
#define VECTOR_H_

#include <stdint.h>
#include "Math/AVRMATH.h"

typedef union
{
    float Vector[3];
    struct
    {
        float Roll;
        float Pitch;
        float Yaw;
    };
} Struct_Vector3x3;

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