#include "MATHSUPPORT.h"

float Fast_SquareRoot(float ValueInput)
{
    if (ValueInput <= 5.877471754e-39f)
    {
        return 0.0f;
    }

    float acc;
    float xPower;
    int32_t intPart;

    union
    {
        float f;
        int32_t i;
    } xBits;

    xBits.f = ValueInput;

    intPart = ((xBits.i) >> 23);
    intPart -= 127;

    ValueInput = (float)(xBits.i & 0x007FFFFF);
    ValueInput *= 1.192092895507812e-07f;

    acc = 0.49959804148061f * ValueInput + 1.0f;
    xPower = ValueInput * ValueInput;
    acc += -0.12047308243453f * xPower;
    xPower *= ValueInput;
    acc += 0.04585425015501f * xPower;
    xPower *= ValueInput;
    acc += -0.01076564682800f * xPower;

    if (intPart & 0x1)
    {
        acc *= 1.41421356237309504880f;
    }

    xBits.i = intPart >> 1;
    xBits.i += 127;
    xBits.i <<= 23;

    return acc * xBits.f;
}

float Fast_Sine(float X)
{
    int32_t X32BITS = X;
    if (X32BITS < -32 || X32BITS > 32)
    {
        return 0.0f;
    }
    while (X > 3.14159265358979323846f)
    {
        X -= (2.0f * 3.14159265358979323846f);
    }
    while (X < -3.14159265358979323846f)
    {
        X += (2.0f * 3.14159265358979323846f);
    }
    if (X > (0.5f * 3.14159265358979323846f))
    {
        X = (0.5f * 3.14159265358979323846f) - (X - (0.5f * 3.14159265358979323846f));
    }
    else if (X < -(0.5f * 3.14159265358979323846f))
    {
        X = -(0.5f * 3.14159265358979323846f) - ((0.5f * 3.14159265358979323846f) + X);
    }
    float XFLOAT = X * X;
    return X + X * XFLOAT * (-1.666665710e-1f + XFLOAT * (8.333017292e-3f + XFLOAT * (-1.980661520e-4f + XFLOAT * 2.600054768e-6f)));
}

float Fast_Cosine(float X)
{
    return Fast_Sine(X + (0.5f * 3.14159265358979323846f));
}

float Fast_Atan2(float Y, float X)
{
    float Result, ABSOfX, ABSOfY;
    ABSOfX = ABS(X);
    ABSOfY = ABS(Y);
    Result = MAX(ABSOfX, ABSOfY);
    if (Result)
    {
        Result = MIN(ABSOfX, ABSOfY) / Result;
    }
    else
    {
        Result = 0.0f;
    }
    Result = -((((0.05030176425872175f * Result - 0.3099814292351353f) * Result - 0.14744007058297684f) * Result - 0.99997356613987f) *
                   Result -
               3.14551665884836e-07f) /
             ((0.6444640676891548f * Result + 0.1471039133652469f) * Result + 1.0f);
    if (ABSOfY > ABSOfX)
    {
        Result = (3.14159265358979323846f / 2.0f) - Result;
    }
    if (X < 0)
    {
        Result = 3.14159265358979323846f - Result;
    }
    if (Y < 0)
    {
        Result = -Result;
    }
    return Result;
}

float Fast_AtanCosine(float X)
{
    float ABSFloatOfX = ABS(X);
    float Result = Fast_SquareRoot(1.0f - ABSFloatOfX) * (1.5707288f + ABSFloatOfX * (-0.2121144f + ABSFloatOfX * (0.0742610f + (-0.0187293f * ABSFloatOfX))));
    if (X < 0.0f)
    {
        return 3.14159265358979323846f - Result;
    }
    else
    {
        return Result;
    }
}

float Fast_Tangent(float InputValue)
{
    return Fast_Sine(InputValue) / Fast_Cosine(InputValue);
}