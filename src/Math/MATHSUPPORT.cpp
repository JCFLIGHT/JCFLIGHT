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

#include "MATHSUPPORT.h"
#include "IMU/ACCGYROREAD.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

float Constrain_Float(float ValueInput, float ValueInputMin, float ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

int8_t Constrain_8Bits(int8_t ValueInput, int8_t ValueInputMin, int8_t ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

uint8_t Constrain_U8Bits(uint8_t ValueInput, uint8_t ValueInputMin, uint8_t ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

int16_t Constrain_16Bits(int16_t ValueInput, int16_t ValueInputMin, int16_t ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

uint16_t Constrain_U16Bits(uint16_t ValueInput, uint16_t ValueInputMin, uint16_t ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

int32_t Constrain_32Bits(int32_t ValueInput, int32_t ValueInputMin, int32_t ValueInputMax)
{
    return ((ValueInput) < (ValueInputMin) ? (ValueInputMin) : ((ValueInput) > (ValueInputMax) ? (ValueInputMax) : (ValueInput)));
}

float ScaleRangeFloat(float Value, float MinInputValue, float MaxInputValue, float MinOutputValue, float MaxOutputValue)
{
    float ValA = (MaxOutputValue - MinOutputValue) * (Value - MinInputValue);
    float ValB = MaxInputValue - MinInputValue;
    return ((ValA / ValB) + MinOutputValue);
}

int16_t ScaleRange16Bits(int16_t Value, int16_t MinInputValue, int16_t MaxInputValue, int16_t MinOutputValue, int16_t MaxOutputValue)
{
    int16_t ValA = ((int16_t)MaxOutputValue - (int16_t)MinOutputValue) * ((int16_t)Value - (int16_t)MinInputValue);
    int16_t ValB = (int16_t)MaxInputValue - (int16_t)MinInputValue;
    return ((ValA / ValB) + MinOutputValue);
}

int32_t ScaleRange32Bits(int32_t Value, int32_t MinInputValue, int32_t MaxInputValue, int32_t MinOutputValue, int32_t MaxOutputValue)
{
    int32_t ValA = ((int32_t)MaxOutputValue - (int32_t)MinOutputValue) * ((int32_t)Value - (int32_t)MinInputValue);
    int32_t ValB = (int32_t)MaxInputValue - (int32_t)MinInputValue;
    return ((ValA / ValB) + MinOutputValue);
}

float SquareFloat(float InputValue)
{
    return InputValue * InputValue;
}

int32_t Square32Bits(int32_t InputValue)
{
    return InputValue * InputValue;
}

float ConvertToDegrees(float InputValue)
{
    return InputValue * 57.2957795131f;
}

float ConvertToRadians(float InputValue)
{
    return InputValue * 0.01745329252f;
}

float ConvertRadiansToDeciDegrees(float Inputvalue)
{
    return ((Inputvalue * 10.0f) / 0.01745329251994329576923690768489f);
}

float ConvertDeciDegreesToRadians(float Inputvalue)
{
    return ((Inputvalue / 10.0f) * 0.01745329251994329576923690768489f);
}

float ConvertDeciDegreesToDegrees(float Inputvalue)
{
    return (Inputvalue / 10);
}

float ConvertDegreesToDecidegrees(float Inputvalue)
{
    return (Inputvalue * 10);
}

float ConvertCoordinateToFloatingPoint(int32_t CoordinateInput)
{
    return CoordinateInput * 1.0e-7f;
}

float ConvertAccelerationToCMSS(float InputAcc)
{
    return InputAcc * (GRAVITY_CMSS / ACC_1G);
}

int32_t ConvertCMToMeters(int32_t CM_Input)
{
    return CM_Input * 100;
}

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

uint16_t SquareRootU16Bits(uint16_t ValueInput)
{
    uint16_t Operation = ValueInput;
    uint16_t Result = 0;
    uint16_t OperationType = 1U << 14;
    while (OperationType > Operation)
    {
        OperationType >>= 2;
    }
    while (OperationType != 0)
    {
        if (Operation >= Result + OperationType)
        {
            Operation = Operation - (Result + OperationType);
            Result = Result + 2 * OperationType;
        }
        Result >>= 1;
        OperationType >>= 2;
    }
    return Result;
}

uint32_t SquareRootU32Bits(uint32_t ValueInput)
{
    uint32_t Operation = ValueInput;
    uint32_t Result = 0;
    uint32_t OperationType = 1UL << 30;
    while (OperationType > Operation)
    {
        OperationType >>= 2;
    }
    while (OperationType != 0)
    {
        if (Operation >= Result + OperationType)
        {
            Operation = Operation - (Result + OperationType);
            Result = Result + 2 * OperationType;
        }
        Result >>= 1;
        OperationType >>= 2;
    }
    return Result;
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

float Sine_Curve(const float InputValue, const float CurveWidth)
{
    return Fast_Pow(2.71828182845904523536f, -SquareFloat(InputValue) / (2.0f * SquareFloat(CurveWidth)));
}

float Fast_Tangent(float InputValue)
{
    return Fast_Sine(InputValue) / Fast_Cosine(InputValue);
}

int32_t WRap_18000(int32_t AngleInput)
{
    if (AngleInput > 18000)
    {
        AngleInput -= 36000;
    }
    if (AngleInput < -18000)
    {
        AngleInput += 36000;
    }
    return AngleInput;
}

float Fast_Exponential(float InputValue)
{
    union
    {
        int32_t i;
        float f;
    } xu, xu2;

    float val2, val3, val4, b;
    int32_t val4i;
    val2 = 12102203.1615614f * InputValue + 1065353216.f;
    val3 = val2 < 2139095040.f ? val2 : 2139095040.f;
    val4 = val3 > 0.f ? val3 : 0.f;
    val4i = (int32_t)val4;
    xu.i = val4i & 0x7F800000;
    xu2.i = (val4i & 0x7FFFFF) | 0x3F800000;
    b = xu2.f;
    return xu.f * (0.509871020343597804469416f + b * (0.312146713032169896138863f + b * (0.166617139319965966118107f + b * (-2.19061993049215080032874e-3f + b * 1.3555747234758484073940937e-2f))));
}

float Fast_Logarithm(float InputValue)
{
    union
    {
        float f;
        int32_t i;
    } valu;

    float exp, addcst, x;
    valu.f = InputValue;
    exp = valu.i >> 23;
    addcst = InputValue > 0 ? -89.970756366f : -(float)__builtin_inf();
    valu.i = (valu.i & 0x7FFFFF) | 0x3F800000;
    x = valu.f;
    return x * (3.529304993f + x * (-2.461222105f + x * (1.130626167f + x * (-0.288739945f + x * 3.110401639e-2f)))) + (addcst + 0.69314718055995f * exp);
}

float Fast_Pow(float ValueA, float ValueB) //X^Y
{
    return Fast_Exponential(ValueB * Fast_Logarithm(ValueA));
}

float Power_Float(float Base, int16_t Exponential) //X^Y
{
    float Result = Base;
    for (int16_t Count = 1; Count < Exponential; Count++)
    {
        Result *= Base;
    }
    return Result;
}

float ConvertCentimeterPerSecondsToKmPerHour(float Value)
{
    return Value / 27.778f;
}