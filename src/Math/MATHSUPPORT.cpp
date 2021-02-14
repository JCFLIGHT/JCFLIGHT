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
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

float ABS_FLOAT(float X)
{
    return ((X) > 0 ? (X) : -(X));
}

int16_t ABS_16BITS(int16_t X)
{
    return ((X) > 0 ? (X) : -(X));
}

int32_t ABS_32BITS(int32_t X)
{
    return ((X) > 0 ? (X) : -(X));
}

float MIN_FLOAT(float X, float Y)
{
    return ((X) < (Y) ? (X) : (Y));
}

float MAX_FLOAT(float X, float Y)
{
    return ((X) > (Y) ? (X) : (Y));
}

int8_t MIN_8BITS(int8_t X, int8_t Y)
{
    return ((X) < (Y) ? (X) : (Y));
}

uint16_t MIN_U16BITS(uint16_t X, uint16_t Y)
{
    return ((X) < (Y) ? (X) : (Y));
}

uint32_t MIN_U32BITS(uint32_t X, uint32_t Y)
{
    return ((X) < (Y) ? (X) : (Y));
}

uint16_t MAX_U16BITS(uint16_t X, uint16_t Y)
{
    return ((X) > (Y) ? (X) : (Y));
}

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

float Map_Float(float Value, float MinInputValue, float MaxInputValue, float MinOutputValue, float MaxOutputValue)
{
    return (Value - MinInputValue) * (MaxOutputValue - MinOutputValue) / (MaxInputValue - MinInputValue) + MinOutputValue;
}

int16_t Map_16Bits(int16_t Value, int16_t MinInputValue, int16_t MaxInputValue, int16_t MinOutputValue, int16_t MaxOutputValue)
{
    return (Value - MinInputValue) * (MaxOutputValue - MinOutputValue) / (MaxInputValue - MinInputValue) + MinOutputValue;
}

int32_t Map_32Bits(int32_t Value, int32_t MinInputValue, int32_t MaxInputValue, int32_t MinOutputValue, int32_t MaxOutputValue)
{
    return (Value - MinInputValue) * (MaxOutputValue - MinOutputValue) / (MaxInputValue - MinInputValue) + MinOutputValue;
}

float SquareFloat(float InputValue)
{
    return InputValue * InputValue;
}

int32_t Square32Bits(int32_t InputValue)
{
    return InputValue * InputValue;
}

float ConvetToDegrees(float InputValue)
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
    ABSOfX = ABS_FLOAT(X);
    ABSOfY = ABS_FLOAT(Y);
    Result = MAX_FLOAT(ABSOfX, ABSOfY);
    if (Result)
    {
        Result = MIN_FLOAT(ABSOfX, ABSOfY) / Result;
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
    float ABSFloatOfX = ABS_FLOAT(X);
    float Result = sqrtf(1.0f - ABSFloatOfX) * (1.5707288f + ABSFloatOfX * (-0.2121144f + ABSFloatOfX * (0.0742610f + (-0.0187293f * ABSFloatOfX))));
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
    return pow(2.71828182845904523536f, -SquareFloat(InputValue) / (2.0f * SquareFloat(CurveWidth)));
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