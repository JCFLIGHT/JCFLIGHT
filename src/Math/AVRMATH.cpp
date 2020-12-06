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

#include "AVRMATH.h"

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

//ESSA FUNÇÃO GASTA 199us PARA CALCULAR O RESULTADO
int16_t ApproximationOfAtan2(int16_t AccRoll, int16_t AccYaw)
{
    float Difference = AccRoll;
    int16_t Ata2Result;
    uint8_t CalculateABS;
    CalculateABS = ABS_16BITS(AccRoll) < ABS_16BITS(AccYaw);
    if (CalculateABS)
        Difference = Difference / AccYaw;
    else
        Difference = AccYaw / Difference;
    Ata2Result = 2046.43f * (Difference / (3.5714f + Difference * Difference));
    if (CalculateABS)
    {
        if (AccYaw < 0)
        {
            if (AccRoll < 0)
                Ata2Result -= 1800;
            else
                Ata2Result += 1800;
        }
    }
    else
    {
        Ata2Result = 900 - Ata2Result;
        if (AccRoll < 0)
            Ata2Result -= 1800;
    }
    return Ata2Result;
}

static const uint16_t TableOfSineApprox[91] __attribute__((__progmem__)) = {0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174, 191, 208, 225, 242, 259, 276, 292, 309, 326, 342, 358, 375,
                                                                            391, 407, 423, 438, 454, 469, 485, 500, 515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682,
                                                                            695, 707, 719, 731, 743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891, 899,
                                                                            906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974, 978, 982, 985, 988, 990, 993, 995, 996,
                                                                            998, 999, 999, 1000, 1000};

float Calculate_Sine_Approx(int16_t InputAngle)
{
    int8_t InvertValue, InvertValueTwo;
    int16_t Calculate_Sine_Value;
    if (InputAngle < 0)
    {
        InvertValue = -1;
        InputAngle = -InputAngle;
    }
    else
    {
        InvertValue = 1;
    }
    if (ABS_16BITS(InputAngle) >= 3600)
    {
        InputAngle %= 3600;
    }
    if (InputAngle <= 900)
    {
        InvertValueTwo = 1;
    }
    else if ((InputAngle > 900) && (InputAngle <= 1800))
    {
        InputAngle = 1800 - InputAngle;
        InvertValueTwo = 1;
    }
    else if ((InputAngle > 1800) && (InputAngle <= 2700))
    {
        InputAngle = InputAngle - 1800;
        InvertValueTwo = -1;
    }
    else
    {
        InputAngle = 3600 - InputAngle;
        InvertValueTwo = -1;
    }
    if (InputAngle < 105)
    {
        return ((float)(InputAngle * InvertValue * InvertValueTwo)) * 0.001745329252f;
    }
    Calculate_Sine_Value = pgm_read_word(&TableOfSineApprox[InputAngle / 10]);
    return (float)(Calculate_Sine_Value * InvertValue * InvertValueTwo) / 1000;
}

float Calculate_Cosine_Approx(int16_t InputAngle)
{
    return (Calculate_Sine_Approx(900 - InputAngle));
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

float InvertSquareRootFloat(float InputValue)
{
    union
    {
        int32_t IntegrerValue;
        float FloatValue;
    } Convert;
    Convert.FloatValue = InputValue;
    Convert.IntegrerValue = 0x5f1ffff9 - (Convert.IntegrerValue >> 1);
    return Convert.FloatValue * (1.68191409f - 0.703952253f * InputValue * Convert.FloatValue * Convert.FloatValue);
}
