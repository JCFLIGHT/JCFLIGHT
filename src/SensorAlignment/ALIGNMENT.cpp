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

#include "ALIGNMENT.h"
#include "Math/AVRMATH.h"
#include "Common/STRUCTS.h"

#define sin_approx(x) sinf(x)
#define cos_approx(x) cosf(x)

typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
} Angles_Struct;

typedef union
{
    float RawAngles[3];
    Angles_Struct Angles;
} Union_Angles_Struct;

typedef struct
{
    float Matrix3x3[3][3];
} Matrix3x3_Struct;

typedef union
{
    int16_t Vector[3];
    struct
    {
        int16_t Roll;
        int16_t Pitch;
        int16_t Yaw;
    };
} Vector3x3_Struct;

static Matrix3x3_Struct SensorRotationMatrix;

static bool SensorAlignmentInit = false;
static bool WithoutSensorAlignment = true;

//VARIAVEIS DE USO EXTERNO PARA SETAR NOVOS VALORES
int16_t RollDeciDegrees = 0;
int16_t PitchDeciDegrees = 0;
int16_t YawDeciDegrees = 0;

static inline Vector3x3_Struct *RotationMatrixRotateVector(Vector3x3_Struct *Result, const Vector3x3_Struct *VectorPointer, const Matrix3x3_Struct *RotationMath)
{
    Vector3x3_Struct CalcedResult;
    CalcedResult.Roll = RotationMath->Matrix3x3[0][0] * VectorPointer->Roll + RotationMath->Matrix3x3[1][0] * VectorPointer->Pitch + RotationMath->Matrix3x3[2][0] * VectorPointer->Yaw;
    CalcedResult.Pitch = RotationMath->Matrix3x3[0][1] * VectorPointer->Roll + RotationMath->Matrix3x3[1][1] * VectorPointer->Pitch + RotationMath->Matrix3x3[2][1] * VectorPointer->Yaw;
    CalcedResult.Yaw = RotationMath->Matrix3x3[0][2] * VectorPointer->Roll + RotationMath->Matrix3x3[1][2] * VectorPointer->Pitch + RotationMath->Matrix3x3[2][2] * VectorPointer->Yaw;
    *Result = CalcedResult;
    return Result;
}

void RotationMatrixFromAngles(Matrix3x3_Struct *RotationMath, const Union_Angles_Struct *Angles)
{
    float CosX;
    float SinX;
    float CosY;
    float SinY;
    float CosZ;
    float SinZ;
    float CosZCosX;
    float SinZCosX;
    float CosZSinX;
    float SinZSinX;

    CosX = cos_approx(Angles->Angles.Roll);
    SinX = sin_approx(Angles->Angles.Roll);
    CosY = cos_approx(Angles->Angles.Pitch);
    SinY = sin_approx(Angles->Angles.Pitch);
    CosZ = cos_approx(Angles->Angles.Yaw);
    SinZ = sin_approx(Angles->Angles.Yaw);

    CosZCosX = CosZ * CosX;
    SinZCosX = SinZ * CosX;
    CosZSinX = SinX * CosZ;
    SinZSinX = SinX * SinZ;

    RotationMath->Matrix3x3[0][ROLL] = CosZ * CosY;
    RotationMath->Matrix3x3[0][PITCH] = -CosY * SinZ;
    RotationMath->Matrix3x3[0][YAW] = SinY;
    RotationMath->Matrix3x3[1][ROLL] = SinZCosX + (CosZSinX * SinY);
    RotationMath->Matrix3x3[1][PITCH] = CosZCosX - (SinZSinX * SinY);
    RotationMath->Matrix3x3[1][YAW] = -SinX * CosY;
    RotationMath->Matrix3x3[2][ROLL] = (SinZSinX) - (CosZCosX * SinY);
    RotationMath->Matrix3x3[2][PITCH] = (CosZSinX) + (SinZCosX * SinY);
    RotationMath->Matrix3x3[2][YAW] = CosY * CosX;
}

static bool CheckValidSensorAlignment()
{
    return !RollDeciDegrees && !PitchDeciDegrees && !YawDeciDegrees;
}

void UpdateSensorAlignment(void)
{
    if (CheckValidSensorAlignment())
    {
        WithoutSensorAlignment = true;
    }
    else
    {
        Union_Angles_Struct RotationAngles;
        RotationAngles.Angles.Roll = ConvertDeciDegreesToRadians(RollDeciDegrees);
        RotationAngles.Angles.Pitch = ConvertDeciDegreesToRadians(PitchDeciDegrees);
        RotationAngles.Angles.Yaw = ConvertDeciDegreesToRadians(YawDeciDegrees);
        RotationMatrixFromAngles(&SensorRotationMatrix, &RotationAngles);
        WithoutSensorAlignment = false;
    }
}

void ApplySensorAlignment(int16_t *Vector)
{
    if (!SensorAlignmentInit)
    {
        UpdateSensorAlignment();
        SensorAlignmentInit = true;
    }
    if (WithoutSensorAlignment)
        return;
    Vector3x3_Struct IntVector = {.Vector = {Vector[ROLL], Vector[PITCH], Vector[YAW]}};
    RotationMatrixRotateVector(&IntVector, &IntVector, &SensorRotationMatrix);
    Vector[ROLL] = lrint(IntVector.Roll);
    Vector[PITCH] = lrint(IntVector.Pitch);
    Vector[YAW] = lrint(IntVector.Yaw);
}