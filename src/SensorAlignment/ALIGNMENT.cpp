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
#include "Math/MATHSUPPORT.h"
#include "Common/STRUCTS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

static Matrix3x3_Struct SensorRotationMatrix;

static bool SensorAlignmentInit = false;
static bool WithoutSensorAlignment = true;

int16_t RollDeciDegrees = 0;
int16_t PitchDeciDegrees = 0;
int16_t YawDeciDegrees = 0;

static inline Vector3x3_Struct *MatrixRotationToVector(Vector3x3_Struct *Result, const Vector3x3_Struct *VectorPointer, const Matrix3x3_Struct *RotationMath)
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

    CosX = Fast_Cosine(Angles->Angles.Roll);
    SinX = Fast_Sine(Angles->Angles.Roll);
    CosY = Fast_Cosine(Angles->Angles.Pitch);
    SinY = Fast_Sine(Angles->Angles.Pitch);
    CosZ = Fast_Cosine(Angles->Angles.Yaw);
    SinZ = Fast_Sine(Angles->Angles.Yaw);

    CosZCosX = CosZ * CosX;
    SinZCosX = SinZ * CosX;
    CosZSinX = SinX * CosZ;
    SinZSinX = SinX * SinZ;

    RotationMath->Matrix3x3[0][0] = CosZ * CosY;
    RotationMath->Matrix3x3[0][1] = -CosY * SinZ;
    RotationMath->Matrix3x3[0][2] = SinY;
    RotationMath->Matrix3x3[1][0] = SinZCosX + (CosZSinX * SinY);
    RotationMath->Matrix3x3[1][1] = CosZCosX - (SinZSinX * SinY);
    RotationMath->Matrix3x3[1][2] = -SinX * CosY;
    RotationMath->Matrix3x3[2][0] = (SinZSinX) - (CosZCosX * SinY);
    RotationMath->Matrix3x3[2][1] = (CosZSinX) + (SinZCosX * SinY);
    RotationMath->Matrix3x3[2][2] = CosY * CosX;
}

static bool CheckValidSensorAlignment()
{
    return !RollDeciDegrees && !PitchDeciDegrees && !YawDeciDegrees;
}

static void UpdateSensorAlignment(void)
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
    if (RollDeciDegrees != STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADJUST_ADDR) ||
        PitchDeciDegrees != STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADJUST_ADDR) ||
        YawDeciDegrees != STORAGEMANAGER.Read_16Bits(ACC_YAW_ADJUST_ADDR))
    {
        RollDeciDegrees = STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADJUST_ADDR);
        PitchDeciDegrees = STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADJUST_ADDR);
        YawDeciDegrees = STORAGEMANAGER.Read_16Bits(ACC_YAW_ADJUST_ADDR);
        SensorAlignmentInit = false;
    }
    if (!SensorAlignmentInit)
    {
        UpdateSensorAlignment();
        SensorAlignmentInit = true;
    }
    if (WithoutSensorAlignment)
    {
        return;
    }
    Vector3x3_Struct IntVector = {.Vector = {Vector[ROLL], Vector[PITCH], Vector[YAW]}};
    MatrixRotationToVector(&IntVector, &IntVector, &SensorRotationMatrix);
    Vector[ROLL] = IntVector.Roll;
    Vector[PITCH] = IntVector.Pitch;
    Vector[YAW] = IntVector.Yaw;
}