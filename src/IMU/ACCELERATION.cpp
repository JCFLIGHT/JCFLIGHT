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

#include "ACCELERATION.h"
#include "Common/VARIABLES.h"
#include "Math/MATHSUPPORT.h"
#include "FastSerial/PRINTF.h"

//#define DEBUG_ACCELERATION

float Cosine_Yaw;
float Sine_Yaw;
float AccelerationAdjustBias[3];
float AccelerationEarthFrame_LPF[3];
float AccelerationDifference[3];

void EarthFrame_Calculate_AccelerationXYZ()
{
  float Cosine_Roll;
  float Sine_Roll;
  float Cosine_Pitch;
  float Sine_Pitch;
  float SinePitch_CosineYaw_Fusion;
  float SinePitch_SineYaw_Fusion;

  //CALCULA O SENO E COSSENO DE CADA EIXO DA ESTIMAÇÃO DE ATTITUDE DADO PELO AHRS
  Cosine_Roll = Fast_Cosine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL]));
  Sine_Roll = Fast_Sine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL]));
  Cosine_Pitch = Fast_Cosine(ConvertDeciDegreesToRadians(-ATTITUDE.AngleOut[PITCH]));
  Sine_Pitch = Fast_Sine(ConvertDeciDegreesToRadians(-ATTITUDE.AngleOut[PITCH]));
  Cosine_Yaw = Fast_Cosine(ConvertDeciDegreesToRadians(ATTITUDE.CompassHeading));
  Sine_Yaw = Fast_Sine(ConvertDeciDegreesToRadians(ATTITUDE.CompassHeading));

#ifdef DEBUG_ACCELERATION

  FastSerialPrintln(PSTR("Cosine_Roll:%.4f Sine_Roll:%.4f ATTITUDE.AngleOut[ROLL]:%.4f\n"),
                    Cosine_Roll,
                    Sine_Roll,
                    ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL]));

/*
  FastSerialPrintln(PSTR("Cosine_Pitch:%.4f Sine_Pitch:%.4f\n"),
                    Cosine_Pitch,
                    Sine_Pitch);
*/

/*
  FastSerialPrintln(PSTR("Cosine_Yaw:%.4f Sine_Yaw:%.4f\n"),
                    Cosine_Yaw,
                    Sine_Yaw);
*/
#endif

  //REALIZA A FUSÃO DE ALGUMAS VARIAVEIS PARA OBTER DIFERENTES VALORES DE ACELERAÇÃO EM TORNO DO EIXO Z
  SinePitch_CosineYaw_Fusion = Sine_Pitch * Cosine_Yaw;
  SinePitch_SineYaw_Fusion = Sine_Pitch * Sine_Yaw;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = -((Cosine_Pitch * Cosine_Yaw) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * SinePitch_CosineYaw_Fusion - Cosine_Roll * Sine_Yaw) * IMU.AccelerometerRead[ROLL] + (Sine_Roll * Sine_Yaw + Cosine_Roll * SinePitch_CosineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = -((Cosine_Pitch * Sine_Yaw) * IMU.AccelerometerRead[PITCH] + (Cosine_Roll * Cosine_Yaw + Sine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[ROLL] + (-Sine_Roll * Cosine_Yaw + Cosine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //YAW
  INS.AccelerationEarthFrame[YAW] = ((-Sine_Pitch) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[ROLL] + (Cosine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[YAW]) - 512;

  //ROLL
  INS.AccelerationEarthFrame[ROLL] = INS.AccelerationEarthFrame[ROLL] * 1.915361328125f;
  AccelerationDifference[ROLL] = INS.AccelerationEarthFrame[ROLL] - AccelerationAdjustBias[ROLL];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[ROLL] = AccelerationAdjustBias[ROLL] * 0.985f + INS.AccelerationEarthFrame[ROLL] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[ROLL]) <= 80.0f)
  {
    AccelerationAdjustBias[ROLL] = AccelerationAdjustBias[ROLL] * 0.9987f + INS.AccelerationEarthFrame[ROLL] * 0.0013f;
  }
  INS.AccelerationEarthFrame[ROLL] = AccelerationDifference[ROLL];
  AccelerationEarthFrame_LPF[ROLL] = AccelerationEarthFrame_LPF[ROLL] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[ROLL] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[ROLL] += AccelerationEarthFrame_LPF[ROLL];
  INS.AccelerationEarthFrame_Sum_Count[ROLL]++;

  //PITCH
  INS.AccelerationEarthFrame[PITCH] = INS.AccelerationEarthFrame[PITCH] * 1.915361328125f;
  AccelerationDifference[PITCH] = INS.AccelerationEarthFrame[PITCH] - AccelerationAdjustBias[PITCH];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[PITCH] = AccelerationAdjustBias[PITCH] * 0.985f + INS.AccelerationEarthFrame[PITCH] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[PITCH]) <= 80.0f)
  {
    AccelerationAdjustBias[PITCH] = AccelerationAdjustBias[PITCH] * 0.9987f + INS.AccelerationEarthFrame[PITCH] * 0.0013f;
  }
  INS.AccelerationEarthFrame[PITCH] = AccelerationDifference[PITCH];
  AccelerationEarthFrame_LPF[PITCH] = AccelerationEarthFrame_LPF[PITCH] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[PITCH] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[PITCH] += AccelerationEarthFrame_LPF[PITCH];
  INS.AccelerationEarthFrame_Sum_Count[PITCH]++;

  //YAW
  INS.AccelerationEarthFrame[YAW] = INS.AccelerationEarthFrame[YAW] * 1.915361328125f;
  AccelerationDifference[YAW] = INS.AccelerationEarthFrame[YAW] - AccelerationAdjustBias[YAW];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[YAW] = AccelerationAdjustBias[YAW] * 0.985f + INS.AccelerationEarthFrame[YAW] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[YAW]) <= 80.0f)
  {
    AccelerationAdjustBias[YAW] = AccelerationAdjustBias[YAW] * 0.9987f + INS.AccelerationEarthFrame[YAW] * 0.0013f;
  }
  INS.AccelerationEarthFrame[YAW] = AccelerationDifference[YAW];
  AccelerationEarthFrame_LPF[YAW] = AccelerationEarthFrame_LPF[YAW] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[YAW] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[YAW] += AccelerationEarthFrame_LPF[YAW];
  INS.AccelerationEarthFrame_Sum_Count[YAW]++;
}
