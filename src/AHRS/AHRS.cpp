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

#include "AHRS.h"
#include "Common/STRUCTS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "I2C/I2C.h"
#include "BAR/BAR.h"
#include "Yaw/HEADINGHOLD.h"
#include "QUATERNION.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "GPS/GPSSTATES.h"
#include "GPS/GPSUBLOX.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

AHRSClass AHRS;

#define SPIN_RATE_LIMIT 20    //VALOR DE GYRO^2 PARA CORTAR A CORREÇÃO DO INTEGRAL NO AHRS
#define MAX_ACC_NEARNESS 0.33 //33% (0.67G - 1.33G)
#ifdef __AVR_ATmega2560__
#define NEARNESS 100.0f //FATOR DE GANHO DE CORREÇÃO DO ACELEROMETRO NO AHRS
#else
#define NEARNESS 1.0f //FATOR DE GANHO DE CORREÇÃO DO ACELEROMETRO NO AHRS
#endif
#define ACC_1G 512.0f             //1G DA IMU - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GYRO_SCALE (1.0f / 16.4f) //16.4 - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GRAVITY_CMSS 980.665f     //VALOR DA GRAVIDADE EM CM/S^2

Struct_Vector3x3 BodyFrameAcceleration;
Struct_Vector3x3 BodyFrameRotation;
static Struct_Vector3x3 CorrectedMagneticFieldNorth;

Struct_Quaternion Orientation;

static Struct_IMURuntimeConfiguration IMURuntimeConfiguration;

static bool GPSHeadingInitialized = false;

float RotationMath[3][3];

static void ComputeRotationMatrix(void)
{
  float q1q1 = Orientation.q1 * Orientation.q1;
  float q2q2 = Orientation.q2 * Orientation.q2;
  float q3q3 = Orientation.q3 * Orientation.q3;
  float q0q1 = Orientation.q0 * Orientation.q1;
  float q0q2 = Orientation.q0 * Orientation.q2;
  float q0q3 = Orientation.q0 * Orientation.q3;
  float q1q2 = Orientation.q1 * Orientation.q2;
  float q1q3 = Orientation.q1 * Orientation.q3;
  float q2q3 = Orientation.q2 * Orientation.q3;
  RotationMath[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  RotationMath[0][1] = 2.0f * (q1q2 + -q0q3);
  RotationMath[0][2] = 2.0f * (q1q3 - -q0q2);
  RotationMath[1][0] = 2.0f * (q1q2 - -q0q3);
  RotationMath[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  RotationMath[1][2] = 2.0f * (q2q3 + -q0q1);
  RotationMath[2][0] = 2.0f * (q1q3 + -q0q2);
  RotationMath[2][1] = 2.0f * (q2q3 - -q0q1);
  RotationMath[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void AHRSClass::Initialization(void)
{
  //CALCULA A DECLINAÇÃO MAGNETICA
  const int16_t Degrees = ((int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100)) / 100;
  const int16_t Remainder = ((int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100)) % 100;
  const float CalcedvalueInRadians = -ConvertToRadians(Degrees + Remainder / 60.0f);
  CorrectedMagneticFieldNorth.Roll = Fast_Cosine(CalcedvalueInRadians);
  CorrectedMagneticFieldNorth.Pitch = Fast_Sine(CalcedvalueInRadians);
  CorrectedMagneticFieldNorth.Yaw = 0;
  //RESETA O QUATERNION E A MATRIX
  QuaternionInitUnit(&Orientation);
  ComputeRotationMatrix();
}

static bool ValidateQuaternion(const Struct_Quaternion *Quaternion)
{
  const float CheckAbsoluteValue = ABS(Quaternion->q0) +
                                   ABS(Quaternion->q1) +
                                   ABS(Quaternion->q2) +
                                   ABS(Quaternion->q3);
  if (!isnan(CheckAbsoluteValue) && !isinf(CheckAbsoluteValue))
  {
    return true;
  }
  const float QuatSquared = QuaternionNormalizedSquared(&Orientation);
  if (QuatSquared > (1.0f - 1e-6f) && QuatSquared < (1.0f + 1e-6f))
  {
    return true;
  }
  return false;
}

static void ResetOrientationQuaternion(const Struct_Vector3x3 *AccelerationBodyFrame)
{
  const float AccVectorSquared = Fast_SquareRoot(VectorNormSquared(AccelerationBodyFrame));
  Orientation.q0 = AccelerationBodyFrame->Yaw + AccVectorSquared;
  Orientation.q1 = AccelerationBodyFrame->Pitch;
  Orientation.q2 = -AccelerationBodyFrame->Roll;
  Orientation.q3 = 0.0f;
  QuaternionNormalize(&Orientation, &Orientation);
}

static void CheckAndResetOrientationQuaternion(const Struct_Quaternion *Quaternion,
                                               const Struct_Vector3x3 *AccelerationBodyFrame)
{
  //CHECA SE O QUATERNION ESTÁ NORMAL
  if (ValidateQuaternion(&Orientation))
  {
    return;
  }
  //ORIENTAÇÃO INVALIDA,É NECESSARIO RESETAR O QUATERNION
  if (ValidateQuaternion(Quaternion))
  {
    //OBTÉM O VALOR ANTERIOR VALIDO
    Orientation = *Quaternion;
  }
  else
  {
    //REFERENCIA INVALIDA,O ACELEROMETRO PODE ESTAR RUIM
    ResetOrientationQuaternion(AccelerationBodyFrame);
  }
}

static void MahonyAHRSUpdate(float DeltaTime,
                             const Struct_Vector3x3 *RotationBodyFrame,
                             const Struct_Vector3x3 *AccelerationBodyFrame,
                             const Struct_Vector3x3 *MagnetometerBodyFrame,
                             bool GPS_HeadingState, float CourseOverGround,
                             float AccelerometerWeightScaler,
                             float MagnetometerWeightScaler)
{
  static Struct_Vector3x3 GyroDriftEstimate = {0};

  Struct_Quaternion PreviousOrientation = Orientation;
  Struct_Vector3x3 RotationRate = *RotationBodyFrame;

  //CALCULA O VALOR DO SPIN RATE EM RADIANOS/S
  const float Spin_Rate_Square = VectorNormSquared(&RotationRate);

  //CORREÇÃO DO ROLL E PITCH USANDO O VETOR DO ACELEROMETRO
  if (AccelerationBodyFrame)
  {
    static const Struct_Vector3x3 Gravity = {.Vector = {0.0f, 0.0f, 1.0f}};

    Struct_Vector3x3 EstimatedGravity;
    Struct_Vector3x3 AccelerationVector;
    Struct_Vector3x3 VectorError;

    //ESTIMA A GRAVIDADE NO BODY FRAME
    QuaternionRotateVector(&EstimatedGravity, &Gravity, &Orientation);

    //ESTIMA A DIREÇÃO DA GRAVIDADE
    VectorNormalize(&AccelerationVector, AccelerationBodyFrame);
    VectorCrossProduct(&VectorError, &AccelerationVector, &EstimatedGravity);

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (IMURuntimeConfiguration.kI_Accelerometer > 0.0f)
    {
      //DESATIVA A INTEGRAÇÃO SE O SPIN RATE ULTRAPASSAR UM CERTO VALOR
      if (Spin_Rate_Square < SquareFloat(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Struct_Vector3x3 OldVector;
        //CALCULA O ERRO ESCALADO POR Ki
        VectorScale(&OldVector, &VectorError, IMURuntimeConfiguration.kI_Accelerometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }
    //CALCULA O GANHO DE Kp E APLICA O FEEDBACK PROPORCIONAL
    VectorScale(&VectorError, &VectorError, IMURuntimeConfiguration.kP_Accelerometer * AccelerometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //CORREÇÃO DO YAW
  if (MagnetometerBodyFrame || GPS_HeadingState)
  {
    static const Struct_Vector3x3 Forward = {.Vector = {1.0f, 0.0f, 0.0f}};

    Struct_Vector3x3 VectorError = {.Vector = {0.0f, 0.0f, 0.0f}};

    if (MagnetometerBodyFrame && VectorNormSquared(MagnetometerBodyFrame) > 0.01f)
    {
      Struct_Vector3x3 MagnetormeterVector;

      //CALCULA O NORTE MAGNETICO
      QuaternionRotateVectorInverse(&MagnetormeterVector, MagnetometerBodyFrame, &Orientation);

      //IGNORA A INCLINAÇÃO Z DO MAGNETOMETRO
      MagnetormeterVector.Yaw = 0.0f;

      //VERIFICA SE O MAGNETOMETRO^2 ESTÁ OK
      if (VectorNormSquared(&MagnetormeterVector) > 0.01f)
      {
        //NORMALIZA O VETOR DO MAGNETOMETRO
        VectorNormalize(&MagnetormeterVector, &MagnetormeterVector);

        //CALCULA A REFERENCIA DO COMPASS
        VectorCrossProduct(&VectorError, &MagnetormeterVector, &CorrectedMagneticFieldNorth);

        //CALCULA O ERRO DE ROTAÇÃO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }
    else if (GPS_HeadingState)
    {
      Struct_Vector3x3 HeadingEarthFrame;

      //USE O COG DO GPS PARA CALCULAR O HEADING
      while (CourseOverGround > 3.14159265358979323846f)
      {
        CourseOverGround -= (6.283185482f);
      }

      while (CourseOverGround < -3.14159265358979323846f)
      {
        CourseOverGround += (6.283185482f);
      }

      //CALCULA O VALOR DE HEADING COM BASE NO COG
      Struct_Vector3x3 CourseOverGroundVector = {.Vector = {-Fast_Cosine(CourseOverGround), Fast_Sine(CourseOverGround), 0.0f}};

      //ROTACIONA O VETOR DO BODY FRAME PARA EARTH FRAME
      QuaternionRotateVectorInverse(&HeadingEarthFrame, &Forward, &Orientation);
      HeadingEarthFrame.Yaw = 0.0f;

      //CORRIJA APENAS SE O SQUARE FOR POSITIVO E MAIOR QUE ZERO
      if (VectorNormSquared(&HeadingEarthFrame) > 0.01f)
      {
        //NORMALIZA O VETOR
        VectorNormalize(&HeadingEarthFrame, &HeadingEarthFrame);

        //CALCULA O ERRO
        VectorCrossProduct(&VectorError, &CourseOverGroundVector, &HeadingEarthFrame);

        //ROTACIONA O ERRO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (IMURuntimeConfiguration.kI_Magnetometer > 0.0f)
    {
      //PARE A INTEGRAÇÃO SE O SPIN RATE FOR MAIOR QUE O LIMITE
      if (Spin_Rate_Square < SquareFloat(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Struct_Vector3x3 OldVector;
        //CALCULA O ERRO INTEGRAL ESCALADO POR Ki
        VectorScale(&OldVector, &VectorError, IMURuntimeConfiguration.kI_Magnetometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }

    //CALCULA O GANHO DE Kp E APLICA O FEEDBACK DO PROPORCIONAL
    VectorScale(&VectorError, &VectorError, IMURuntimeConfiguration.kP_Magnetometer * MagnetometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //APLICA A CORREÇÃO DE DRIFT DO GYROSCOPIO
  VectorAdd(&RotationRate, &RotationRate, &GyroDriftEstimate);

  //TAXA DE MUDANÇA DO QUATERNION
  Struct_Vector3x3 Theta; //THETA É A ROTAÇÃO DE EIXO/ÂNGULO.
  Struct_Quaternion QuaternionDelta;

  VectorScale(&Theta, &RotationRate, 0.5f * DeltaTime);
  QuaternionInitFromVector(&QuaternionDelta, &Theta);
  const float ThetaSquared = VectorNormSquared(&Theta);

  //ATUALIZE O QUATERNION APENAS SE A ROTAÇÃO FOR MAIOR QUE ZERO
  if (ThetaSquared >= 1e-20)
  {
    //CONFORME THETA SE APROXIMA DE ZERO,AS OPERAÇÕES DE SENO E COSENO SE TORNAM CADA VEZ MAIS INSTAVEIS,
    //PARA CONTORNAR ISSO,USAMOS A SÉRIE DE TAYLOR,VERIFICAMOS SE O SQUARE DE THETA É MENOR QUE A PRECISÃO
    //FLOAT DO MICROCONTROLADOR (FLOAT = 24 BITS COM EXPONENTE DE 8 BITS),SENDO ASSIM SER POSSIVEL CALCULAR
    //COM SEGURANÇA O VALOR DE UM ÂNGULO PEQUENO SEM PERDER A PRECISÃO NUMÉRICA.
    if (ThetaSquared < Fast_SquareRoot(24.0f * 1e-6f))
    {
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, 1.0f - ThetaSquared / 6.0f);
      QuaternionDelta.q0 = 1.0f - ThetaSquared / 2.0f;
    }
    else
    {
      const float ThetaMagnitude = Fast_SquareRoot(ThetaSquared);
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, Fast_Sine(ThetaMagnitude) / ThetaMagnitude);
      QuaternionDelta.q0 = Fast_Cosine(ThetaMagnitude);
    }

    //CALCULA O VALOR FINA DA ORIENTAÇÃO E RENORMALIZA O QUATERNION
    QuaternionMultiply(&Orientation, &Orientation, &QuaternionDelta);
    QuaternionNormalize(&Orientation, &Orientation);
  }

  //CHECA SE O NOVO VALOR DO QUATERNION É VALIDO,SE SIM RESETA O VALOR ANTIGO
  CheckAndResetOrientationQuaternion(&PreviousOrientation, AccelerationBodyFrame);

  //CALCULA A MATRIX ROTATIVA PARA O QUATERNION
  ComputeRotationMatrix();
}

static float CalculateAccelerometerWeight()
{
  float AccelerometerMagnitudeSquare = 0;

  //CALCULA A RAIZ QUADRADA DE TODOS OS EIXOS DO ACELEROMETRO PARA EXTRAIR A MAGNITUDE
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[ROLL] / ACC_1G);
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[PITCH] / ACC_1G);
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[YAW] / ACC_1G);

  //CALCULA A CURVA DE SENO DA MAGNITUDE DO ACELEROMETRO
  const float AccWeight_Nearness = Sine_Curve(Fast_SquareRoot(AccelerometerMagnitudeSquare) - 1.0f, MAX_ACC_NEARNESS) * NEARNESS;

  return AccWeight_Nearness;
}

static void ComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
  if (initialRoll > 1800)
  {
    initialRoll -= 3600;
  }

  if (initialPitch > 1800)
  {
    initialPitch -= 3600;
  }

  if (initialYaw > 1800)
  {
    initialYaw -= 3600;
  }

  const float cosRoll = Fast_Cosine(ConvertDeciDegreesToRadians(initialRoll) * 0.5f);
  const float sinRoll = Fast_Sine(ConvertDeciDegreesToRadians(initialRoll) * 0.5f);

  const float cosPitch = Fast_Cosine(ConvertDeciDegreesToRadians(initialPitch) * 0.5f);
  const float sinPitch = Fast_Sine(ConvertDeciDegreesToRadians(initialPitch) * 0.5f);

  const float cosYaw = Fast_Cosine(ConvertDeciDegreesToRadians(-initialYaw) * 0.5f);
  const float sinYaw = Fast_Sine(ConvertDeciDegreesToRadians(-initialYaw) * 0.5f);

  Orientation.q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  Orientation.q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  Orientation.q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  Orientation.q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

  ComputeRotationMatrix();
}

void GetMeasuredAcceleration(Struct_Vector3x3 *MeasureAcceleration)
{
  MeasureAcceleration->Vector[ROLL] = ((float)IMU.AccelerometerRead[ROLL] / ACC_1G) * GRAVITY_CMSS;
  MeasureAcceleration->Vector[PITCH] = ((float)IMU.AccelerometerRead[PITCH] / ACC_1G) * GRAVITY_CMSS;
  MeasureAcceleration->Vector[YAW] = ((float)IMU.AccelerometerRead[YAW] / ACC_1G) * GRAVITY_CMSS;
}

void GetMeasuredRotationRate(Struct_Vector3x3 *MeasureRotation)
{
  MeasureRotation->Vector[ROLL] = ConvertToRadians(((float)IMU.GyroscopeRead[ROLL] * GYRO_SCALE));
  MeasureRotation->Vector[PITCH] = ConvertToRadians(((float)IMU.GyroscopeRead[PITCH] * GYRO_SCALE));
  MeasureRotation->Vector[YAW] = ConvertToRadians(((float)IMU.GyroscopeRead[YAW] * GYRO_SCALE));
}

void AHRSClass::Update(float DeltaTime)
{
  bool SafeToUseCompass = false;
  bool GPS_HeadingState = false;
  float CourseOverGround = 0;

  GetMeasuredRotationRate(&BodyFrameRotation);     //CALCULA A ROTAÇÃO DA IMU EM RADIANOS/S
  GetMeasuredAcceleration(&BodyFrameAcceleration); //CALCULA A ACELERAÇÃO DA IMU EM CM/S^2

  if (GetFrameStateOfAirPlane())
  {
    const bool SafeToUseCOG = (Get_GPS_In_Good_Condition() && GPS_Ground_Speed >= 300);

    if (I2C.CompassFound)
    {
      SafeToUseCompass = true;
      GPSHeadingInitialized = true;
    }
    else if (SafeToUseCOG)
    {
      if (GPSHeadingInitialized)
      {
        CourseOverGround = ConvertDeciDegreesToRadians(GPS_Ground_Course);
        GPS_HeadingState = true;
      }
      else
      {
        ComputeQuaternionFromRPY(ATTITUDE.AngleOut[ROLL], ATTITUDE.AngleOut[PITCH], GPS_Ground_Course);
        GPSHeadingInitialized = true;
      }
    }
  }
  else
  {
    if (I2C.CompassFound)
    {
      SafeToUseCompass = true;
    }
  }

  Struct_Vector3x3 MagnetometerBodyFrame = {.Vector = {(float)IMU.CompassRead[ROLL],
                                                       (float)IMU.CompassRead[PITCH],
                                                       (float)IMU.CompassRead[YAW]}};

  const float CalcedCompassWeight = NEARNESS;
  const float CalcedAccelerometerWeight = CalculateAccelerometerWeight();
  const bool SafeToUseAccelerometer = (CalcedAccelerometerWeight > 0.001f);

  //ATUALIZA O AHRS
  MahonyAHRSUpdate(DeltaTime, &BodyFrameRotation,
                   SafeToUseAccelerometer ? &BodyFrameAcceleration : NULL,
                   SafeToUseCompass ? &MagnetometerBodyFrame : NULL,
                   GPS_HeadingState, CourseOverGround,
                   CalcedAccelerometerWeight,
                   CalcedCompassWeight);

  //SAÍDA DOS EIXOS DO APÓS O AHRS
  //PITCH
  ATTITUDE.AngleOut[PITCH] = ConvertRadiansToDeciDegrees(-Fast_Atan2(RotationMath[2][1], RotationMath[2][2]));
  //ROLL
  ATTITUDE.AngleOut[ROLL] = ConvertRadiansToDeciDegrees((0.5f * 3.14159265358979323846f) - Fast_AtanCosine(-RotationMath[2][0]));
  //YAW
  ATTITUDE.CompassHeading = ConvertRadiansToDeciDegrees(-Fast_Atan2(RotationMath[1][0], RotationMath[0][0]));
  //CONVERTE O VALOR DE COMPASS HEADING PARA O VALOR ACEITAVEL
  if (ATTITUDE.CompassHeading < 0)
  {
    ATTITUDE.CompassHeading += 3600;
  }
  ATTITUDE.AngleOut[YAW] = ConvertDeciDegreesToDegrees(ATTITUDE.CompassHeading);
}

float AHRSClass::CosineTiltAngle(void)
{
  return 1.0f - 2.0f * SquareFloat(Orientation.q1) - 2.0f * SquareFloat(Orientation.q2);
}

bool AHRSClass::CheckAnglesInclination(int16_t Angle)
{
  if (AHRS.CosineTiltAngle() < Fast_Cosine(ConvertToRadians(Angle)))
  {
    return true;
  }
  return false;
}

float AHRSClass::SineRoll()
{
  return Fast_Sine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL]));
}

float AHRSClass::CosineRoll()
{
  return Fast_Cosine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[ROLL]));
}

float AHRSClass::SinePitch()
{
  return Fast_Sine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[PITCH]));
}

float AHRSClass::CosinePitch()
{
  return Fast_Cosine(ConvertDeciDegreesToRadians(ATTITUDE.AngleOut[PITCH]));
}

float AHRSClass::SineYaw()
{
  return Fast_Sine(ConvertDeciDegreesToRadians(ATTITUDE.CompassHeading));
}

float AHRSClass::CosineYaw()
{
  return Fast_Cosine(ConvertDeciDegreesToRadians(ATTITUDE.CompassHeading));
}