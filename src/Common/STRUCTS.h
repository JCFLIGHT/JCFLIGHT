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

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "ENUM.h"
#include "RCDEFINES.h"

typedef struct
{
  struct Accelerometer_Struct
  {
    int16_t Read[3] = {0, 0, 0};
    int16_t ReadNotFiltered[3] = {0, 0, 0};
    struct Gravity_Struct
    {
      bool Initialization = false;
      float Value = 0;
    } GravityForce;
  } Accelerometer;

  struct Gyroscope_Struct
  {
    int16_t Read[3] = {0, 0, 0};
    int16_t ReadNotFiltered[3] = {0, 0, 0};
  } Gyroscope;

  struct Compass_Struct
  {
    bool Calibrating = false;
    int16_t Read[3] = {0, 0, 0};
    float ReadSmooth[3] = {0, 0, 0};
  } Compass;
} IMU_Struct;

typedef struct
{
  uint8_t AccelerationEarthFrame_Sum_Count[3];
  float AccelerationEarthFrame[3];
  float AccelerationEarthFrame_Filtered[3];
  float AccelerationEarthFrame_Sum[3];
  float Velocity_EarthFrame[3];
  float Position_EarthFrame[3];
  int32_t PositionToHold[2];
} INS_Struct;

typedef struct
{
  struct Calibration_Struct
  {
    float GroundPressure = 0;
    float GroundTemperature = 0;
  } Calibration;

  struct Raw_Struct
  {
    int16_t Temperature = 0;
    int32_t Pressure = 0;
    int32_t PressureFiltered = 0;
  } Raw;

  struct Altitude_Struct
  {
    int32_t Actual = 0;
    int32_t GroundOffSet = 0;
  } Altitude;

  struct INS_Struct
  {
    struct Velocity_Struct
    {
      int16_t Vertical = 0;
    } Velocity;

    struct Altitude_Struct
    {
      int32_t Estimated = 0;
    } Altitude;
  } INS;

} Barometer_Struct;

typedef struct
{
  int16_t AngleOut[3];
  int16_t CompassHeading;
} Attitude_Struct;

typedef struct
{
  struct Accelerometer_Struct
  {
    int16_t Counter = 0;
    int16_t PositionCount = 0;
    int16_t OffSet[3] = {0, 0, 0};
    uint16_t Scale[3] = {0, 0, 0};
    int32_t Samples[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  } Accelerometer;

  struct Magnetometer_Struct
  {
    float Gain[3] = {1.0f, 1.0f, 1.0f};
    int16_t OffSet[3] = {0, 0, 0};
    int16_t MinOffSet[3] = {0, 0, 0};
    int16_t MaxOffSet[3] = {0, 0, 0};
    int16_t Count = 0;
    int16_t SimpleMode_Initial_Value = 0;
  } Magnetometer;

} Calibration_Struct;

struct PID_Terms_Struct
{
  bool State;
  uint8_t ProportionalVector;
  uint8_t IntegralVector;
  uint8_t DerivativeVector;
  uint8_t FeedForwardVector;
  uint8_t MinMaxValueVector;
};

typedef struct
{
  bool UpdateRequired = false;
  float Factor = 0;
  float CalcedValue = 0;
  int16_t PreviousThrottle;
  int16_t BreakPointer = 1500;
  uint16_t ThrottlePercent = 0;
  uint16_t FixedWingTauMS;
} TPA_Parameters_Struct;

typedef struct
{
  float OldMeasure;
  float NewMeasure;
  float OldValue;
  float NewValue;
  int16_t MeasureCount;
} Device_Struct;

typedef struct
{
  float Roll;
  float Pitch;
  float Yaw;
} PID_Mixer_Struct;

typedef struct
{
  uint8_t FrameMotorsCount;
} Motors_Count_Struct;

typedef struct
{
  uint8_t Mode;
  uint8_t Priority;
  const uint8_t *Sequence;
} BeeperEntry_Struct;

typedef struct
{
  float kP_Accelerometer = 0.25f;
  float kI_Accelerometer = 0.0050f;
  float kP_Magnetometer = 1;
  float kI_Magnetometer = 0;
} AHRS_Configuration_Struct;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion_Struct;

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

//VETOR EM INT,PARA EVITAR AVISO DE COMPILAÇÃO DO GCC
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

typedef struct _PID_PARAM
{
  float kP;
  float kI;
  float kD;
  float IntegralMax;
} PID_PARAM;

typedef struct _GPS_PID
{
  float Integral;
  float Last_Derivative;
  float Derivative;
  int32_t Last_Input;
} GPS_PID;

typedef struct
{
  uint32_t ActualTime;
  uint32_t StoredTime;
} Scheduler_Struct;

typedef struct PT1Filter
{
  float State;
  float RC;
  float DeltaTime;
} PT1_Filter_Struct;

typedef struct
{
  uint32_t ActualTime;
  uint32_t PreviousTime;
  uint32_t TotalTime;
} MachineInitTime_Struct;

typedef struct
{
  const char *TaskName;
  void (*TaskFunction)();
  int32_t DesiredPeriod;
  const uint8_t StaticPriority;
  uint16_t DynamicPriority;
  uint16_t TaskAgeCycles;
  uint32_t LastExecuted;
  int32_t TaskLatestDeltaTime;
} Task_Resources_Struct;

typedef struct
{
  float a, b, g, e;
  float aK, vK, xK, jK;
  float DeltaTime, DeltaTime2, DeltaTime3;
  float HalfLife, Boost;
} AlphaBetaGammaFilter_Struct;

typedef struct
{
  //MATRIX JACOBIANA
  //http://en.wikipedia.org/wiki/Jacobian_matrix
  float Matrix_JtR[4];
  float Matrix_JtJ[4][4];
} Jacobian_Struct;

typedef struct
{
  bool Healthy = false;

  struct Raw_Stuct
  {
    float Pressure = 0;
    float IASPressure = 0;
    uint16_t IASPressureInCM = 0;
  } Raw;

  struct Calibration_Stuct
  {
    bool Initialized = false;
    float OffSet = 0;
    float Sum = 0;
    uint16_t Count = 0;
    uint16_t Read_Count = 0;
    uint32_t Start_MS = 0;
  } Calibration;
} AirSpeed_Struct;

typedef struct
{
  bool ValidWindEstimated = false;
  struct Ground_Struct
  {
    float Velocity[3] = {0, 0, 0};
    float VelocityDifference[3] = {0, 0, 0};
    float VelocitySum[3] = {0, 0, 0};
    float LastVelocity[3] = {0, 0, 0};
  } Ground;

  struct Fuselage_Struct
  {
    float Direction[3] = {0, 0, 0};
    float DirectionDifference[3] = {0, 0, 0};
    float DirectionSum[3] = {0, 0, 0};
    float LastDirection[3] = {0, 0, 0};
  } Fuselage;

  struct EarthFrame_Struct
  {
    float EstimatedWindVelocity[3] = {0, 0, 0};
  } EarthFrame;

  struct Time_Struct
  {
    uint32_t Now = 0;
    uint32_t LastUpdate = 0;
  } Time;
} WindEstimator_Struct;

typedef struct
{
  struct Pulse_Struct
  {
    int16_t Min[4] = {0, 0, 0, 0};
    int16_t Middle[4] = {0, 0, 0, 0};
    int16_t Max[4] = {0, 0, 0, 0};
  } Pulse;

  struct Direction_Struct
  {
    int8_t GetAndSet[4] = {1, 1, 1, 1};
  } Direction;

  struct Rate_Struct
  {
    int8_t GetAndSet[4] = {0, 0, 0, 0};
  } Rate;

  struct Filter_Struct
  {
    int16_t CutOff = 0;
  } Filter;

  struct Signal_Struct
  {
    int16_t UnFiltered[4] = {0, 0, 0, 0};
    int16_t Filtered[4] = {0, 0, 0, 0};
  } Signal;

} Servo_Struct;

#endif
