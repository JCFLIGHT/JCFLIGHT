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

#include <inttypes.h>
#include "ENUM.h"
#include "RCDEFINES.h"

typedef struct
{
  int16_t AccelerometerRead[3];
  int16_t AccelerometerReadNotFiltered[3];
  int16_t GyroscopeRead[3];
  int16_t GyroscopeReadNotFiltered[3];
  int16_t CompassRead[3];
} IMU_STRUCT;

typedef struct
{
  uint8_t AccelerationEarthFrame_Sum_Count[3];
  float AccelerationEarthFrame[3];
  float AccelerationEarthFrame_Filtered[3];
  float AccelerationEarthFrame_Sum[3];
  float Velocity_EarthFrame[3];
  float Position_EarthFrame[3];
  int32_t PositionToHold[2];
} INS_STRUCT;

typedef struct
{
  int32_t RealBaroAltitude;
  int32_t EstimatedAltitude;
  int16_t EstimatedVariometer;
  int32_t GroundAltitude;
} ALTITUDE_STRUCT;

typedef struct
{
  int16_t AngleOut[3];
  int16_t CompassHeading;
} ATTITUDE_STRUCT;

typedef struct
{
  int16_t AccelerometerZero[3];
  uint16_t AccelerometerScale[3];
  int16_t Magnetometer[3];
} CALIBRATION_STRUCT;

struct PID_TERMS
{
  uint8_t ProportionalVector;
  uint8_t IntegratorVector;
  uint8_t DerivativeVector;
};

typedef struct
{
  uint8_t CalcedValue = 0;
  int16_t BreakPointer = 1500;
  uint16_t ThrottlePercent = 0;
  float Factor = 0;
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
} Struct_BeeperEntry;

typedef struct
{
  float kP_Accelerometer = 0.25f;
  float kI_Accelerometer = 0.0050f;
  float kP_Magnetometer = 1;
  float kI_Magnetometer = 0;
} Struct_IMURuntimeConfiguration;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} Struct_Quaternion;

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
  float IntegratorMax;
} PID_PARAM;

typedef struct _GPS_PID
{
  float Integrator;
  int32_t Last_Input;
  float Last_Derivative;
  float Derivative;
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
  const char *TaskName;
  void (*TaskFunction)();
  int32_t DesiredPeriod;
  const uint8_t StaticPriority;
  uint16_t DynamicPriority;
  uint16_t TaskAgeCycles;
  uint32_t LastExecuted;
  int32_t TaskLatestDeltaTime;
} Task_Resources_Struct;
#endif
