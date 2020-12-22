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
  int16_t AccelerometerReadNF[3];
  int16_t GyroscopeRead[3];
  int16_t GyroscopeReadNF[3];
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
} INS_STRUCT;

typedef struct
{
  int32_t RealBaroAltitude;
  int32_t EstimateAltitude;
  int16_t EstimateVariometer;
  int32_t GroundAltitude;
} ALTITUDE_STRUCT;

typedef struct
{
  int16_t AngleOut[2];
  int16_t CalculedHeading;
  int16_t CompassHeading;
} ATTITUDE_STRUCT;

typedef struct
{
  int16_t AccelerometerCalibration[3];
  uint16_t AccelerometerCalibrationScale[3];
  int16_t MagnetometerCalibration[3];
} CALIBRATION_STRUCT;

struct PID_TERMS
{
  uint8_t ProportionalVector;
  uint8_t IntegratorVector;
  uint8_t DerivativeVector;
};

typedef struct
{
  int8_t ThrottleIdleFactor = 15;
  int16_t MaxThrottle = 1900;
  int16_t TPABreakPointer = 1500;
  int16_t ThrottleIdleValue = 0;
  uint16_t TPAThrottlePercent = 0;
} TPA_Parameters_Struct;
#endif
