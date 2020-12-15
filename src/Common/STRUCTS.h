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

#include "Arduino.h"

//AUX1
#define AUX1_LOW RadioControllOutput[AUX1] < 1100
#define AUX1_MID RadioControllOutput[AUX1] > 1400 && RadioControllOutput[AUX1] < 1600
#define AUX1_HIGH RadioControllOutput[AUX1] > 1900
//AUX2
#define AUX2_LOW RadioControllOutput[AUX2] < 1100
#define AUX2_MID RadioControllOutput[AUX2] > 1400 && RadioControllOutput[AUX2] < 1600
#define AUX2_HIGH RadioControllOutput[AUX2] > 1900
//AUX3
#define AUX3_LOW RadioControllOutput[AUX3] < 1100
#define AUX3_MID RadioControllOutput[AUX3] > 1400 && RadioControllOutput[AUX3] < 1600
#define AUX3_HIGH RadioControllOutput[AUX3] > 1900
//AUX4
#define AUX4_LOW RadioControllOutput[AUX4] < 1100
#define AUX4_MID RadioControllOutput[AUX4] > 1400 && RadioControllOutput[AUX4] < 1600
#define AUX4_HIGH RadioControllOutput[AUX4] > 1900
//AUX5
#define AUX5_LOW RadioControllOutput[AUX5] < 1100
#define AUX5_MID RadioControllOutput[AUX5] > 1400 && RadioControllOutput[AUX5] < 1600
#define AUX5_HIGH RadioControllOutput[AUX5] > 1900
//AUX6
#define AUX6_LOW RadioControllOutput[AUX6] < 1100
#define AUX6_MID RadioControllOutput[AUX6] > 1400 && RadioControllOutput[AUX6] < 1600
#define AUX6_HIGH RadioControllOutput[AUX6] > 1900
//AUX7
#define AUX7_LOW RadioControllOutput[AUX7] < 1100
#define AUX7_MID RadioControllOutput[AUX7] > 1400 && RadioControllOutput[AUX7] < 1600
#define AUX7_HIGH RadioControllOutput[AUX7] > 1900
//AUX8
#define AUX8_LOW RadioControllOutput[AUX8] < 1100
#define AUX8_MID RadioControllOutput[AUX8] > 1400 && RadioControllOutput[AUX8] < 1600
#define AUX8_HIGH RadioControllOutput[AUX8] > 1900

enum
{
  RESET = 0,
  ROLL = 0,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8
};

enum
{
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALTITUDE,
  PIDGPSPOSITION,
  PIDGPSPOSITIONRATE,
  PIDGPSNAVIGATIONRATE,
  PIDAUTOLEVEL,
  PIDYAWVELOCITY,
  SIZE_OF_THIS_PID_PARAMS
};

enum
{
  ARMDISARM_MODE,
  STABILIZE_MODE,
  ALTITUDEHOLD_MODE,
  SETCOMPASSACTUAL,
  IOC_MODE,
  RTH_MODE,
  GPSHOLD_MODE,
  ATACK_MODE,
  LAND_MODE,
  SIZEOFTHIS
};

enum
{
  AUXONELOW = 1,
  AUXONEMIDDLE,
  AUXONEHIGH,
  AUXTWOLOW,
  AUXTWOMIDDLE,
  AUXTWOHIGH,
  AUXTHREELOW,
  AUXTHREEMIDDLE,
  AUXTHREEHIGH,
  AUXFOURLOW,
  AUXFOURMIDDLE,
  AUXFOURHIGH,
  AUXFIVELOW,
  AUXFIVEMIDDLE,
  AUXFIVEHIGH,
  AUXSIXLOW,
  AUXSIXMIDDLE,
  AUXSIXHIGH,
  AUXSEVENLOW,
  AUXSEVENMIDDLE,
  AUXSEVENHIGH,
  AUXEIGHTLOW,
  AUXEIGHTMIDDLE,
  AUXEIGHTHIGH
};

enum
{
  MOTOR1 = 0,
  MOTOR2,
  MOTOR3,
  MOTOR4,
  MOTOR5,
  MOTOR6,
  GIMBAL,
  PARACHUTESERVO
};

enum
{
  SERVO1 = 0,
  SERVO2,
  SERVO3,
  SERVO4
};

enum
{
  GPS_MODE_NONE = 0,
  GPS_MODE_HOLD,
  GPS_MODE_RTH,
  WAYPOINT
};

enum
{
  Do_None = 0,
  Do_Start_RTH,
  Do_RTH_Enroute,
  Do_PositionHold,
  Do_Land_Init,
  Do_LandInProgress,
  Do_Landed,
  Do_Land_Settle,
  Do_Land_Descent,
  Do_Land_Detected
};

enum
{
  NONE = 0,
  RCAUX1,
  RCAUX2,
  RCAUX3,
  RCAUX4,
  RCAUX5,
  RCAUX6,
  RCAUX7,
  RCAUX8
};

enum
{
  RED = 0,
  GREEN,
  BLUE
};

enum
{
  GPSLED = 0,
  ACCLED,
  MAGLED,
  CONFIGFLIGHT,
  CALIBRATIONESC,
  CALIBRATIONESCFINISH,
  SAVINGVALUESSAVETRIM,
  SUCESSSAVETRIM,
  FAILSAVETRIM,
  PREARMINIT,
  PREARMSUCESS,
  PREARMFAIL,
  OFFLEDS
};

enum
{
  //16 PINOS ANALOGICOS MAXIMO,DE ACORDO COM O MEGA 2560
  ADC0 = 0, //OFF(V.BATTERY INPUT)
  ADC1,     //OFF(CURRENT INPUT)
  ADC2,     //OFF(PITOT TUBE)
  ADC3,
  ADC4,
  ADC5,
  ADC6,
  ADC7,
  ADC8,
  ADC9,
  ADC10,
  ADC11,
  ADC12,
  ADC13,
  ADC14, //OFF(SAFETY BUTTON)
  ADC15  //OFF (PPM INPUT)
};

enum
{
  COMPASS_AK8975 = 0,
  COMPASS_HMC5843,
  COMPASS_HMC5883
};

enum
{
  NONE_ROTATION = 0,
  COMPASS_ROTATION_YAW_45_DEGREES,
  COMPASS_ROTATION_YAW_315_DEGREES,
  COMPASS_ROTATION_ROLL_180_YAW_45_DEGREES,
  COMPASS_ROTATION_PITCH_180_DEGREES
};

enum
{
  UART0 = 0,
  UART1,
  UART2,
  UART3
};

enum
{
  GPS_ONBOARD_COMPASS = 0,
  EXTERNAL_COMPASS
};

enum
{
  BAROMETER_MS5611 = 0,
  BAROMETER_BMP280
};

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
#endif
