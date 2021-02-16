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

#include "VARIABLES.h"
#include "STRUCTS.h"

//*******************************************************
//BOOL OU BOOLEAN (0 - 1) OU (TRUE - FALSE)
//*******************************************************
bool COMMAND_ARM_DISARM;
bool SetFlightModes[SIZE_OF_FLIGHT_MODES];
bool CalibratingCompass;
bool Do_GPS_Altitude;
bool GPSHold_CallBaro;
bool Fail_Safe_Event;
bool Home_Point;
bool GPS_3DFIX;
bool Do_Stabilize_Mode;
bool Do_HeadingHold_Mode;
bool Do_AltitudeHold_Mode;
bool ImmediatelyFailSafe;
bool TurnCoordinatorMode;

//*******************************************************
//UNSIGNED 8 BITS (0 - 255)
//*******************************************************
uint8_t FrameType;
uint8_t RTH_Altitude;
uint8_t Compass_Type;
uint8_t MagAddress;
uint8_t MagRegister;
uint8_t GPS_NumberOfSatellites;
uint8_t NavigationMode;
uint8_t GPS_Flight_Mode;
uint8_t RCRate;
uint8_t RCExpo;
uint8_t DynamicRollAndPitchRate[2];
uint8_t YawRate;
uint8_t ThrottleMiddle;
uint8_t ThrottleExpo;

//*******************************************************
//UNSIGNED 16 BITS (0 - 65.535‬)
//*******************************************************
uint16_t GPS_HDOP;
uint16_t CalibratingAccelerometer;
uint16_t CalibratingGyroscope = 512;
uint16_t CalculeLookUpThrottle[11];
uint16_t GPS_Ground_Course;
uint16_t DistanceToHome;
uint16_t GPS_Altitude;
uint16_t GPS_Ground_Speed;
uint16_t LedRGB[3];

//*******************************************************
//SIGNED 16 BITS (-32.767 - 32.767)
//*******************************************************
int16_t RadioControllOutput[12];
int16_t DirectRadioControllRead[12];
int16_t RCController[4];
int16_t PIDControllerApply[3];
int16_t MotorControl[8];
int16_t HeadingHoldTarget;
int16_t IOC_Initial_Compass;
int16_t GPS_Navigation_Array[2];
int16_t GPS_Angle[3];
int16_t DirectionToHome;
int16_t I2CErrors;
int16_t AttitudeThrottleMin;
int16_t AttitudeThrottleMax;
int16_t GPSVelNED[3];
volatile int16_t Fail_Safe_System;

//*******************************************************
//FLOAT 24 BITS COM EXPONENTE DE 8 BITS
//*******************************************************
float CoordinatedTurnRateEarthFrame;

//*******************************************************
//UNSIGNED 32 BITS (0 - 4.294.967.295)
//*******************************************************
uint32_t Time_To_Start_The_Land;

//*******************************************************
//SIGNED 32 BITS (-2.147.483.647 - 2.147.483.647)
//*******************************************************
int32_t GPS_Coordinates_Vector[2];
int32_t Stored_Coordinates_Home_Point[2];
int32_t GPS_CoordinatesToHold[2];

IMU_STRUCT IMU;
INS_STRUCT INS;
ALTITUDE_STRUCT ALTITUDE;
ATTITUDE_STRUCT ATTITUDE;
CALIBRATION_STRUCT CALIBRATION;
PID_TERMS PID[SIZE_OF_PID_PARAMS];
