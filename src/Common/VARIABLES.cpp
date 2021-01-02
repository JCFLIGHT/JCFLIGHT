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
bool COMMAND_ARM_DISARM = false;
bool SetFlightModes[SIZEOFTHIS];
bool CalibratingCompass = false;
bool Do_GPS_Altitude = false;
bool GPSHold_CallBaro = false;
bool Fail_Safe_Event = false;
bool Home_Point = false;
bool GPS_3DFIX = false;
bool Do_Stabilize_Mode = false;
bool Do_HeadingHold_Mode = false;
bool Do_IOC_Mode = false;
bool Do_AltitudeHold_Mode = false;
bool Cancel_Arm_Disarm = false;
bool ImmediatelyFailSafe = false;
bool TurnCoordinatorMode = false;

//*******************************************************
//UNSIGNED 8 BITS (0 - 255)
//*******************************************************
uint8_t FrameType = 0;
uint8_t RTH_Altitude = 0;
uint16_t LedRGB[3] = {0, 0, 0};
uint8_t Compass_Type = 0;
uint8_t MagAddress = 0;
uint8_t MagRegister = 0;
uint8_t GPS_NumberOfSatellites;
uint8_t NavigationMode = 0;
uint8_t GPS_Flight_Mode;
uint8_t RCRate;
uint8_t RCExpo;
uint8_t RollAndPitchRate;
uint8_t YawRate;
uint8_t ThrottleMiddle;
uint8_t ThrottleExpo;

//*******************************************************
//UNSIGNED 16 BITS (0 - 65.535‬)
//*******************************************************
uint16_t MotorSpeed = 0;
uint16_t GPS_HDOP;
uint16_t CalibratingAccelerometer = 0;
uint16_t CalibratingGyroscope;
uint16_t CalculeLookUpThrottle[11];
uint16_t GPS_Ground_Course = 0;
uint16_t DistanceToHome;
uint16_t GPS_Altitude;
uint16_t GPS_Ground_Speed;

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
int16_t GPS_Angle[3] = {0, 0, 0};
int16_t DirectionToHome;
int16_t ErrorI2C = 0;
int16_t SlipAngleForAirPlane;
volatile int16_t Fail_Safe_System = 0;

//*******************************************************
//FLOAT 24 BITS COM EXPONENTE DE 8 BITS
//*******************************************************

//*******************************************************
//UNSIGNED 32 BITS (0 - 4.294.967.295)
//*******************************************************
uint32_t Time_To_Start_The_Land = 0;

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
PID_TERMS PID[SIZE_OF_THIS_PID_PARAMS];
