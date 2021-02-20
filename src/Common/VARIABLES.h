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

#ifndef JCFLIGHT_H_
#define JCFLIGHT_H_
#include "Arduino.h"
#include "STRUCTS.h"

//*******************************************************
//BOOL OU BOOLEAN (0 - 1) OU (TRUE - FALSE)
//*******************************************************
extern bool CalibratingCompass;
extern bool GPSHold_CallBaro;
extern bool Fail_Safe_Event;
extern bool Home_Point;
extern bool GPS_3DFIX;
extern bool Do_Stabilize_Mode;
extern bool Do_HeadingHold_Mode;
extern bool Do_AltitudeHold_Mode;
extern bool Do_GPS_Altitude;
extern bool Do_AutoThrottle_Mode;
extern bool ImmediatelyFailSafe;

//*******************************************************
//UNSIGNED 8 BITS (0 - 255)
//*******************************************************
extern uint8_t RTH_Altitude;
extern uint8_t Compass_Type;
extern uint8_t MagAddress;
extern uint8_t MagRegister;
extern uint8_t RCRate;
extern uint8_t RCExpo;
extern uint8_t DynamicRollAndPitchRate[2];
extern uint8_t YawRate;
extern uint8_t ThrottleMiddle;
extern uint8_t ThrottleExpo;
extern uint8_t NavigationMode;
extern uint8_t GPS_NumberOfSatellites;
extern uint8_t FrameType;
extern uint8_t GPS_Flight_Mode;
extern uint8_t SetFlightMode[SIZE_OF_FLIGHT_MODES];

//*******************************************************
//UNSIGNED 16 BITS (0 - 65.535‬)
//*******************************************************
extern uint16_t GPS_HDOP;
extern uint16_t GPS_Ground_Course;
extern uint16_t GPS_Altitude;
extern uint16_t GPS_Ground_Speed;
extern uint16_t DistanceToHome;
extern uint16_t CalculeLookUpThrottle[11];
extern uint16_t CalibratingAccelerometer;
extern uint16_t CalibratingGyroscope;
extern uint16_t LedRGB[3];

//*******************************************************
//SIGNED 16 BITS (-32.767 - 32.767)
//*******************************************************
extern int16_t DirectRadioControllRead[12];
extern int16_t PIDControllerApply[3];
extern int16_t MotorControl[8];
extern int16_t GPS_Navigation_Array[2];
extern int16_t DirectionToHome;
extern int16_t RadioControllOutput[12];
extern int16_t RCController[4];
extern int16_t GPS_Angle[3];
extern int16_t HeadingHoldTarget;
extern int16_t IOC_Initial_Compass;
extern int16_t I2CErrors;
extern int16_t AttitudeThrottleMin;
extern int16_t AttitudeThrottleMax;
extern int16_t GPSVelNED[3];
extern volatile int16_t Fail_Safe_System;

//*******************************************************
//FLOAT 24 BITS COM EXPONENTE DE 8 BITS
//*******************************************************
extern float CoordinatedTurnRateEarthFrame;

//*******************************************************
//UNSIGNED 32 BITS (0 - 4.294.967.295)
//*******************************************************
extern uint32_t Time_To_Start_The_Land;

//*******************************************************
//SIGNED 32 BITS (-2.147.483.647 - 2.147.483.647)
//*******************************************************
extern int32_t GPS_Coordinates_Vector[2];
extern int32_t Stored_Coordinates_Home_Point[2];
extern int32_t GPS_CoordinatesToHold[2];

extern CALIBRATION_STRUCT CALIBRATION;
extern IMU_STRUCT IMU;
extern INS_STRUCT INS;
extern ALTITUDE_STRUCT ALTITUDE;
extern ATTITUDE_STRUCT ATTITUDE;
extern PID_TERMS PID[SIZE_OF_PID_PARAMS];
#endif
