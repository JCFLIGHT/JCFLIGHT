#ifndef DJINAZAGPS_H_
#define DJINAZAGPS_H_
#include "inttypes.h"
//VARIAVEIS DE SAÍDA
extern uint8_t DJINaza_Num_Sat;
extern uint8_t DJINaza_Fix_State;
extern uint16_t DJINaza_HDOP;
extern int16_t DJINaza_Compass_Roll;
extern int16_t DJINaza_Compass_Pitch;
extern int16_t DJINaza_Compass_Yaw;
extern int32_t DJINaza_Latitude;
extern int32_t DJINaza_Longitude;
extern int32_t DJINaza_Altitude;
extern int32_t DJINaza_GroundCourse;
extern int32_t DJINaza_GroundSpeed;
void DjiNazaGpsNewFrame(uint8_t SerialReceiverBuffer);
#endif
