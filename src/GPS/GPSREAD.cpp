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

#include "GPSREAD.h"
#include "FastSerial/FASTSERIAL.h"
#include "Common/VARIABLES.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "ProgMem/PROGMEM.h"

//COM OS GPS-M8N É POSSIVEL ATIGIR MAIS DE 30 SATELITES
#define UBLOX_BUFFER_SIZE 464

struct Ublox_Navigation_PosLLH
{
  uint32_t time;
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};

struct Ublox_Navigation_Solution
{
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};

struct Ublox_Navigation_VelNED
{
  uint32_t time;
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
};

static union
{
  Ublox_Navigation_PosLLH PositionLLH;
  Ublox_Navigation_Solution Solution;
  Ublox_Navigation_VelNED VelocityNED;
  uint8_t Bytes_Array[464];
} Buffer;

//CHECAGEM DE STATUS DO 3D FIX
static bool Next_GPSFix;

//VERIFICAÇÃO DOS PACOTES DE DADOS
static uint8_t Check_Packet_A;
static uint8_t Check_Packet_B;

//ESTADO DE MAQUINA
static uint8_t Step_Counter;
static uint8_t Get_GPS_Message_ID;
static uint16_t Payload_Length;
static uint16_t Payload_Counter;

#ifdef __AVR_ATmega2560__
const uint8_t Ublox_Set_Configuration[] __attribute__((__progmem__)) = {
#elif defined __arm__
const uint8_t Ublox_Set_Configuration[] = {
#endif
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2,
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28,
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0xC9,
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

static void SerialSendConfigToGPS(const char *STR)
{
#ifdef __AVR_ATmega2560__
  char ProgramMemory;
  while (STR && (ProgramMemory = ProgMemReadByte(STR++)))
  {
    FASTSERIAL.Write(UART1, ProgramMemory);
    AVRTIME.SchedulerSleep(5);
  }
#elif defined __arm__

#endif
}

void GPS_SerialInit(uint32_t Get_BaudRate)
{
  static uint8_t Parse_Baud_Rate = 0;
  FASTSERIAL.Begin(UART1, Get_BaudRate);
  AVRTIME.SchedulerSleep(1000);
  if (Parse_Baud_Rate == 0)
  {
    FASTSERIAL.Begin(UART1, 9600);
    if (Get_BaudRate == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (Get_BaudRate == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (Get_BaudRate == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (Get_BaudRate == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 1;
  }
  else if (Parse_Baud_Rate == 1)
  {
    FASTSERIAL.Begin(UART1, 19200);
    if (Get_BaudRate == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (Get_BaudRate == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (Get_BaudRate == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (Get_BaudRate == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 2;
  }
  else if (Parse_Baud_Rate == 2)
  {
    FASTSERIAL.Begin(UART1, 38400);
    if (Get_BaudRate == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (Get_BaudRate == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (Get_BaudRate == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (Get_BaudRate == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 3;
  }
  else if (Parse_Baud_Rate == 3)
  {
    FASTSERIAL.Begin(UART1, 57600);
    if (Get_BaudRate == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (Get_BaudRate == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (Get_BaudRate == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (Get_BaudRate == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 4;
  }
  else if (Parse_Baud_Rate == 4)
  {
    FASTSERIAL.Begin(UART1, 115200);
    if (Get_BaudRate == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (Get_BaudRate == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (Get_BaudRate == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (Get_BaudRate == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
  }
  AVRTIME.SchedulerSleep(200);
  FASTSERIAL.Begin(UART1, Get_BaudRate);
  for (uint8_t SizeOfCount = 0; SizeOfCount < sizeof(Ublox_Set_Configuration); SizeOfCount++)
  {
#ifdef __AVR_ATmega2560__
    FASTSERIAL.Write(UART1, ProgMemReadByte(Ublox_Set_Configuration + SizeOfCount));
#elif defined __arm__

#endif
    AVRTIME.SchedulerSleep(5);
  }
}

void GPS_SerialRead(uint8_t ReadData)
{
  switch (Step_Counter)
  {

  case 1:
    if (PREAMBLE2 == ReadData)
    {
      Step_Counter++;
      break;
    }
    Step_Counter = 0;

  case 0:
    if (PREAMBLE1 == ReadData)
      Step_Counter++;
    break;

  case 2:
    Step_Counter++;
    Check_Packet_B = Check_Packet_A = ReadData;
    break;

  case 3:
    Step_Counter++;
    Check_Packet_B += (Check_Packet_A += ReadData);
    Get_GPS_Message_ID = ReadData;
    break;

  case 4:
    Step_Counter++;
    Check_Packet_B += (Check_Packet_A += ReadData);
    Payload_Length = ReadData;
    break;

  case 5:
    Step_Counter++;
    Check_Packet_B += (Check_Packet_A += ReadData);
    Payload_Length += (uint16_t)(ReadData << 8);
    if (Payload_Length > UBLOX_BUFFER_SIZE)
    {
      Payload_Length = 0;
      Step_Counter = 0;
    }
    Payload_Counter = 0;
    break;

  case 6:
    Check_Packet_B += (Check_Packet_A += ReadData);
    if (Payload_Counter < UBLOX_BUFFER_SIZE)
    {
      Buffer.Bytes_Array[Payload_Counter] = ReadData;
    }
    if (++Payload_Counter == Payload_Length)
      Step_Counter++;
    break;

  case 7:
    Step_Counter++;
    if (Check_Packet_A != ReadData)
      Step_Counter = 0;
    break;

  case 8:
    Step_Counter = 0;
    if (Check_Packet_B != ReadData)
      break;
    GetAllGPSData();
  }
}

void GetAllGPSData(void)
{
  switch (Get_GPS_Message_ID)
  {

  case MSG_POSLLH:
    GPS_Coordinates_Vector[1] = Buffer.PositionLLH.longitude;
    GPS_Coordinates_Vector[0] = Buffer.PositionLLH.latitude;
    GPS_Altitude = Buffer.PositionLLH.altitude_msl / 10 / 100;
    GPS_3DFIX = Next_GPSFix;
    break;

  case MSG_STATUS:
    Next_GPSFix = ((Buffer.Solution.fix_status & NAV_STATUS_FIX_VALID) && (Buffer.Solution.fix_type == FIX_3D));
    if (!Next_GPSFix)
      GPS_3DFIX = false;
    break;

  case MSG_SOL:
    Next_GPSFix = (Buffer.Solution.fix_status & NAV_STATUS_FIX_VALID) && (Buffer.Solution.fix_type == FIX_3D);
    if (!Next_GPSFix)
      GPS_3DFIX = false;
    GPS_NumberOfSatellites = Buffer.Solution.satellites;
    GPS_HDOP = Buffer.Solution.position_DOP;
    break;

  case MSG_VELNED:
    GPS_Ground_Speed = Buffer.VelocityNED.speed_2d;
    GPS_Ground_Course = (uint16_t)(Buffer.VelocityNED.heading_2d / 10000);
    //APENAS PARA AERO MODE
    if (GPS_Ground_Speed > 100)
    {
      GPS_Ground_Course = WRap_180(GPS_Ground_Course * 10) / 10;
    }
    break;
  }
}