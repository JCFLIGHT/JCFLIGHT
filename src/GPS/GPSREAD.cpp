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

//COM OS GPS-M8N É POSSIVEL ATIGIR MAIS DE 30 SATELITES
#define UBLOX_BUFFER_SIZE 464

static bool Next_GPSFix;

//VERIFICAÇÃO DOS PACOTES DE DADOS
static uint8_t _ck_a;
static uint8_t _ck_b;
//ESTADO DE MAQUINA
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

struct ubx_header
{
  uint8_t preamble1;
  uint8_t preamble2;
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t length;
};
struct ubx_nav_posllh
{
  uint32_t time;
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};
struct ubx_nav_solution
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
struct ubx_nav_velned
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

enum ubs_protocol_bytes
{
  PREAMBLE1 = 0xb5,
  PREAMBLE2 = 0x62,
  CLASS_NAV = 0x01,
  CLASS_ACK = 0x05,
  CLASS_CFG = 0x06,
  MSG_ACK_NACK = 0x00,
  MSG_ACK_ACK = 0x01,
  MSG_POSLLH = 0x2,
  MSG_STATUS = 0x3,
  MSG_SOL = 0x6,
  MSG_VELNED = 0x12,
  MSG_CFG_PRT = 0x00,
  MSG_CFG_RATE = 0x08,
  MSG_CFG_SET_RATE = 0x01,
  MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type
{
  FIX_NONE = 0,
  FIX_DEAD_RECKONING = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_GPS_DEAD_RECKONING = 4,
  FIX_TIME = 5
};
enum ubx_nav_status_bits
{
  NAV_STATUS_FIX_VALID = 1
};

static union
{
  ubx_nav_posllh posllh;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  uint8_t bytes[464];
} _buffer;

const uint8_t UBLOX_INIT[] __attribute__((__progmem__)) = {
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
  char ProgramMemory;
  while (STR && (ProgramMemory = pgm_read_byte(STR++)))
  {
    FASTSERIAL.Write(UART1, ProgramMemory);
    AVRTIME.SchedulerSleep(5);
  }
}

void GPS_SerialInit(uint32_t GPS_BAUDRATE)
{
  static uint8_t Parse_Baud_Rate = 0;
  FASTSERIAL.Begin(UART1, GPS_BAUDRATE);
  AVRTIME.SchedulerSleep(1000);
  if (Parse_Baud_Rate == 0)
  {
    FASTSERIAL.Begin(UART1, 9600);
    if (GPS_BAUDRATE == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (GPS_BAUDRATE == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (GPS_BAUDRATE == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (GPS_BAUDRATE == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 1;
  }
  else if (Parse_Baud_Rate == 1)
  {
    FASTSERIAL.Begin(UART1, 19200);
    if (GPS_BAUDRATE == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (GPS_BAUDRATE == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (GPS_BAUDRATE == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (GPS_BAUDRATE == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 2;
  }
  else if (Parse_Baud_Rate == 2)
  {
    FASTSERIAL.Begin(UART1, 38400);
    if (GPS_BAUDRATE == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (GPS_BAUDRATE == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (GPS_BAUDRATE == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (GPS_BAUDRATE == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 3;
  }
  else if (Parse_Baud_Rate == 3)
  {
    FASTSERIAL.Begin(UART1, 57600);
    if (GPS_BAUDRATE == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (GPS_BAUDRATE == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (GPS_BAUDRATE == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (GPS_BAUDRATE == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
    Parse_Baud_Rate = 4;
  }
  else if (Parse_Baud_Rate == 4)
  {
    FASTSERIAL.Begin(UART1, 115200);
    if (GPS_BAUDRATE == 19200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    else if (GPS_BAUDRATE == 38400)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    else if (GPS_BAUDRATE == 57600)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    else if (GPS_BAUDRATE == 115200)
      SerialSendConfigToGPS(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    while (!FASTSERIAL.Flush(UART1))
      AVRTIME.SchedulerSleep(50);
  }
  AVRTIME.SchedulerSleep(200);
  FASTSERIAL.Begin(UART1, GPS_BAUDRATE);
  for (uint8_t i = 0; i < sizeof(UBLOX_INIT); i++)
  {
    FASTSERIAL.Write(UART1, pgm_read_byte(UBLOX_INIT + i));
    AVRTIME.SchedulerSleep(5);
  }
}

void GPS_SerialRead(uint8_t ReadData)
{
  switch (_step)
  {

  case 1:
    if (PREAMBLE2 == ReadData)
    {
      _step++;
      break;
    }
    _step = 0;

  case 0:
    if (PREAMBLE1 == ReadData)
      _step++;
    break;

  case 2:
    _step++;
    _ck_b = _ck_a = ReadData;
    break;

  case 3:
    _step++;
    _ck_b += (_ck_a += ReadData);
    _msg_id = ReadData;
    break;

  case 4:
    _step++;
    _ck_b += (_ck_a += ReadData);
    _payload_length = ReadData;
    break;

  case 5:
    _step++;
    _ck_b += (_ck_a += ReadData);
    _payload_length += (uint16_t)(ReadData << 8);
    if (_payload_length > UBLOX_BUFFER_SIZE)
    {
      _payload_length = 0;
      _step = 0;
    }
    _payload_counter = 0;
    break;

  case 6:
    _ck_b += (_ck_a += ReadData);
    if (_payload_counter < UBLOX_BUFFER_SIZE)
    {
      _buffer.bytes[_payload_counter] = ReadData;
    }
    if (++_payload_counter == _payload_length)
      _step++;
    break;

  case 7:
    _step++;
    if (_ck_a != ReadData)
      _step = 0;
    break;

  case 8:
    _step = 0;
    if (_ck_b != ReadData)
      break;
    GetAllGPSData();
  }
}

void GetAllGPSData(void)
{
  switch (_msg_id)
  {

  case MSG_POSLLH:
    GPS_Coordinates_Vector[1] = _buffer.posllh.longitude;
    GPS_Coordinates_Vector[0] = _buffer.posllh.latitude;
    GPS_Altitude = _buffer.posllh.altitude_msl / 10 / 100;
    GPS_3DFIX = Next_GPSFix;
    break;

  case MSG_STATUS:
    Next_GPSFix = ((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D));
    if (!Next_GPSFix)
      GPS_3DFIX = false;
    break;

  case MSG_SOL:
    Next_GPSFix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
    if (!Next_GPSFix)
      GPS_3DFIX = false;
    GPS_NumberOfSatellites = _buffer.solution.satellites;
    GPS_HDOP = _buffer.solution.position_DOP;
    break;

  case MSG_VELNED:
    GPS_Ground_Speed = _buffer.velned.speed_2d;
    GPS_Ground_Course = (uint16_t)(_buffer.velned.heading_2d / 10000);
    //APENAS PARA AERO MODE
    if (GPS_Ground_Speed > 100)
    {
      GPS_Ground_Course = WRap_180(GPS_Ground_Course * 10) / 10;
    }
    break;
  }
}