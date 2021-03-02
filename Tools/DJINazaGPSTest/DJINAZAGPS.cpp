#include "DJINAZAGPS.h"

#include "math.h"    //fabsf
#include "Arduino.h" //max,min

typedef struct
{
  uint8_t res[4]; //0
  uint8_t fw[4];  //4
  uint8_t hw[4];  //8
} NazaGPS_Version;

//COMPASS DATA
typedef struct
{
  uint16_t MagAxisRoll;  //0
  uint16_t MagAxisPitch; //2
  uint16_t MagAxisYaw;   //4
} Struct_NazaGPS_Mag_Data;

//GPS DATA
typedef struct
{
  uint32_t Unused_Time;
  int32_t Longitude; //4
  int32_t Latitude;  //8
  int32_t Altitude;  //12
  int32_t Unused_Horizontal_Acc;
  int32_t Unused_Vertical_Acc;
  int32_t Null;
  int32_t NED_North;
  int32_t NED_East;
  int32_t NED_Down;      //39
  uint16_t Position_DOP; //40
  uint16_t Unused_VDOP;
  uint16_t Unused_NDOP;
  uint16_t Unused_EDOP;
  uint8_t Satellites; //48
  uint8_t Null3;
  uint8_t Fix_Type; //50
  uint8_t Null4;
  uint8_t Unused_FS; //52
  uint8_t Null5;
  uint8_t Null6;
  uint8_t Data_Mask; //55
} Struct_NazaGPS_GPS_Data;

//PROTOCOLOS
enum
{
  HEADER1 = 0x55,
  HEADER2 = 0xAA,
  ID_NAV = 0x10,
  ID_MAG = 0x20
};

//GPS FIX
enum
{
  NO_FIX = 0,
  FIX_2D = 2,
  FIX_3D = 3
};

//GPS FIX
typedef enum
{
  GPS_NO_FIX = 0,
  GPS_FIX_2D,
  GPS_FIX_3D
} GPS_Fix_Type;

//LEITURA DE TODOS OS DADOS SERIAIS DO GPS
static union
{
  Struct_NazaGPS_Mag_Data NazaGPS_Mag_Data;
  Struct_NazaGPS_GPS_Data NazaGPS_GPS_Data;
  NazaGPS_Version Version;
  uint8_t bytes[256];
} NazaGPS_Buffer_Read;

//PRINCIPAIS INFORMAÇÕES DO GPS
typedef struct
{
  int32_t Latitude;
  int32_t Longitude;
  int32_t Altitude;
} Struct_GPS_Location_Data;

typedef struct
{
  GPS_Fix_Type FixType;                       //INSTANCIA
  Struct_GPS_Location_Data GPS_Location_Data; //INSTANCIA
  uint8_t GPS_NumSat;
  int16_t GPS_Read_Compass[3];
  int16_t VelocityNE[3];
  int16_t GroundSpeed;
  int16_t GroundCourse;
  uint16_t GPS_HDOP_State;
} Struct_SolutionData;

Struct_SolutionData GPSSolutionData; //INSTANCIA

//VERIFICÇÃO DE PACOTES DE DADOS DO GPS
static uint8_t CheckPacket_A;
static uint8_t CheckPacket_B;

//ESTADO DE MAQUINA
static bool GPS_New_Information;
static bool Next_Packet;
static uint8_t Step;
static uint8_t Decode_Message_ID;
static uint16_t Payload_Lenght;
static uint16_t Payload_Counter;

//VARIAVEIS DE SAÍDA
uint8_t DJINaza_Num_Sat;
uint8_t DJINaza_Fix_State;
uint16_t DJINaza_HDOP;
int16_t DJINaza_Compass_Roll;
int16_t DJINaza_Compass_Pitch;
int16_t DJINaza_Compass_Yaw;
int32_t DJINaza_Latitude;
int32_t DJINaza_Longitude;
int32_t DJINaza_Altitude;
int32_t DJINaza_GroundCourse;
int32_t DJINaza_GroundSpeed;

int16_t Decode16BitsValues(uint16_t Index, uint8_t Data_Mask)
{
  union
  {
    uint16_t UnsignedShort;
    uint8_t Byte[2];
  } Value;
  Value.UnsignedShort = Index;
  Value.Byte[0] ^= Data_Mask;
  Value.Byte[1] ^= Data_Mask;
  return Value.UnsignedShort;
}

uint16_t HDOPMaxError(uint32_t Value)
{
  return (Value > 9999) ? 9999 : Value;
}

int32_t Decode32BitsValues(uint32_t Index, uint8_t Data_Mask)
{
  union
  {
    uint32_t UnsignedLong;
    uint8_t Byte[4];
  } Value;
  Value.UnsignedLong = Index;
  Value.Byte[0] ^= Data_Mask;
  Value.Byte[1] ^= Data_Mask;
  Value.Byte[2] ^= Data_Mask;
  Value.Byte[3] ^= Data_Mask;
  return Value.UnsignedLong;
}

float Fast_Atan2(float y, float x)
{
#define atanPolyCoef1 3.14551665884836e-07f
#define atanPolyCoef2 0.99997356613987f
#define atanPolyCoef3 0.14744007058297684f
#define atanPolyCoef4 0.3099814292351353f
#define atanPolyCoef5 0.05030176425872175f
#define atanPolyCoef6 0.1471039133652469f
#define atanPolyCoef7 0.6444640676891548f

  float res, absX, absY;
  absX = fabsf(x);
  absY = fabsf(y);
  res = max(absX, absY);
  if (res)
  {
    res = min(absX, absY) / res;
  }
  else
  {
    res = 0.0f;
  }
  res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
  if (absY > absX)
  {
    res = (3.14159265358979323846f / 2.0f) - res;
  }
  if (x < 0)
  {
    res = 3.14159265358979323846f - res;
  }
  if (y < 0)
  {
    res = -res;
  }
  return res;
}

static void NazaGPS_Check_Valid_Data()
{
  uint8_t Data_Mask;
  uint8_t Data_Mask_Mag;

  switch (Decode_Message_ID)
  {

  case ID_NAV:
  {
    Data_Mask = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Data_Mask;

    //DECODE LATITUDE,LONGITUDE & ALTITUDE
    GPSSolutionData.GPS_Location_Data.Longitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Longitude, Data_Mask);
    GPSSolutionData.GPS_Location_Data.Latitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Latitude, Data_Mask);
    GPSSolutionData.GPS_Location_Data.Altitude = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Altitude, Data_Mask) / 10.0f;

    //DECODE GPS FIX
    uint8_t FixType = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Fix_Type ^ Data_Mask;
    if (FixType == FIX_2D)
    {
      GPSSolutionData.FixType = GPS_FIX_2D;
    }
    else if (FixType == FIX_3D)
    {
      GPSSolutionData.FixType = GPS_FIX_3D;
    }
    else
    {
      GPSSolutionData.FixType = GPS_NO_FIX;
    }

    //DECODE A VELOCIDADE NED
    GPSSolutionData.VelocityNE[0] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_North, Data_Mask);
    GPSSolutionData.VelocityNE[1] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_East, Data_Mask);
    GPSSolutionData.VelocityNE[2] = Decode32BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.NED_Down, Data_Mask);

    //DECODE DO PDOP QUE AGORA VAI SER HDOP
    uint16_t Position_DOP = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Position_DOP, Data_Mask);
    GPSSolutionData.GPS_HDOP_State = HDOPMaxError(Position_DOP);

    //DECODE O NÚMERO DE SATELITES
    GPSSolutionData.GPS_NumSat = NazaGPS_Buffer_Read.NazaGPS_GPS_Data.Satellites;

    //CALCULA O GROUND SPEED DADO PELO GPS A PARTIR DAS VELOCIDADES NORTH & EAST
    GPSSolutionData.GroundSpeed = sqrtf(powf(GPSSolutionData.VelocityNE[0], 2) + powf(GPSSolutionData.VelocityNE[1], 2));

    //CALCULA O GROUND COURSE DADO PELO GPS A PARTIR DAS VELOCIDADES NORTH & EASTH
    GPSSolutionData.GroundCourse = (uint16_t)(fmodf((Fast_Atan2(GPSSolutionData.VelocityNE[1], GPSSolutionData.VelocityNE[0]) * 57.295779513082320876798154814105f) + 3600.0f, 3600.0f));

    //NOVAS INFORMAÇÕES
    GPS_New_Information = true;
    break;
  }

  case ID_MAG:
  {
    Data_Mask_Mag = (NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisYaw) & 0xFF;
    Data_Mask_Mag = (((Data_Mask_Mag ^ (Data_Mask_Mag >> 4)) & 0x0F) | ((Data_Mask_Mag << 3) & 0xF0)) ^ (((Data_Mask_Mag & 0x01) << 3) | ((Data_Mask_Mag & 0x01) << 7));

    //DECODE OS EIXOS X,Y E Z DO MAGNETOMETRO
    GPSSolutionData.GPS_Read_Compass[0] = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisRoll, Data_Mask_Mag);
    GPSSolutionData.GPS_Read_Compass[1] = Decode16BitsValues(NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisPitch, Data_Mask_Mag);
    GPSSolutionData.GPS_Read_Compass[2] = NazaGPS_Buffer_Read.NazaGPS_Mag_Data.MagAxisYaw ^ (Data_Mask_Mag << 8);
    break;
  }
  }

  if (GPS_New_Information) //ATUALIZA A DECODIFICAÇÃO
  {
    GPS_New_Information = false;
  }
}

void DjiNazaGpsNewFrame(uint8_t SerialReceiverBuffer)
{

  switch (Step)
  {

  case 0: //SINCRONIZAÇÃO DE DATA (0X55)
    if (HEADER1 == SerialReceiverBuffer)
    { //SEM FALHA
      Next_Packet = false;
      Step++;
    }
    break;

  case 1: //SINCRONIZAÇÃO DE DATA (0XAA)
    if (HEADER2 != SerialReceiverBuffer)
    { //FALHA
      Step = 0;
      break;
    }
    Step++; //SEM FALHA
    break;

  case 2:
    Step++;
    CheckPacket_B = CheckPacket_A = SerialReceiverBuffer; //RESETA OS PACOTES DE DADOS
    //ID DO PACOTE DE DADOS PARA DECODIFICAÇÃO
    Decode_Message_ID = SerialReceiverBuffer;
    break;

  case 3:
    //LEITURA DO PACOTE DE DADOS
    Step++;
    CheckPacket_B += (CheckPacket_A += SerialReceiverBuffer); //PACOTE DE DADOS OK
    Payload_Lenght = SerialReceiverBuffer;                    //LEITURA DE VALORES LOWBYTE
    if (Payload_Lenght > 256)
    { //VALOR DO PACOTE MAIOR QUE 256 BYTES?SIM...FALHA,RETORNA AO STEP 0
      Step = 0;
      break;
    }
    //REINICIA PARA NOVOS DADOS
    Payload_Counter = 0;
    if (Payload_Lenght == 0)
    {
      Step = 6;
    }
    break;

  case 4:
    //PACOTE DE DADOS OK
    CheckPacket_B += (CheckPacket_A += SerialReceiverBuffer);
    if (Payload_Counter < 256)
    {
      NazaGPS_Buffer_Read.bytes[Payload_Counter] = SerialReceiverBuffer;
    }
    if (Payload_Counter++ >= Payload_Lenght)
    {
      Step++;
    }
    break;

  case 5:
    Step++;
    if (CheckPacket_A != SerialReceiverBuffer)
    {
      Next_Packet = true; //FALHA
    }
    break;

  case 6:
    Step = 0;
    if (CheckPacket_B != SerialReceiverBuffer)
    {
      break; //FALHA
    }
    if (Next_Packet)
    {
      break;
    }
    NazaGPS_Check_Valid_Data();
  }
  DJINaza_Num_Sat = GPSSolutionData.GPS_NumSat;
  DJINaza_Fix_State = GPSSolutionData.FixType;
  DJINaza_HDOP = GPSSolutionData.GPS_HDOP_State;
  DJINaza_Compass_Roll = GPSSolutionData.GPS_Read_Compass[0];
  DJINaza_Compass_Pitch = GPSSolutionData.GPS_Read_Compass[1];
  DJINaza_Compass_Yaw = GPSSolutionData.GPS_Read_Compass[2];
  DJINaza_Latitude = GPSSolutionData.GPS_Location_Data.Latitude;
  DJINaza_Longitude = GPSSolutionData.GPS_Location_Data.Longitude;
  DJINaza_Altitude = GPSSolutionData.GPS_Location_Data.Altitude;
  DJINaza_GroundCourse = GPSSolutionData.GroundCourse;
  DJINaza_GroundSpeed = GPSSolutionData.GroundSpeed;
}
