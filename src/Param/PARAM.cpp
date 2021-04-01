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

/**************************
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
***************************/

#include "PARAM.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "StorageManager/EEPROMCHECK.h"
#include "Common/ENUM.h"
#include "Build/BOARDDEFS.h"
#include "IOMCU/IOMCU.h"
#include "FastSerial/PRINTF.h"
#include "FastSerial/FASTSERIAL.h"

ParamClass PARAM;

#ifdef __AVR_ATmega2560__
#define OPTIMIZE_LIST
#endif

//#define OPERATOR_CHECK_EEPROM
//#define ERASE_ALL_EEPROM

typedef struct JCF_Param_Adjustable
{
#ifndef OPTIMIZE_LIST
  uint8_t kP_Acc_AHRS;
  uint8_t kI_Acc_AHRS;
  uint8_t kP_Mag_AHRS;
  uint8_t kI_Mag_AHRS;
  uint8_t AutoLaunch_AHRS_BankAngle;
  uint16_t AutoLaunch_IMU_BankAngle;
  uint8_t AutoLaunch_IMU_Swing;
  uint16_t AutoLaunch_Trigger_Motor_Delay;
  uint8_t AutoLaunch_Elevator;
  uint16_t AutoLaunch_SpinUp;
  uint16_t AutoLaunch_SpinUp_Time;
  uint16_t AutoLaunch_MaxThrottle;
  uint16_t AutoLaunch_Exit;
  uint8_t AutoLaunch_Altitude;
#endif
  uint32_t Batt_Voltage_Factor;
  uint16_t Amps_Per_Volt;
#ifndef OPTIMIZE_LIST
  uint16_t Amps_OffSet;
  uint8_t CrashCheck_BankAngle;
  uint8_t CrashCheck_Time;
  uint16_t GimbalMinValue;
  uint16_t GimbalMiddleValue;
  uint16_t GimbalMaxValue;
  uint8_t Land_Check_Acc;
  uint8_t Land_LPF;
  uint8_t Throttle_Factor;
  uint8_t AutoDisarm_Time;
  uint16_t AutoDisarm_Throttle_Min;
  uint16_t AutoDisarm_YPR_Min;
  uint16_t AutoDisarm_YPR_Max;
#endif
  uint8_t AirPlane_Wheels;
#ifndef OPTIMIZE_LIST
  uint8_t GPS_Baud_Rate;
#endif
  uint16_t Navigation_Vel;
  uint8_t GPS_WP_Radius;
  uint8_t GPS_RTH_Land;
#ifndef OPTIMIZE_LIST
  uint8_t GPS_TiltCompensation;
  uint8_t AirSpeed_Samples;
#endif
  uint16_t AirSpeed_Factor;
} Struct_JCF_Param_Adjustable;

Struct_JCF_Param_Adjustable JCF_Param;

typedef struct
{
  const char *Param_Name;
  const uint16_t Address;
  const uint8_t Variable_Type;
  void *Ptr;
  const int32_t Value_Min;
  const int32_t Value_Max;
  const int32_t DefaultValue;
} Requesited_Values_Of_Param;

const Requesited_Values_Of_Param Params_Table[] = {
    //NOME                                 ENDEREÇO NA EEPROM                    TIPO                    VARIAVEL                                    MIN            MAX              VALOR PADRÃO
#ifndef OPTIMIZE_LIST
    {"kP_Acc_AHRS",                        KP_ACC_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kP_Acc_AHRS,                     0,             255,             25},
    {"kI_Acc_AHRS",                        KI_ACC_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kI_Acc_AHRS,                     0,             255,             50},
    {"kP_Mag_AHRS",                        KP_MAG_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kP_Mag_AHRS,                     0,             255,             10},
    {"kI_Mag_AHRS",                        KI_MAG_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kI_Mag_AHRS,                     0,             255,             0},
    {"AutoLaunch_AHRS_BankAngle",          AL_AHRS_BA_ADDR,                      VAR_8BITS,              &JCF_Param.AutoLaunch_AHRS_BankAngle,       0,             255,             25},
    {"AutoLaunch_IMU_BankAngle",           AL_IMU_BA_ADDR,                       VAR_16BITS,             &JCF_Param.AutoLaunch_IMU_BankAngle,        0,             1000,            450},
    {"AutoLaunch_IMU_Swing",               AL_IMU_SWING_ADDR,                    VAR_8BITS,              &JCF_Param.AutoLaunch_IMU_Swing,            0,             255,             100},
    {"AutoLaunch_Trigger_Motor_Delay",     AL_TRIGGER_MOTOR_DELAY_ADDR,          VAR_16BITS,             &JCF_Param.AutoLaunch_Trigger_Motor_Delay,  0,             10000,           1500},
    {"AutoLaunch_Elevator",                AL_ELEVATOR_ADDR,                     VAR_8BITS,              &JCF_Param.AutoLaunch_Elevator,             0,             255,             18},
    {"AutoLaunch_SpinUp",                  AL_SPINUP_ADDR,                       VAR_16BITS,             &JCF_Param.AutoLaunch_SpinUp,               0,             2000,            100},
    {"AutoLaunch_SpinUp_Time",             AL_SPINUP_TIME_ADDR,                  VAR_16BITS,             &JCF_Param.AutoLaunch_SpinUp_Time,          0,             5000,            300},
    {"AutoLaunch_MaxThrottle",             AL_MAX_THROTTLE_ADDR,                 VAR_16BITS,             &JCF_Param.AutoLaunch_MaxThrottle,          1000,          2200,            1700},
    {"AutoLaunch_Exit",                    AL_EXIT_ADDR,                         VAR_16BITS,             &JCF_Param.AutoLaunch_Exit,                 0,             30000,           5000},
    {"AutoLaunch_Altitude",                AL_ALTITUDE_ADDR,                     VAR_8BITS,              &JCF_Param.AutoLaunch_Altitude,             0,             255,             0},
#endif
    {"Batt_Voltage_Factor",                BATT_VOLTAGE_FACTOR_ADDR,             VAR_32BITS,             &JCF_Param.Batt_Voltage_Factor,             0,             400000,          259489},
    {"Batt_Amps_Volt",                     BATT_AMPS_VOLT_ADDR,                  VAR_16BITS,             &JCF_Param.Amps_Per_Volt,                   0,             1000,            620},
#ifndef OPTIMIZE_LIST   
    {"Batt_Amps_OffSet",                   BATT_AMPS_OFFSET_ADDR,                VAR_16BITS,             &JCF_Param.Amps_OffSet,                     0,             1000,            0},
    {"CrashCheck_BankAngle",               CC_BANKANGLE_ADDR,                    VAR_8BITS,              &JCF_Param.CrashCheck_BankAngle,            0,             255,             30},
    {"CrashCheck_Time",                    CC_TIME_ADDR,                         VAR_8BITS,              &JCF_Param.CrashCheck_Time,                 0,             255,             2},
    {"GimbalMinValue",                     GIMBAL_MIN_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMinValue,                  800,           2200,            1000},
    {"GimbalMiddleValue",                  GIMBAL_MID_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMiddleValue,               800,           2200,            1500},
    {"GimbalMaxValue",                     GIMBAL_MAX_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMaxValue,                  800,           2200,            2000},
    {"Land_CheckAcc",                      LAND_CHECKACC_ADDR,                   VAR_8BITS,              &JCF_Param.Land_Check_Acc,                  0,             255,             3},
    {"Land_LPF",                           LAND_LPF_ADDR,                        VAR_8BITS,              &JCF_Param.Land_LPF,                        0,             255,             1},
    {"ThrottleFactor",                     THROTTLE_FACTOR_ADDR,                 VAR_16BITS,             &JCF_Param.Throttle_Factor,                 0,             255,             1},
    {"AutoDisarm_Time",                    AUTODISARM_ADDR,                      VAR_8BITS,              &JCF_Param.AutoDisarm_Time,                 0,             255,             5},
    {"AutoDisarm_Throttle_Min",            AUTODISARM_THR_MIN_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_Throttle_Min,         800,           1500,            1100},
    {"AutoDisarm_YPR_Min",                 AUTODISARM_YPR_MIN_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_YPR_Min,              800,           1500,            1450},
    {"AutoDisarm_YPR_Max",                 AUTODISARM_YPR_MAX_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_YPR_Max,              800,           2200,            1550},
#endif
    {"AirPlane_Wheels",                    WHEELS_ADDR,                          VAR_8BITS,              &JCF_Param.AirPlane_Wheels,                 0,             255,             0},
#ifndef OPTIMIZE_LIST
    {"GPS_Baud_Rate",                      GPS_BAUDRATE_ADDR,                    VAR_8BITS,              &JCF_Param.GPS_Baud_Rate,                   0,             4,               0},
#endif
    {"Navigation_Vel",                     NAV_VEL_ADDR,                         VAR_16BITS,             &JCF_Param.Navigation_Vel,                  0,             400,             400},
    {"GPS_WP_Radius",                      WP_RADIUS_ADDR,                       VAR_8BITS,              &JCF_Param.GPS_WP_Radius,                   0,             255,             2},
    {"GPS_RTH_Land",                       RTH_LAND_ADDR,                        VAR_8BITS,              &JCF_Param.GPS_RTH_Land,                    0,             255,             10},
#ifndef OPTIMIZE_LIST   
    {"GPS_TiltCompensation",               GPS_TILT_COMP_ADDR,                   VAR_8BITS,              &JCF_Param.GPS_TiltCompensation,            0,             100,             20},
    {"AirSpeed_Samples",                   AIRSPEED_SAMPLES_ADDR,                VAR_8BITS,              &JCF_Param.AirSpeed_Samples,                0,             255,             15},
#endif
    {"AirSpeed_Factor",                    AIRSPEED_FACTOR_ADDR,                 VAR_16BITS,             &JCF_Param.AirSpeed_Factor,                 0,             5000,            1},
};

#define TABLE_COUNT (sizeof(Params_Table) / sizeof(Requesited_Values_Of_Param))

static char SerialBuffer[48];

uint8_t Actual_Format_Version = 10; //1.0

static uint32_t SerialBufferIndex = 0;

void ParamClass::Initialization(void)
{
#ifdef OPERATOR_CHECK_EEPROM
  Operator_Check_Values_In_Address(SIZE_OF_EEPROM);
#endif

#ifdef ERASE_ALL_EEPROM
  STORAGEMANAGER.Erase(INITIAL_ADDRESS_EEPROM_TO_CLEAR, FINAL_ADDRESS_EEPROM_TO_CLEAR);
#endif

  //PARAM.Load_Sketch();
}

static void DefaultParams(const Requesited_Values_Of_Param *ParamValue)
{
  for (uint32_t Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
  {
    ParamValue = &Params_Table[Table_Counter];

    switch (ParamValue->Variable_Type)
    {

    case VAR_8BITS:
      *(uint8_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_8Bits(ParamValue->Address ,ParamValue->DefaultValue);
      break;

    case VAR_16BITS:
      *(int16_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_16Bits(ParamValue->Address, ParamValue->DefaultValue);
      break;

    case VAR_32BITS:
      *(int32_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_32Bits(ParamValue->Address, ParamValue->DefaultValue);
      break;
    }
  }
}

void ParamClass::Load_Sketch(void)
{
  static uint8_t System_Version = STORAGEMANAGER.Read_8Bits(1800);
  const Requesited_Values_Of_Param *ParamValue;

  if (Actual_Format_Version != System_Version)
  {
    LOG("Restaurando os valores de fabrica dos parametros...");
    DefaultParams(ParamValue);
    LOG("Ok...Parametros reconfigurados!");
    STORAGEMANAGER.Write_8Bits(1800, Actual_Format_Version);
    return;
  }
  else
  {
  for (uint32_t Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
  {
    ParamValue = &Params_Table[Table_Counter];

    switch (ParamValue->Variable_Type)
    {

    case VAR_8BITS:
      *(uint8_t *)ParamValue->Ptr = STORAGEMANAGER.Read_8Bits(ParamValue->Address);
      break;

    case VAR_16BITS:
      *(int16_t *)ParamValue->Ptr = STORAGEMANAGER.Read_16Bits(ParamValue->Address);
      break;

    case VAR_32BITS:
      *(int32_t *)ParamValue->Ptr = STORAGEMANAGER.Read_32Bits(ParamValue->Address);
      break;
    }
  }
}
}

static void Param_Set_Value(const Requesited_Values_Of_Param *VariablePointer, const int32_t New_Value)
{
  switch (VariablePointer->Variable_Type)
  {

  case VAR_8BITS:
    *(uint8_t *)VariablePointer->Ptr = (uint8_t)New_Value;
    STORAGEMANAGER.Write_8Bits(VariablePointer->Address, New_Value);
    break;

  case VAR_16BITS:
    *(int16_t *)VariablePointer->Ptr = (int16_t)New_Value;
    STORAGEMANAGER.Write_16Bits(VariablePointer->Address, New_Value);
    break;

  case VAR_32BITS:
    *(int32_t *)VariablePointer->Ptr = (int32_t)New_Value;
    STORAGEMANAGER.Write_32Bits(VariablePointer->Address, New_Value);
    break;
  }
}

static void Param_Print_Value(const Requesited_Values_Of_Param *VariablePointer)
{
  int32_t New_Value = 0;

  switch (VariablePointer->Variable_Type)
  {

  case VAR_8BITS:
    New_Value = STORAGEMANAGER.Read_8Bits(VariablePointer->Address);
    break;

  case VAR_16BITS:
    New_Value = STORAGEMANAGER.Read_16Bits(VariablePointer->Address);
    break;

  case VAR_32BITS:
    New_Value = STORAGEMANAGER.Read_32Bits(VariablePointer->Address);
    DEBUG("%ld", New_Value);
    return;
  }
  DEBUG("%d", New_Value);
}

void ParamClass::Set_And_Save(char *ParamCommandLine)
{
  const Requesited_Values_Of_Param *ParamValue;
  char *PtrInput = NULL;
  int32_t New_Value = 0;
  uint32_t Table_Counter;
  uint32_t StringLength;

  StringLength = strlen(ParamCommandLine);

  if (StringLength == 0)
  {
    return;
  }
   else if (strncasecmp(ParamCommandLine, "ajuda", 5) == 0)
  {
    for (Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
    {
      ParamValue = &Params_Table[Table_Counter];
      DEBUG("%s", Params_Table[Table_Counter].Param_Name);
    }
    DEBUG("\r");
  }
  else if ((PtrInput = strstr(ParamCommandLine, "=")) != NULL)
  {
    PtrInput++;
    StringLength--;
    New_Value = atoi(PtrInput);
    for (Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
    {
      ParamValue = &Params_Table[Table_Counter];
      if (strncasecmp(ParamCommandLine, Params_Table[Table_Counter].Param_Name, strlen(Params_Table[Table_Counter].Param_Name)) == 0)
      {
        if (New_Value >= Params_Table[Table_Counter].Value_Min && New_Value <= Params_Table[Table_Counter].Value_Max)
        {
          Param_Set_Value(ParamValue, New_Value);
          DEBUG_WITHOUT_NEW_LINE("%s setado para ", Params_Table[Table_Counter].Param_Name);
          Param_Print_Value(ParamValue);
        }
        else if (New_Value < Params_Table[Table_Counter].Value_Min)
        {
          LOG_PARAM_ERROR("O valor setado esta fora do limite minimo!");
        }
        else if (New_Value > Params_Table[Table_Counter].Value_Max)
        {
          LOG_PARAM_ERROR("O valor setado esta fora do limite maximo!");
        }
        return;
      }
    }
    LOG_PARAM_ERROR("Parametro nao encontrado na lista");
  }
  else if (strncasecmp(ParamCommandLine, "relatorio", 9) == 0)
  {
    for (Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
    {
      ParamValue = &Params_Table[Table_Counter];
      DEBUG_WITHOUT_NEW_LINE("%s = ", Params_Table[Table_Counter].Param_Name);
      Param_Print_Value(ParamValue);
    }
    DEBUG("\r");
  }
  else if (strncasecmp(ParamCommandLine, "formatar", 8) == 0)
  {
  }
  else if (strncasecmp(ParamCommandLine, "reiniciar", 9) == 0)
  {
  }
  else if (strncasecmp(ParamCommandLine, "sair", 4) == 0)
  {
    GCS.CliMode = false;
  }
  else
  {
    LOG_PARAM_ERROR("Comando invalido!");
  }
}

void ParamClass::SerialProcess(void)
{
  if (!GCS.CliMode)
  {
    return;
  }

  while (FASTSERIAL.Available(UART_NUMB_0))
  {
    uint8_t SerialReadCommand = FASTSERIAL.Read(UART_NUMB_0);

    SerialBuffer[SerialBufferIndex++] = SerialReadCommand;

    if (SerialBufferIndex && ((strstr(SerialBuffer, ";")) != NULL))
    {
      SerialBuffer[SerialBufferIndex] = 0;
      PARAM.Set_And_Save(SerialBuffer);
    }
  }
  if (SerialBufferIndex > 0)
  {
    SerialBuffer[SerialBufferIndex--] = 0;
  }
}