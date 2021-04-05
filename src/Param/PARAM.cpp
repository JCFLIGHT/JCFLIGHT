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
#include "ATO.h"
#include "FastSerial/PRINTF.h"
#include "FastSerial/FASTSERIAL.h"

ParamClass PARAM;

//#define OPERATOR_CHECK_EEPROM
//#define ERASE_ALL_EEPROM

Struct_JCF_Param_Adjustable JCF_Param;

const Resources_Of_Param Params_Table[] = {
    //NOME                                 ENDEREÇO NA EEPROM                    TIPO                    VARIAVEL                                    MIN            MAX              VALOR PADRÃO
#ifndef __AVR_ATmega2560__
    {"kP_Acc_AHRS",                        KP_ACC_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kP_Acc_AHRS,                     0,             255,             25},
    {"kI_Acc_AHRS",                        KI_ACC_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kI_Acc_AHRS,                     0,             255,             50},
    {"kP_Mag_AHRS",                        KP_MAG_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kP_Mag_AHRS,                     0,             255,             10},
    {"kI_Mag_AHRS",                        KI_MAG_AHRS_ADDR,                     VAR_8BITS,              &JCF_Param.kI_Mag_AHRS,                     0,             255,             0},
    {"AutoLaunch_AHRS_BankAngle",          AL_AHRS_BA_ADDR,                      VAR_8BITS,              &JCF_Param.AutoLaunch_AHRS_BankAngle,       0,             255,             25},
    {"AutoLaunch_IMU_BankAngle",           AL_IMU_BA_ADDR,                       VAR_16BITS,             &JCF_Param.AutoLaunch_IMU_BankAngle,        -1000,         1000,            -450},
    {"AutoLaunch_IMU_Swing",               AL_IMU_SWING_ADDR,                    VAR_8BITS,              &JCF_Param.AutoLaunch_IMU_Swing,            0,             255,             100},
    {"AutoLaunch_Trigger_Motor_Delay",     AL_TRIGGER_MOTOR_DELAY_ADDR,          VAR_16BITS,             &JCF_Param.AutoLaunch_Trigger_Motor_Delay,  0,             10000,           1500},
    {"AutoLaunch_Elevator",                AL_ELEVATOR_ADDR,                     VAR_8BITS,              &JCF_Param.AutoLaunch_Elevator,             0,             255,             18},
    {"AutoLaunch_SpinUp",                  AL_SPINUP_ADDR,                       VAR_16BITS,             &JCF_Param.AutoLaunch_SpinUp,               0,             2000,            100},
    {"AutoLaunch_SpinUp_Time",             AL_SPINUP_TIME_ADDR,                  VAR_16BITS,             &JCF_Param.AutoLaunch_SpinUp_Time,          0,             5000,            300},
    {"AutoLaunch_MaxThrottle",             AL_MAX_THROTTLE_ADDR,                 VAR_16BITS,             &JCF_Param.AutoLaunch_MaxThrottle,          1000,          2200,            1700},
    {"AutoLaunch_Exit",                    AL_EXIT_ADDR,                         VAR_16BITS,             &JCF_Param.AutoLaunch_Exit,                 0,             30000,           5000},
    {"AutoLaunch_Altitude",                AL_ALTITUDE_ADDR,                     VAR_8BITS,              &JCF_Param.AutoLaunch_Altitude,             0,             255,             0},
#endif
    {"Batt_Voltage_Factor",                BATT_VOLTAGE_FACTOR_ADDR,             VAR_FLOAT,              &JCF_Param.Batt_Voltage_Factor,             0,             1000,            259.489f},
    {"Batt_Amps_Per_Volt",                 BATT_AMPS_VOLT_ADDR,                  VAR_FLOAT,              &JCF_Param.Amps_Per_Volt,                   0,             1000,            62.0f},
#ifndef __AVR_ATmega2560__   
    {"Batt_Amps_OffSet",                   BATT_AMPS_OFFSET_ADDR,                VAR_FLOAT,              &JCF_Param.Amps_OffSet,                     0,             1000,            0},
    {"CrashCheck_BankAngle",               CC_BANKANGLE_ADDR,                    VAR_8BITS,              &JCF_Param.CrashCheck_BankAngle,            0,             255,             30},
    {"CrashCheck_Time",                    CC_TIME_ADDR,                         VAR_8BITS,              &JCF_Param.CrashCheck_Time,                 0,             255,             2},
    {"GimbalMinValue",                     GIMBAL_MIN_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMinValue,                  800,           2200,            1000},
    {"GimbalMiddleValue",                  GIMBAL_MID_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMiddleValue,               800,           2200,            1500},
    {"GimbalMaxValue",                     GIMBAL_MAX_ADDR,                      VAR_16BITS,             &JCF_Param.GimbalMaxValue,                  800,           2200,            2000},
    {"Land_CheckAcc",                      LAND_CHECKACC_ADDR,                   VAR_8BITS,              &JCF_Param.Land_Check_Acc,                  0,             255,             3},
    {"Land_LPF",                           LAND_LPF_ADDR,                        VAR_8BITS,              &JCF_Param.Land_LPF,                        0,             255,             1},
    {"ThrottleFactor",                     THROTTLE_FACTOR_ADDR,                 VAR_FLOAT,              &JCF_Param.Throttle_Factor,                 0,             1,               1.0f},
    {"AutoDisarm_Time",                    AUTODISARM_ADDR,                      VAR_8BITS,              &JCF_Param.AutoDisarm_Time,                 0,             255,             5},
    {"AutoDisarm_Throttle_Min",            AUTODISARM_THR_MIN_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_Throttle_Min,         800,           1500,            1100},
    {"AutoDisarm_YPR_Min",                 AUTODISARM_YPR_MIN_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_YPR_Min,              800,           1500,            1450},
    {"AutoDisarm_YPR_Max",                 AUTODISARM_YPR_MAX_ADDR,              VAR_16BITS,             &JCF_Param.AutoDisarm_YPR_Max,              800,           2200,            1550},
#endif
    {"AirPlane_Wheels",                    WHEELS_ADDR,                          VAR_8BITS,              &JCF_Param.AirPlane_Wheels,                 0,             255,             0},
#ifndef __AVR_ATmega2560__
    {"GPS_Baud_Rate",                      GPS_BAUDRATE_ADDR,                    VAR_8BITS,              &JCF_Param.GPS_Baud_Rate,                   0,             4,               0},
#endif
    {"Navigation_Vel",                     NAV_VEL_ADDR,                         VAR_16BITS,             &JCF_Param.Navigation_Vel,                  0,             400,             400},
    {"GPS_WP_Radius",                      WP_RADIUS_ADDR,                       VAR_8BITS,              &JCF_Param.GPS_WP_Radius,                   0,             255,             2},
    {"GPS_RTH_Land_Radius",                RTH_LAND_ADDR,                        VAR_8BITS,              &JCF_Param.GPS_RTH_Land,                    0,             255,             10},
#ifndef __AVR_ATmega2560__   
    {"GPS_TiltCompensation",               GPS_TILT_COMP_ADDR,                   VAR_8BITS,              &JCF_Param.GPS_TiltCompensation,            0,             100,             20},
    {"AirSpeed_Samples",                   AIRSPEED_SAMPLES_ADDR,                VAR_8BITS,              &JCF_Param.AirSpeed_Samples,                0,             255,             15},
#endif
    {"AirSpeed_Factor",                    AIRSPEED_FACTOR_ADDR,                 VAR_FLOAT,              &JCF_Param.AirSpeed_Factor,                 0,             100,             1.0f},
#ifndef __AVR_ATmega2560__  
    {"Arm_Time_Safety",                    ARM_TIME_SAFETY_ADDR,                 VAR_8BITS,              &JCF_Param.Arm_Time_Safety,                 0,             255,             2},
    {"Disarm_Time_Safety",                 DISARM_TIME_SAFETY_ADDR,              VAR_8BITS,              &JCF_Param.Disarm_Time_Safety,              0,             255,             2},
#endif
};

#define TABLE_COUNT (sizeof(Params_Table) / sizeof(Resources_Of_Param))

void ParamClass::Initialization(void)
{
#ifdef OPERATOR_CHECK_EEPROM
  Operator_Check_Values_In_Address(SIZE_OF_EEPROM);
#endif

#ifdef ERASE_ALL_EEPROM
  STORAGEMANAGER.Erase(INITIAL_ADDRESS_EEPROM_TO_CLEAR, FINAL_ADDRESS_EEPROM_TO_CLEAR);
#endif

  PARAM.Load_Sketch();
}

static void DefaultParams(const Resources_Of_Param *ParamValue)
{
  for (uint32_t Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
  {
    ParamValue = &Params_Table[Table_Counter];

    switch (ParamValue->Variable_Type)
    {

    case VAR_8BITS:
      *(uint8_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_8Bits(ParamValue->Address, ParamValue->DefaultValue);
      break;

    case VAR_16BITS:
      *(int16_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_16Bits(ParamValue->Address, ParamValue->DefaultValue);
      break;

    case VAR_32BITS:
      *(int32_t *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_32Bits(ParamValue->Address, ParamValue->DefaultValue);
      break;

    case VAR_FLOAT:
      *(float *)ParamValue->Ptr = ParamValue->DefaultValue;
      STORAGEMANAGER.Write_Float(ParamValue->Address, ParamValue->DefaultValue);
      break;
    }
  }
}

void ParamClass::Load_Sketch(void)
{
  static uint8_t System_Version = STORAGEMANAGER.Read_8Bits(PARAM_LINK_ADDR);
  const Resources_Of_Param *ParamValue;

  if (PARAM.Actual_Format_Version != System_Version)
  {
    LOG("Restaurando os valores de fabrica dos parametros...");
    DefaultParams(ParamValue);
    LOG("Ok...Parametros reconfigurados!");
    STORAGEMANAGER.Write_8Bits(PARAM_LINK_ADDR, PARAM.Actual_Format_Version);
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

      case VAR_FLOAT:
        *(float *)ParamValue->Ptr = STORAGEMANAGER.Read_Float(ParamValue->Address);
        break;
      }
    }
  }
}

static void Param_Set_Value(const Resources_Of_Param *VariablePointer, const int32_t New_Value, const float New_Value_Float)
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

  case VAR_FLOAT:
    *(float *)VariablePointer->Ptr = (float)New_Value_Float;
    STORAGEMANAGER.Write_Float(VariablePointer->Address, New_Value_Float);
    break;
  }
}

static void Param_Print_Value(const Resources_Of_Param *VariablePointer)
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

  case VAR_FLOAT:
    DEBUG("%.4f", STORAGEMANAGER.Read_Float(VariablePointer->Address));
    return;
  }
  DEBUG("%d", New_Value);
}

void ParamClass::Set_And_Save(char *ParamCommandLine)
{
  const Resources_Of_Param *ParamValue;
  char *PtrInput = NULL;
  float New_Value_Float = 0;
  int32_t New_Value = 0;
  uint32_t Table_Counter;
  uint32_t StringLength;

  while (*ParamCommandLine == ' ')
  {
    ++ParamCommandLine;
  }

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
    LINE_SPACE;
  }
  else if ((PtrInput = strstr(ParamCommandLine, "=")) != NULL)
  {
    PtrInput++;
    New_Value = ATO_Int(PtrInput);
    New_Value_Float = ATO_Float(PtrInput);
    for (Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
    {
      ParamValue = &Params_Table[Table_Counter];
      if (strncasecmp(ParamCommandLine, Params_Table[Table_Counter].Param_Name, strlen(Params_Table[Table_Counter].Param_Name)) == 0)
      {
        if (New_Value >= Params_Table[Table_Counter].Value_Min && New_Value <= Params_Table[Table_Counter].Value_Max)
        {
          if (Params_Table[Table_Counter].Variable_Type == VAR_FLOAT)
          {
            Param_Set_Value(ParamValue, 0, New_Value_Float);
          }
          else
          {
            Param_Set_Value(ParamValue, New_Value, 0);
          }
          DEBUG_WITHOUT_NEW_LINE("%s setado para ", Params_Table[Table_Counter].Param_Name);
          Param_Print_Value(ParamValue);
          LINE_SPACE;
        }
        else if (New_Value < Params_Table[Table_Counter].Value_Min)
        {
          LOG_PARAM_ERROR("O valor setado esta fora do limite minimo!");
          LINE_SPACE;
        }
        else if (New_Value > Params_Table[Table_Counter].Value_Max)
        {
          LOG_PARAM_ERROR("O valor setado esta fora do limite maximo!");
          LINE_SPACE;
        }
        return;
      }
    }
    LOG_PARAM_ERROR("Parametro nao encontrado na lista");
    LINE_SPACE;
  }
  else if (strncasecmp(ParamCommandLine, "relatorio", 9) == 0)
  {
    for (Table_Counter = 0; Table_Counter < TABLE_COUNT; Table_Counter++)
    {
      ParamValue = &Params_Table[Table_Counter];
      DEBUG_WITHOUT_NEW_LINE("%s = ", Params_Table[Table_Counter].Param_Name);
      Param_Print_Value(ParamValue);
    }
    LINE_SPACE;
  }
  else if (strncasecmp(ParamCommandLine, "formatar", 8) == 0)
  {
    DEBUG("Restaurando os valores de fabrica dos parametros...");
    DefaultParams(ParamValue);
    DEBUG("Ok...Parametros reconfigurados!");
    LINE_SPACE;
  }
  else if (strncasecmp(ParamCommandLine, "reiniciar", 9) == 0)
  {
  }
  else if (strncasecmp(ParamCommandLine, "sair", 4) == 0)
  {
    LINE_SPACE;
    DEBUG("Modo CLI desativado.")
    PARAM.PrintMessage = false;
    GCS.CliMode = false;
  }
  else
  {
    LOG_PARAM_ERROR("Comando invalido!");
    LINE_SPACE;
  }
}

void ParamClass::SerialProcess(void)
{
  if (!GCS.CliMode)
  {
    return;
  }

  if (!PARAM.PrintMessage)
  {
    LINE_SPACE;
    DEBUG("Modo CLI ativado!");
    LINE_SPACE;
    DEBUG("Comandos:");
    DEBUG("Ajuda; para listar os parametros disponiveis.");
    DEBUG("Relatorio; para listar todos os parametros com os valores atuais.");
    DEBUG("Formatar; para voltar todos os parametros ao padrao de fabrica.");
    DEBUG("Reiniciar; para reiniciar o sistema da JCFLIGHT.");
    DEBUG("Sair; para sair do modo CLI.");
    LINE_SPACE;
    PARAM.PrintMessage = true;
  }

  while (FASTSERIAL.Available(UART_NUMB_0))
  {
    uint8_t SerialReadCommand = FASTSERIAL.Read(UART_NUMB_0);

    PARAM.SerialBuffer[PARAM.SerialBufferIndex++] = SerialReadCommand; //BUFFER

    if (PARAM.SerialBufferIndex && ((strstr(PARAM.SerialBuffer, ";")) != NULL))
    {
      PARAM.SerialBuffer[PARAM.SerialBufferIndex] = 0;
      PARAM.Set_And_Save(PARAM.SerialBuffer);
    }
  }
  if (PARAM.SerialBufferIndex > 0)
  {
    PARAM.SerialBuffer[PARAM.SerialBufferIndex--] = 0;
  }
}