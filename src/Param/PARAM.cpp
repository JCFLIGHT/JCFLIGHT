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

#include "PARAM.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "StorageManager/EEPROMCHECK.h"
#include "Common/ENUM.h"
#include "StorageManager/ERASE.h"
#include "Build/BOARDDEFS.h"
#include "FastSerial/PRINTF.h"

ParamClass PARAM;

//#define OPERATOR_CHECK_EEPROM
//#define ERASE_ALL_EEPROM

typedef struct JCF_Param_Adjustable
{
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
  uint32_t Batt_Voltage_Factor;
  uint16_t Amps_Per_Volt;
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
  uint8_t AirPlane_Wheels;
  uint8_t GPS_Baud_Rate;
  uint16_t Navigation_Vel;
  uint8_t GPS_WP_Radius;
  uint8_t GPS_RTH_Land;
  uint8_t GPS_TiltCompensation;
  uint8_t AirSpeed_Samples;
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
} Requesited_Values_Of_Param;

const Requesited_Values_Of_Param Params_Table[] = {
    {"kP_Acc_AHRS", KP_ACC_AHRS_ADDR, VAR_8BITS, &JCF_Param.kP_Acc_AHRS, 0, 255},
    {"kI_Acc_AHRS", KI_ACC_AHRS_ADDR, VAR_8BITS, &JCF_Param.kI_Acc_AHRS, 0, 255},
    {"kP_Mag_AHRS", KP_MAG_AHRS_ADDR, VAR_8BITS, &JCF_Param.kP_Mag_AHRS, 0, 255},
    {"kI_Mag_AHRS", KI_MAG_AHRS_ADDR, VAR_8BITS, &JCF_Param.kI_Mag_AHRS, 0, 255},
    /*{"AutoLaunch_AHRS_BankAngle", AL_AHRS_BA_ADDR, VAR_8BITS, &JCF_Param.AutoLaunch_AHRS_BankAngle, 0, 255},
    {"AutoLaunch_IMU_BankAngle", AL_IMU_BA_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_IMU_BankAngle, 0, 1000},
    {"AutoLaunch_IMU_Swing", AL_IMU_SWING_ADDR, VAR_8BITS, &JCF_Param.AutoLaunch_IMU_Swing, 0, 255},
    {"AutoLaunch_Trigger_Motor_Delay", AL_TRIGGER_MOTOR_DELAY_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_Trigger_Motor_Delay, 0, 10000},
    {"AutoLaunch_Elevator", AL_ELEVATOR_ADDR, VAR_8BITS, &JCF_Param.AutoLaunch_Elevator, 0, 255},
    {"AutoLaunch_SpinUp", AL_SPINUP_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_SpinUp, 0, 2000},
    {"AutoLaunch_SpinUp_Time", AL_SPINUP_TIME_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_SpinUp_Time, 0, 5000},
    {"AutoLaunch_MaxThrottle", AL_MAX_THROTTLE_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_MaxThrottle, 1000, 2200},
    {"AutoLaunch_Exit", AL_EXIT_ADDR, VAR_16BITS, &JCF_Param.AutoLaunch_Exit, 0, 30000},
    {"AutoLaunch_Altitude", AL_ALTITUDE_ADDR, VAR_8BITS, &JCF_Param.AutoLaunch_Altitude, 0, 255},
    {"AutoLaunch_Batt_Voltage_Factor", BATT_VOLTAGE_FACTOR_ADDR, VAR_32BITS, &JCF_Param.Batt_Voltage_Factor, 0, 400000},
    {"AutoLaunch_Batt_Amps_Volt", BATT_AMPS_VOLT_ADDR, VAR_16BITS, &JCF_Param.Amps_Per_Volt, 0, 1000},
    {"AutoLaunch_Batt_Amps_OffSet", BATT_AMPS_OFFSET_ADDR, VAR_16BITS, &JCF_Param.Amps_OffSet, 0, 1000},
    {"CrashCheck_BankAngle", CC_BANKANGLE_ADDR, VAR_8BITS, &JCF_Param.CrashCheck_BankAngle, 0, 255},
    {"CrashCheck_Time", CC_TIME_ADDR, VAR_8BITS, &JCF_Param.CrashCheck_Time, 0, 255},
    {"GimbalMinValue", GIMBAL_MIN_ADDR, VAR_16BITS, &JCF_Param.GimbalMinValue, 800, 2200},
    {"GimbalMiddleValue", GIMBAL_MID_ADDR, VAR_16BITS, &JCF_Param.GimbalMiddleValue, 800, 2200},
    {"GimbalMaxValue", GIMBAL_MAX_ADDR, VAR_16BITS, &JCF_Param.GimbalMaxValue, 800, 2200},
    {"Land_CheckAcc", LAND_CHECKACC_ADDR, VAR_8BITS, &JCF_Param.Land_Check_Acc, 0, 255},
    {"Land_LPF", LAND_LPF_ADDR, VAR_8BITS, &JCF_Param.Land_LPF, 0, 255},
    {"ThrottleFactor", THROTTLE_FACTOR_ADDR, VAR_16BITS, &JCF_Param.Throttle_Factor, 0, 255},
    {"AutoDisarm", AUTODISARM_ADDR, VAR_8BITS, &JCF_Param.AutoDisarm_Time, 0, 255},
    {"AutoDisarm_Throttle_Min", AUTODISARM_THR_MIN_ADDR, VAR_16BITS, &JCF_Param.AutoDisarm_Throttle_Min, 800, 1500},
    {"AutoDisarm_YPR_Min", AUTODISARM_YPR_MIN_ADDR, VAR_16BITS, &JCF_Param.AutoDisarm_YPR_Min, 800, 1500},
    {"AutoDisarm_YPR_Max", AUTODISARM_YPR_MAX_ADDR, VAR_16BITS, &JCF_Param.AutoDisarm_YPR_Max, 800, 2200},
    {"AirPlane_Wheels", WHEELS_ADDR, VAR_8BITS, &JCF_Param.AirPlane_Wheels, 0, 255},
    {"GPS_Baud_Rate", GPS_BAUDRATE_ADDR, VAR_8BITS, &JCF_Param.GPS_Baud_Rate, 0, 4},
    {"Navigation_Vel", NAV_VEL_ADDR, VAR_16BITS, &JCF_Param.Navigation_Vel, 0, 400},
    {"GPS_WP_Radius", WP_RADIUS_ADDR, VAR_8BITS, &JCF_Param.GPS_WP_Radius, 0, 255},
    {"GPS_RTH_Land", RTH_LAND_ADDR, VAR_8BITS, &JCF_Param.GPS_RTH_Land, 0, 255},
    {"GPS_TiltCompensation", GPS_TILT_COMP_ADDR, VAR_8BITS, &JCF_Param.GPS_TiltCompensation, 0, 100},
    {"AirSpeed_Samples", AIRSPEED_SAMPLES_ADDR, VAR_8BITS, &JCF_Param.AirSpeed_Samples, 0, 255},
    {"AirSpeed_Factor", AIRSPEED_FACTOR_ADDR, VAR_16BITS, &JCF_Param.AirSpeed_Factor, 0, 5000},*/
};

#define TABLE_COUNT (sizeof(Params_Table) / sizeof(Requesited_Values_Of_Param))

void ParamClass::Initialization()
{

  //PASSA OS VALORES DA EEPROM PARA OS PARAMETROS,ISSO DEVE FICAR SEMPRE ATIVO NA VERSÃO FINAL,ATÉ MESMO EM TESTES DE DESENVOLVIMENTO
  //PARAM.LoadSketch();

  //PARAM.Set_And_Save("dagasdg = 54"); //TESTE DE PARAMETRO INEXISTENTE

  //PARAM.Set_And_Save("kP_Acc_AHRS = 12"); //TESTE DE PARAMETRO EXISTENTE

  //PARAM.Set_And_Save("kP_Acc_AHRS = 300"); //TESTE DE LIMITE MAXIMO DO PARAMETRO

  //PARAM.Set_And_Save("kP_Acc_AHRS = -10"); //TESTE DE LIMITE MINIMO DO PARAMETRO

  //PARAM.Set_And_Save("relatorio"); //TESTE DE RELATORIO ATUAL DOS PARAMETROS

  //DEBUG("%d", JCF_Param.kP_Acc_AHRS); //VERFICAÇÃO DO FUNCIONAMENTO DO LoadSketch()

#ifdef OPERATOR_CHECK_EEPROM
  Operator_Check_Values_In_Address(SIZE_OF_EEPROM);
#endif

#ifdef ERASE_ALL_EEPROM
  EraseEEPROM(INITIAL_ADDRESS_EEPROM_TO_CLEAR, FINAL_ADDRESS_EEPROM_TO_CLEAR, SIZE_OF_EEPROM);
#endif
}

void ParamClass::LoadSketch()
{
  const Requesited_Values_Of_Param *ParamValue;
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

static void ParamSetValue(const Requesited_Values_Of_Param *VariablePointer, const int32_t New_Value)
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

static void ParamPrintValue(const Requesited_Values_Of_Param *VariablePointer)
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
          ParamSetValue(ParamValue, New_Value);
          PRINTF.SendToConsole(ProgramMemoryString("%s setado para "), Params_Table[Table_Counter].Param_Name);
          ParamPrintValue(ParamValue);
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
      PRINTF.SendToConsole(ProgramMemoryString("%s = "), Params_Table[Table_Counter].Param_Name);
      ParamPrintValue(ParamValue);
    }
    DEBUG("\r");
  }
  else if (strncasecmp(ParamCommandLine, "reiniciar", 9) == 0)
  {
  }
  else if (strncasecmp(ParamCommandLine, "formatar", 8) == 0)
  {
  }
  else
  {
    LOG_PARAM_ERROR("Comando invalido!");
  }
}

#include "FastSerial/FASTSERIAL.h"

static char SerialBuffer[48];
static uint32_t SerialBufferIndex = 0;

void ParamSerialProcess()
{
  while (FASTSERIAL.Available(UART_NUMB_0))
  {
    uint8_t SerialReadCommand = FASTSERIAL.Read(UART_NUMB_0);

    if (SerialReadCommand == 127)
    {
      if (SerialBufferIndex)
      {
        SerialBuffer[--SerialBufferIndex] = 0;
      }
    }

    if (SerialBufferIndex < sizeof(SerialBuffer) && SerialReadCommand >= 32 && SerialReadCommand <= 126)
    {
      if (!SerialBufferIndex && SerialReadCommand == 32)
      {
        continue;
      }
      SerialBuffer[SerialBufferIndex++] = SerialReadCommand;
    }

    if (SerialBufferIndex)
    {
      SerialBuffer[SerialBufferIndex] = 0;
      PARAM.Set_And_Save(SerialBuffer);
    }
  }
  SerialBufferIndex = 0;
}