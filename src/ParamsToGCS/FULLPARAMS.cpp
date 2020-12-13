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

#include "FULLPARAMS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "StorageManager/EEPROMCHECK.h"
#include "StringSupport/STRINGSUPPORT.h"

//#define OPERATOR_CHECK_EEPROM

typedef struct
{
  uint8_t Param_kP_Acc_AHRS;
  uint8_t Param_kI_Acc_AHRS;
  uint8_t Param_kP_Mag_AHRS;
  uint8_t Param_kI_Mag_AHRS;
  uint8_t Param_Acc_Ignore_Rate;
  uint8_t Param_Acc_Ignore_Slope;
  uint16_t Param_Servo_Pulse_Min;
  uint16_t Param_Servo_Pulse_Middle;
  uint16_t Param_Servo_Pulse_Max;
  uint16_t Param_Servo_LPF_CutOff;
  uint8_t Param_AutoLaunch_AHRS_BankAngle;
  uint16_t Param_AutoLaunch_IMU_BankAngle;
  uint16_t Param_AutoLaunch_Motor_SpinUp_Time;
  uint16_t Param_AutoLaunch_Motor_MaxThrottle;
} Struct_FullParamsList;

Struct_FullParamsList FullParamsList;

typedef enum
{
  VAR_8BITS,
  VAR_16BITS,
  VAR_32BITS
} VarType;

typedef struct
{
  const char *Param_Name;
  const uint16_t Address;
  const uint8_t VariableType;
  void *Ptr;
  const int32_t Value_Min;
  const int32_t Value_Max;
} Requesited_Values_Of_Param;

const Requesited_Values_Of_Param Params_Table[] = {
    {"kP_Acc_AHRS", KP_ACC_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kP_Acc_AHRS, 0, 255},
    {"kI_Acc_AHRS", KI_ACC_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kI_Acc_AHRS, 0, 255},
    {"kP_Mag_AHRS", KP_MAG_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kP_Mag_AHRS, 0, 255},
    {"kI_Mag_AHRS", KI_MAG_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kI_Mag_AHRS, 0, 255},
    {"Ignore_Rate", IGNORE_RATE_ADDR, VAR_8BITS, &FullParamsList.Param_Acc_Ignore_Rate, 0, 255},
    {"Ignore_Slope", IGNORE_SLOPE_ADDR, VAR_8BITS, &FullParamsList.Param_Acc_Ignore_Slope, 0, 255},
    {"Servo_Pulse_Min", SERVO_PULSE_MIN_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Min, 300, 2500},
    {"Pulse_Middle", SERVO_PULSE_MIDDLE_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Middle, 400, 2500},
    {"Servo_Pulse_Max", SERVO_PULSE_MAX_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Max, 1000, 2600},
    {"Servo_LPF_CutOff", SERVO_LPF_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_LPF_CutOff, 0, 32000},
    {"AutoLaunch_AHRS_BankAngle", AL_AHRS_BA_ADDR, VAR_8BITS, &FullParamsList.Param_AutoLaunch_AHRS_BankAngle, 0, 255},
    {"AutoLaunch_IMU_BankAngle", AL_IMU_BA_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_IMU_BankAngle, 0, 1000},
    {"AutoLaunch_Motor_SpinUp_Time", AL_MOTOR_SPINUP_TIME_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_Motor_SpinUp_Time, 0, 10000},
    {"AutoLaunch_Motor_MaxThrottle", AL_MOTOR_MAX_THROTTLE_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_Motor_MaxThrottle, 1000, 2200},
};

#define TABLE_COUNT (sizeof(Params_Table) / sizeof(Requesited_Values_Of_Param))

void FullParamsListInitialization()
{
  //SetNewValue("kP_Acc_AHRS", 242);
#ifdef OPERATOR_CHECK_EEPROM
  Operator_Check_Values_In_Address();
#endif
}

void SetNewValue(const char *ParamName, int32_t NewValue)
{
  for (uint32_t i = 0; i < TABLE_COUNT; i++)
  {
    if (StringCompare(ParamName, Params_Table[i].Param_Name, StringLength(Params_Table[i].Param_Name)) == 0)
    {
      if (NewValue >= Params_Table[i].Value_Min && NewValue <= Params_Table[i].Value_Max)
      {
        if (Params_Table[i].VariableType == VAR_8BITS && NewValue != STORAGEMANAGER.Read_8Bits(Params_Table[i].Address))
          STORAGEMANAGER.Write_8Bits(Params_Table[i].Address, NewValue);
        else if (Params_Table[i].VariableType == VAR_16BITS && NewValue != STORAGEMANAGER.Read_16Bits(Params_Table[i].Address))
          STORAGEMANAGER.Write_16Bits(Params_Table[i].Address, NewValue);
        else if (Params_Table[i].VariableType == VAR_32BITS && NewValue != STORAGEMANAGER.Read_32Bits(Params_Table[i].Address))
          STORAGEMANAGER.Write_32Bits(Params_Table[i].Address, NewValue);
      }
      else
      {
        //VALOR SETADO FORA DO RANGE MIN E MAX
      }
      return;
    }
  }
}