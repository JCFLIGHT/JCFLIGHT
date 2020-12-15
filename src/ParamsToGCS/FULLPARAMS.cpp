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

//A LISTA COMPLETA DE PARAMETROS IRÁ FUNCIONAR APENAS NA PLATAFORMA STM32
//PARA O ATMEGA2560 FICARA APENAS OS PARAMETROS MAIS IMPORTANTES
#define MEGA2560

//#define OPERATOR_CHECK_EEPROM

typedef struct
{
#ifndef MEGA2560
  uint8_t Param_kP_Acc_AHRS;
  uint8_t Param_kI_Acc_AHRS;
  uint8_t Param_kP_Mag_AHRS;
  uint8_t Param_kI_Mag_AHRS;
  uint16_t Param_Servo_Pulse_Min;
  uint16_t Param_Servo_Pulse_Middle;
  uint16_t Param_Servo_Pulse_Max;
  uint8_t Param_AutoLaunch_AHRS_BankAngle;
  uint16_t Param_AutoLaunch_IMU_BankAngle;
  uint8_t Param_AutoLaunch_IMU_Swing;
  uint16_t Param_AutoLaunch_Trigger_Motor_Delay;
  uint8_t Param_AutoLaunch_Elevator;
  uint16_t Param_AutoLaunch_SpinUp;
  uint16_t Param_AutoLaunch_SpinUp_Time;
  uint16_t Param_AutoLaunch_MaxThrottle;
  uint16_t Param_AutoLaunch_Exit;
  uint8_t Param_AutoLaunch_Altitude;
  uint32_t Param_Batt_Voltage_Factor;
  uint16_t Param_Amps_Per_Volt;
  uint16_t Param_Amps_OffSet;
  uint8_t Param_CrashCheck_BankAngle;
  uint8_t Param_CrashCheck_Time;
#endif
  uint16_t Param_FailSafeValue;
#ifndef MEGA2560
  uint16_t Param_GimbalMinValue;
  uint16_t Param_GimbalMiddleValue;
  uint16_t Param_GimbalMaxValue;
  uint8_t Param_Land_Check_Acc;
  uint8_t Param_Land_LPF;
#endif
  uint8_t Param_RC_Rate;
  uint8_t Param_RC_Expo;
  uint8_t Param_Roll_Pitch_Rate;
  uint8_t Param_Yaw_Rate;
  uint8_t Param_Throttle_Middle;
  uint8_t Param_Throttle_Expo;
#ifndef MEGA2560
  uint8_t Param_AutoDisarm_Time;
  uint16_t Param_AutoDisarm_Throttle_Min;
  uint16_t Param_AutoDisarm_YPR_Min;
  uint16_t Param_AutoDisarm_YPR_Max;
  uint8_t Param_AHRS_Nearness;
  uint8_t Param_AirPlane_Wheels;
  uint8_t Param_GPS_Baud_Rate;
  uint16_t Param_Navigation_Vel;
  uint8_t Param_GPS_WP_Radius;
  uint8_t Param_GPS_RTH_Land;
  uint8_t Param_GPS_TiltCompensation;
  uint8_t Param_AirSpeed_Samples;
  uint16_t Param_AirSpeed_Factor;
  int16_t Param_Acc_Adjust_Roll;
  int16_t Param_Acc_Adjust_Pitch;
  int16_t Param_Acc_Adjust_Yaw;
#endif
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
#ifndef MEGA2560
    {"kP_Acc_AHRS", KP_ACC_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kP_Acc_AHRS, 0, 255},
    {"kI_Acc_AHRS", KI_ACC_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kI_Acc_AHRS, 0, 255},
    {"kP_Mag_AHRS", KP_MAG_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kP_Mag_AHRS, 0, 255},
    {"kI_Mag_AHRS", KI_MAG_AHRS_ADDR, VAR_8BITS, &FullParamsList.Param_kI_Mag_AHRS, 0, 255},
    {"Servo_Pulse_Min", SERVO_PULSE_MIN_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Min, 300, 2500},
    {"Pulse_Middle", SERVO_PULSE_MIDDLE_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Middle, 400, 2500},
    {"Servo_Pulse_Max", SERVO_PULSE_MAX_ADDR, VAR_16BITS, &FullParamsList.Param_Servo_Pulse_Max, 1000, 2600},
    {"AutoLaunch_AHRS_BankAngle", AL_AHRS_BA_ADDR, VAR_8BITS, &FullParamsList.Param_AutoLaunch_AHRS_BankAngle, 0, 255},
    {"AutoLaunch_IMU_BankAngle", AL_IMU_BA_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_IMU_BankAngle, 0, 1000},
    {"AutoLaunch_IMU_Swing", AL_IMU_SWING_ADDR, VAR_8BITS, &FullParamsList.Param_AutoLaunch_IMU_Swing, 0, 255},
    {"AutoLaunch_Trigger_Motor_Delay", AL_TRIGGER_MOTOR_DELAY_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_Trigger_Motor_Delay, 0, 10000},
    {"AutoLaunch_Elevator", AL_ELEVATOR_ADDR, VAR_8BITS, &FullParamsList.Param_AutoLaunch_Elevator, 0, 255},
    {"AutoLaunch_SpinUp", AL_SPINUP_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_SpinUp, 0, 2000},
    {"AutoLaunch_SpinUp_Time", AL_SPINUP_TIME_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_SpinUp_Time, 0, 5000},
    {"AutoLaunch_MaxThrottle", AL_MAX_THROTTLE_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_MaxThrottle, 1000, 2200},
    {"AutoLaunch_Exit", AL_EXIT_ADDR, VAR_16BITS, &FullParamsList.Param_AutoLaunch_Exit, 0, 30000},
    {"AutoLaunch_Altitude", AL_ALTITUDE_ADDR, VAR_8BITS, &FullParamsList.Param_AutoLaunch_Altitude, 0, 255},
    {"AutoLaunch_Batt_Voltage_Factor", BATT_VOLTAGE_FACTOR_ADDR, VAR_32BITS, &FullParamsList.Param_Batt_Voltage_Factor, 0, 400000},
    {"AutoLaunch_Batt_Amps_Volt", BATT_AMPS_VOLT_ADDR, VAR_16BITS, &FullParamsList.Param_Amps_Per_Volt, 0, 1000},
    {"AutoLaunch_Batt_Amps_OffSet", BATT_AMPS_OFFSET_ADDR, VAR_16BITS, &FullParamsList.Param_Amps_OffSet, 0, 1000},
    {"CrashCheck_BankAngle", CC_BANKANGLE_ADDR, VAR_8BITS, &FullParamsList.Param_CrashCheck_BankAngle, 0, 255},
    {"CrashCheck_Time", CC_TIME_ADDR, VAR_8BITS, &FullParamsList.Param_CrashCheck_Time, 0, 255},
#endif
    {"FailSafeValue", FAILSAFE_ADDR, VAR_16BITS, &FullParamsList.Param_FailSafeValue, 0, 2000},
#ifndef MEGA2560
    {"GimbalMinValue", GIMBAL_MIN_ADDR, VAR_16BITS, &FullParamsList.Param_GimbalMinValue, 800, 2200},
    {"GimbalMiddleValue", GIMBAL_MID_ADDR, VAR_16BITS, &FullParamsList.Param_GimbalMiddleValue, 800, 2200},
    {"GimbalMaxValue", GIMBAL_MAX_ADDR, VAR_16BITS, &FullParamsList.Param_GimbalMaxValue, 800, 2200},
    {"Land_CheckAcc", LAND_CHECKACC_ADDR, VAR_8BITS, &FullParamsList.Param_Land_Check_Acc, 0, 255},
    {"Land_LPF", LAND_LPF_ADDR, VAR_8BITS, &FullParamsList.Param_Land_LPF, 0, 255},
#endif
    {"RC_Rate", RC_RATE_ADDR, VAR_8BITS, &FullParamsList.Param_RC_Rate, 0, 255},
    {"RC_Expo", RC_EXPO_ADDR, VAR_8BITS, &FullParamsList.Param_RC_Expo, 0, 255},
    {"Roll_Pitch_Rate", ROLL_PITCH_RATE_ADDR, VAR_8BITS, &FullParamsList.Param_Roll_Pitch_Rate, 0, 255},
    {"Yaw_Rate", YAW_RATE_ADDR, VAR_8BITS, &FullParamsList.Param_Yaw_Rate, 0, 255},
    {"ThrottleMiddle", THROTTLE_MIDDLE_ADDR, VAR_8BITS, &FullParamsList.Param_Throttle_Middle, 0, 255},
    {"ThrottleExpo", THROTTLE_EXPO_ADDR, VAR_8BITS, &FullParamsList.Param_Throttle_Expo, 0, 255},
#ifndef MEGA2560
    {"AutoDisarm", AUTODISARM_ADDR, VAR_8BITS, &FullParamsList.Param_AutoDisarm_Time, 0, 255},
    {"Throttle_Min", THR_MIN_ADDR, VAR_16BITS, &FullParamsList.Param_AutoDisarm_Throttle_Min, 800, 1500},
    {"YPR_Min", YPR_MIN_ADDR, VAR_16BITS, &FullParamsList.Param_AutoDisarm_YPR_Min, 800, 1500},
    {"YPR_Max", YPR_MAX_ADDR, VAR_16BITS, &FullParamsList.Param_AutoDisarm_YPR_Max, 800, 2200},
    {"AHRS_Nearness", NEARNESS_ADDR, VAR_8BITS, &FullParamsList.Param_AHRS_Nearness, 0, 255},
    {"AirPlane_Wheels", WHEELS_ADDR, VAR_8BITS, &FullParamsList.Param_AirPlane_Wheels, 0, 255},
    {"GPS_Baud_Rate", GPS_BAUDRATE_ADDR, VAR_8BITS, &FullParamsList.Param_GPS_Baud_Rate, 0, 4},
    {"Navigation_Vel", NAV_VEL_ADDR, VAR_16BITS, &FullParamsList.Param_Navigation_Vel, 0, 400},
    {"GPS_WP_Radius", WP_RADIUS_ADDR, VAR_8BITS, &FullParamsList.Param_GPS_WP_Radius, 0, 255},
    {"GPS_RTH_Land", RTH_LAND_ADDR, VAR_8BITS, &FullParamsList.Param_GPS_RTH_Land, 0, 255},
    {"GPS_TiltCompensation", TILT_COMP_ADDR, VAR_8BITS, &FullParamsList.Param_GPS_TiltCompensation, 0, 100},
    {"AirSpeed_Samples", AIRSPEED_SAMPLES_ADDR, VAR_8BITS, &FullParamsList.Param_AirSpeed_Samples, 0, 255},
    {"AirSpeed_Factor", AIRSPEED_FACTOR_ADDR, VAR_16BITS, &FullParamsList.Param_AirSpeed_Factor, 0, 5000},
    {"Adjust_Roll", ROLL_ADJ_ADDR, VAR_16BITS, &FullParamsList.Param_Acc_Adjust_Roll, -800, 800},
    {"Adjust_Pitch", PITCH_ADJ_ADDR, VAR_16BITS, &FullParamsList.Param_Acc_Adjust_Pitch, -800, 800},
    {"Adjust_Yaw", YAW_ADJ_ADDR, VAR_16BITS, &FullParamsList.Param_Acc_Adjust_Yaw, -800, 800},
#endif
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