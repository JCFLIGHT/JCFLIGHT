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

#ifndef ENUM_H_
#define ENUM_H_
enum RadioControl_Enum
{
    RESET_PPM = 0,
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8
};

enum Frames_Type_Enum
{
    QUAD_X = 0,
    HEXA_X,
    HEXA_I,
    AIRPLANE,
    FIXED_WING,
    PLANE_VTAIL,
    ZMR250,
    TBS

};

enum PID_Params_Enum
{
    PIDROLL = 0,
    PIDPITCH,
    PIDYAW,
    PIDALTITUDE,
    PIDGPSPOSITION,
    PIDGPSPOSITIONRATE,
    PIDGPSNAVIGATIONRATE,
    PIDAUTOLEVEL,
    PIDYAWVELOCITY,
    SIZE_OF_THIS_PID_PARAMS
};

enum FlightModes_Enum
{
    ARM_DISARM_MODE = 0,
    STABILIZE_MODE,
    ALTITUDE_HOLD_MODE,
    HEADING_HOLD_MODE,
    IOC_MODE,
    RTH_MODE,
    GPS_HOLD_MODE,
    ATACK_MODE,
    LAND_MODE,
    FLIP_MODE,
    WAYPOINT_MODE,
    SIZEOFTHIS
};

enum AuxiliarChannels_EEPROM_Enum
{
    AUXONELOW = 1,
    AUXONEMIDDLE,
    AUXONEHIGH,
    AUXTWOLOW,
    AUXTWOMIDDLE,
    AUXTWOHIGH,
    AUXTHREELOW,
    AUXTHREEMIDDLE,
    AUXTHREEHIGH,
    AUXFOURLOW,
    AUXFOURMIDDLE,
    AUXFOURHIGH,
    AUXFIVELOW,
    AUXFIVEMIDDLE,
    AUXFIVEHIGH,
    AUXSIXLOW,
    AUXSIXMIDDLE,
    AUXSIXHIGH,
    AUXSEVENLOW,
    AUXSEVENMIDDLE,
    AUXSEVENHIGH,
    AUXEIGHTLOW,
    AUXEIGHTMIDDLE,
    AUXEIGHTHIGH
};

enum PWM_Output_Enum
{
    MOTOR4 = 0,
    MOTOR3,
    MOTOR2,
    MOTOR1,
    MOTOR6,
    MOTOR5,
    GIMBAL,
    PARACHUTESERVO
};

enum Servos_Enum
{
    SERVO1 = 0,
    SERVO2,
    SERVO3,
    SERVO4
};

enum GPS_Modes_Enum
{
    GPS_MODE_NONE = 0,
    GPS_MODE_HOLD,
    GPS_MODE_RTH,
    WAYPOINT
};

enum GPS_Flight_Modes_Enum
{
    Do_None = 0,
    Do_Start_RTH,
    Do_RTH_Enroute,
    Do_PositionHold,
    Do_Land_Init,
    Do_LandInProgress,
    Do_Landed,
    Do_Land_Settle,
    Do_Land_Descent,
    Do_Land_Detected
};

enum AuxiliarChannels
{
    NONE = 0,
    RCAUX1,
    RCAUX2,
    RCAUX3,
    RCAUX4,
    RCAUX5,
    RCAUX6,
    RCAUX7,
    RCAUX8
};

enum RGB_Led_Enum
{
    RED = 0,
    GREEN,
    BLUE
};

enum RGB_Led_State_Enum
{
    GPSLED = 0,
    ACCLED,
    MAGLED,
    CONFIGFLIGHT,
    CALIBRATIONESC,
    CALIBRATIONESCFINISH,
    PREARMINIT,
    PREARMSUCESS,
    PREARMFAIL,
    OFFLEDS
};

enum ADC_Pins_Enum
{
    //16 PINOS ANALOGICOS MAXIMO,DE ACORDO COM O MEGA 2560
    ADC_NUM_0 = 0, //OFF(V.BATTERY INPUT)
    ADC_NUM_1,     //OFF(CURRENT INPUT)
    ADC_NUM_2,     //OFF(PITOT TUBE)
    ADC_NUM_3,
    ADC_NUM_4,
    ADC_NUM_5,
    ADC_NUM_6,
    ADC_NUM_7,
    ADC_NUM_8,
    ADC_NUM_9,
    ADC_NUM_10,
    ADC_NUM_11,
    ADC_NUM_12,
    ADC_NUM_13,
    ADC_NUM_14, //OFF(SAFETY BUTTON)
    ADC_NUM_15  //OFF (PPM INPUT)
};

enum Compass_Type_Enum
{
    COMPASS_AK8975 = 0,
    COMPASS_HMC5843,
    COMPASS_HMC5883
};

enum Compass_Board_Type_Enum
{
    GPS_ONBOARD_COMPASS = 0,
    EXTERNAL_COMPASS
};

enum Compass_Rotation_Enum
{
    NONE_ROTATION = 0,
    COMPASS_ROTATION_YAW_45_DEGREES,
    COMPASS_ROTATION_YAW_315_DEGREES,
    COMPASS_ROTATION_ROLL_180_YAW_45_DEGREES,
    COMPASS_ROTATION_PITCH_180_DEGREES
};

enum UART_Type_Enum
{
    UART_NUMB_0 = 0,
    UART_NUMB_1,
    UART_NUMB_2,
    UART_NUMB_3
};

enum Barometer_Type_Enum
{
    BAROMETER_MS5611 = 0,
    BAROMETER_BMP280
};

enum Battery_Type_Enum
{
    NONE_BATTERY = 0,
    BATTERY_3S,
    BATTERY_4S,
    BATTERY_6S
};

enum Flip_Stages_Enum
{
    STAGEONE = 0,
    STAGETWO,
    STAGETHREE,
    STAGERECOVER,
    STAGEABANDON,
    STAGEWAITING
};

enum Speed_Type_Enum
{
    VERY_LOW_SPEED = 0,
    LOW_SPEED,
    MEDIUM_SPEED,
    HIGH_SPEED,
    VERY_HIGH_SPEED
};

typedef enum
{
    BEEPER_CALIBRATION_DONE = 0,
    BEEPER_DISARMING,
    BEEPER_BAT_CRIT_LOW,
    BEEPER_ACTION_SUCCESS,
    BEEPER_ACTION_FAIL,
    BEEPER_ARM,
    BEEPER_ALGORITHM_INIT,
    BEEPER_AUTOLAUNCH,
    BEEPER_LAUNCHED,
    BEEPER_FMU_INIT,
    BEEPER_FMU_SAFE_TO_ARM
} Beeper_Mode;

enum FilterTypes
{
    LPF = 0,
    HPF,
    NOTCH
};

enum I2C_Supported_Devices_Enum
{
    //ENDEREÇOS EM ORDEM NÚMERICA,DO MENOR PARA O MAIOR
    ADDRESS_COMPASS_AK8975 = 0x0C,
    ADDRESS_COMPASS_QMC5883 = 0x0D,
    ADDRESS_COMPASS_HMC5843 = 0x1E,
    ADDRESS_COMPASS_HMC5883 = 0x1E,
    ADDRESS_IMU_MPU6050 = 0X68,
    ADDRESS_BAROMETER_BMP280 = 0x76,
    ADDRESS_BAROMETER_MS5611 = 0x77,
    SizeOfThis
};

enum Parachute_RCAuxiliaryChannels_Enum
{
    PARACHUTEAUXONELOW = 2,
    PARACHUTEAUXONEMIDDLE,
    PARACHUTEAUXONEHIGH,
    PARACHUTEAUXTWOLOW,
    PARACHUTEAUXTWOMIDDLE,
    PARACHUTEAUXTWOHIGH,
    PARACHUTEAUXTHREELOW,
    PARACHUTEAUXTHREEMIDDLE,
    PARACHUTEAUXTHREEHIGH,
    PARACHUTEAUXFOURLOW,
    PARACHUTEAUXFOURMIDDLE,
    PARACHUTEAUXFOURHIGH,
    PARACHUTEAUXFIVELOW,
    PARACHUTEAUXFIVEMIDDLE,
    PARACHUTEAUXFIVEHIGH,
    PARACHUTEAUXSIXLOW,
    PARACHUTEAUXSIXMIDDLE,
    PARACHUTEAUXSIXHIGH,
    PARACHUTEAUXSEVENLOW,
    PARACHUTEAUXSEVENMIDDLE,
    PARACHUTEAUXSEVENHIGH,
    PARACHUTEAUXEIGHTLOW,
    PARACHUTEAUXEIGHTMIDDLE,
    PARACHUTEAUXEIGHTHIGH
};

enum VarType_Enum
{
    VAR_8BITS,
    VAR_16BITS,
    VAR_32BITS
};

enum class Led_Pattern : uint16_t
{
    FMU_INIT_ARM = 0x0003,
    FMU_REFUSE_TO_ARM = 0x5555,
    FMU_SAFE_TO_ARM = 0xffff,
};

enum Function_Names
{
    SLOW_LOOP = 0,
    MEDIUM_LOOP,
    FAST_MEDIUM_LOOP,
    FAST_LOOP,
    TOTAL_LOOP,
    SIZE_LOOPS
};

enum WayPoint_States_Enum
{
    WP_MISSION_INIT = 0,
    GET_ALTITUDE,
    GET_ALTITUDE_TAKEOFF,
    WP_START_MISSION,
    WP_EN_ROUTE
};

enum WayPoint_FlightModes_Enum
{
    WP_ADVANCE = 1,
    WP_TIMED,
    WP_LAND,
    WP_RTH,
    WP_TAKEOFF
};

enum AirPlane_Wheels_Enum
{
    WITHOUT_WHEELS = 0,
    WITH_WHEELS
};

enum AirSpeed_Type_Enum
{
    NONE_AIRSPEED = 0,
    ANALOG_AIRSPEED,
    I2C_AIRSPEED
};

enum Ublox_Protocol_Byte_Enum
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

enum Ublox_Navigation_Fix_Type_Enum
{
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};

enum Ublox_Navigation_Status_Enum
{
    NAV_STATUS_FIX_VALID = 1
};

enum Buzzer_Modes_Enum
{
    NORMAL_OPERATION_MODE = -10,
    ESC_CALIBRATION_MODE = 1,
    ESC_FINISH_CALIBRATION_MODE = 2
};

enum Dispositives_Passives_Enum
{
    OFF_ALL_DISP = 0,
    BUZZER_AND_SWITCH, //NÃO USADO,MAS NÃO PODE SER REMOVIDO E NEM MOVIDO
    ONLY_BUZZER,
    ONLY_SWITCH
};

enum TaskPriority_Enum
{
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 6
};

typedef enum
{
    TASK_SLOW_LOOP = 0,
    TASK_MEDIUM_LOOP,
    TASK_FAST_MEDIUM_LOOP,
    TASK_FAST_LOOP,
    TASK_SUPER_FAST_LOOP,
    TASK_INTEGRAL_LOOP,
    TASK_IMU_CALIBRATION,
    //TASK COUNT SEMPRE EM ÚLTIMO LUGAR
    TASK_COUNT
} Tasks_ID_Enum;
#endif