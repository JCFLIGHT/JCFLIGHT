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
    PID_ROLL = 0,
    PID_PITCH,
    PID_YAW,
    PID_ALTITUDE,
    PID_GPS_POSITION,
    PID_GPS_POSITION_RATE,
    PID_GPS_NAVIGATION_RATE,
    PI_AUTO_LEVEL,
    P_YAW_RATE,
    P_YAW_RATE_LIMIT,
    ROLL_BANK_MAX,
    PITCH_BANK_MIN,
    PITCH_BANK_MAX,
    ATTACK_BANK_MAX,
    GPS_BANK_MAX,
    PID_UPDATED,
    SIZE_OF_PID_PARAMS
};

enum FlightModes_Enum
{
    //MODOS DE VOO PARA TODOS OS FRAMES
    PRIMARY_ARM_DISARM = 0,
    SECONDARY_ARM_DISARM,
    STABILIZE_MODE,
    HEADING_HOLD_MODE,
    RTH_MODE,

    //MODOS DE VOOS PARA MULTIROTORES
    ALTITUDE_HOLD_MODE,
    SIMPLE_MODE,
    POS_HOLD_MODE,
    ATTACK_MODE,
    LAND_MODE,
    FLIP_MODE,
    WAYPOINT_MODE,

    //MODOS DE VOO PARA AEROS
    AUTO_THROTTLE_MODE,
    MANUAL_MODE,
    CIRCLE_MODE,
    LAUNCH_MODE,
    CRUISE_MODE,
    TURN_MODE,
    CLIMBOUT_MODE,

    //TAMANHO TOTAL PARA O ARRAY
    SIZE_OF_FLIGHT_MODES
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
    GPS_MODE_RTH
};

enum GPS_Flight_Modes_Enum
{
    DO_NONE = 0,
    DO_START_RTH,
    DO_RTH_ENROUTE,
    DO_POSITION_HOLD,
    DO_LAND_INIT,
    DO_LAND_IN_PROGRESS,
    DO_LANDED,
    DO_LAND_SETTLE,
    DO_LAND_DESCENT,
    DO_LAND_DETECTED
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
    CALL_LED_GPS = 0,
    CALL_LED_ACC_CALIBRATION,
    CALL_LED_MAG_CALIBRATION,
    CALL_LED_CONFIG_FLIGHT,
    CALL_LED_CALIBRATION_ESC,
    CALL_LED_PRE_ARM_INIT,
    CALL_LED_PRE_ARM_SUCESS,
    CALL_LED_PRE_ARM_FAIL
};

enum ADC_Pins_Enum
{
    //16 PINOS ANALOGICOS MAXIMO,DE ACORDO COM O MEGA 2560
    ADC_NUM_0 = 0, //OFF(LEITURA DA TENSÃO)
    ADC_NUM_1,     //OFF(LEITURA DA CORRENTE)
    ADC_NUM_2,     //OFF(LEITURA DO TUBO PITOT)
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
    ADC_NUM_14, //OFF(CAPTURA DO SAFETY BUTTON)
    ADC_NUM_15  //OFF(CAPTURA DO PPM)
};

enum Compass_Type_Enum
{
    COMPASS_AK8975 = 0,
    COMPASS_HMC5843,
    COMPASS_HMC5883,
    COMPASS_QMC5883,
    COMPASS_DJI_NAZA
};

enum Compass_Rotation_Enum
{
    ROTATION_NONE = 0,
    ROTATION_YAW_45 = 1,
    ROTATION_YAW_90 = 2,
    ROTATION_YAW_135 = 3,
    ROTATION_YAW_180 = 4,
    ROTATION_YAW_225 = 5,
    ROTATION_YAW_270 = 6,
    ROTATION_YAW_315 = 7,
    ROTATION_ROLL_180 = 8,
    ROTATION_ROLL_180_YAW_45 = 9,
    ROTATION_ROLL_180_YAW_90 = 10,
    ROTATION_ROLL_180_YAW_135 = 11,
    ROTATION_PITCH_180 = 12,
    ROTATION_ROLL_180_YAW_225 = 13,
    ROTATION_ROLL_180_YAW_270 = 14,
    ROTATION_ROLL_180_YAW_315 = 15,
    ROTATION_ROLL_90 = 16,
    ROTATION_ROLL_90_YAW_45 = 17,
    ROTATION_ROLL_90_YAW_90 = 18,
    ROTATION_ROLL_90_YAW_135 = 19,
    ROTATION_ROLL_270 = 20,
    ROTATION_ROLL_270_YAW_45 = 21,
    ROTATION_ROLL_270_YAW_90 = 22,
    ROTATION_ROLL_270_YAW_135 = 23,
    ROTATION_PITCH_90 = 24,
    ROTATION_PITCH_270 = 25,
    ROTATION_PITCH_180_YAW_90 = 26,
    ROTATION_PITCH_180_YAW_270 = 27,
    ROTATION_ROLL_90_PITCH_90 = 28,
    ROTATION_ROLL_180_PITCH_90 = 29,
    ROTATION_ROLL_270_PITCH_90 = 30,
    ROTATION_ROLL_90_PITCH_180 = 31,
    ROTATION_ROLL_270_PITCH_180 = 32,
    ROTATION_ROLL_90_PITCH_270 = 33,
    ROTATION_ROLL_180_PITCH_270 = 34,
    ROTATION_ROLL_270_PITCH_270 = 35,
    ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    ROTATION_ROLL_90_YAW_270 = 37,
    ROTATION_YAW_293_PITCH_68_ROLL_90 = 38,
    ROTATION_MAX
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

typedef enum
{
    BEEPER_CALIBRATION_DONE = 0,
    BEEPER_DISARMING,
    BEEPER_BATT_CRIT_LOW,
    BEEPER_ACTION_SUCCESS,
    BEEPER_ACTION_FAIL,
    BEEPER_ARM,
    BEEPER_ALGORITHM_INIT,
    BEEPER_AUTOLAUNCH,
    BEEPER_LAUNCHED,
    BEEPER_FMU_INIT,
    BEEPER_FMU_SAFE_TO_ARM,
    BEEPER_FAIL_SAFE,
    BEEPER_FAIL_SAFE_GOOD,
    BEEPER_PARACHUTE
} Beeper_Mode;

enum BiquadFilterTypes
{
    LPF = 0,
    NOTCH
};

enum I2C_Supported_Devices_Enum
{
    //ENDEREÇOS EM ORDEM NÚMERICA,DO MENOR PARA O MAIOR
    ADDRESS_COMPASS_AK8975 = 0x0C,
    ADDRESS_COMPASS_QMC5883 = 0x0D,
    ADDRESS_COMPASS_HMC5843_OR_HMC5883 = 0x1E,
    ADDRESS_IMU_MPU6050 = 0x68,
    ADDRESS_BAROMETER_BMP280 = 0x76,
    ADDRESS_BAROMETER_MS5611 = 0x77,
    SIZE_OF_I2C_DEVICES
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

enum Led_Pattern_Enum
{
    FMU_INIT_ARM = 0x0003,
    FMU_REFUSE_TO_ARM = 0x5555,
    FMU_SAFE_TO_ARM = 0xFFFF,
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
    AIR_SPEED_DISABLED = 0,
    ANALOG_AIR_SPEED,
    DIGITAL_AIR_SPEED,
    VIRTUAL_AIR_SPEED
};

enum Ublox_Protocol_Byte_Enum
{
    PREAMBLE1 = 0xB5,
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
#ifndef __AVR_ATmega2560__
    TASK_FAST_MEDIUM_LOOP,
    TASK_FAST_LOOP,
    TASK_SUPER_FAST_LOOP,
#endif
    TASK_INTEGRAL_LOOP,
    TASK_SYSTEM_LOAD,
    //TASK COUNT SEMPRE EM ÚLTIMO LUGAR
    TASK_COUNT
} Tasks_ID_Enum;

enum GCS_Message_Type_Enum
{
    IMU_ERROR = 0,
    FLIGHT_MODES_ERROR,
    GPS_ERROR,
    FAIL_SAFE_ERROR,
    GYRO_EEROR,
    INCLINATION_ERROR,
    BUTTON_ERROR,
    BATTERY_ERROR,
    COMPASS_ERROR,
    BAROMETER_ERROR,
    NONE_ERROR = 254
};

typedef enum
{
    SERVO_AUTOTRIM_IDLE = 0,
    SERVO_AUTOTRIM_COLLECTING,
    SERVO_AUTOTRIM_SAVE_PENDING,
    SERVO_AUTOTRIM_DONE,
} ServoAutoTrimState_Enum;

enum Receivers_Type_Enum
{
    PPM_RECEIVER = 0,
    SBUS_RECEIVER,
    IBUS_RECEIVER
};

enum GPS_Velocity_NED_Enum
{
    NORTH = 0,
    EAST,
    DOWN
};

enum GCS_FlightModes_Enum
{
    GCS_ACRO_MODE = 0,
    GCS_STABILIZE_MODE,
    GCS_ALTITUDE_HOLD_MODE,
    GCS_ATTACK_MODE,
    GCS_POS_HOLD_MODE,
    GCS_SIMPLE_MODE,
    GCS_RTH_MODE,
    GCS_LAND_MODE = 8,
    GCS_FLIP_MODE = 11,
    GCS_WAYPOINT_MODE,
    GCS_LANDED_MODE
};

enum Throttle_Status_Enum
{
    THROTTLE_LOW = 0,
    THROTTLE_MIDDLE,
    THROTTLE_HIGH
};

enum GPS_Type_Enum
{
    GPS_UBLOX = 0,
    GPS_DJI_NAZA
};

enum Inertial_Navigation_Enum
{
    INS_LATITUDE = 0,
    INS_LONGITUDE,
    INS_VERTICAL_Z
};

enum GPS_Orientation_Enum
{
    COORD_LATITUDE = 0,
    COORD_LONGITUDE
};
#endif