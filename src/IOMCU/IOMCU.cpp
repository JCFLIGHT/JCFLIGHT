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

#include "IOMCU.h"
#include "FastSerial/FASTSERIAL.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "I2C/I2C.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/RCCONFIG.h"
#include "Barometer/BAROREAD.h"
#include "Barometer/BAROBACKEND.h"
#include "BatteryMonitor/BATTERY.h"
#include "LedRGB/LEDRGB.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "Buzzer/BUZZER.h"
#include "GPSNavigation/NAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "ParamsToGCS/IMUCALGCS.h"
#include "AirSpeed/AIRSPEEDBACKEND.h"
#include "MemoryCheck/FREERAM.h"
#include "AirSpeed/AIRSPEED.h"
#include "Build/VERSION.h"
#include "ProgMem/PROGMEM.h"
#include "WatchDog/REBOOT.h"
#include "Arming/ARMING.h"
#include "PID/PIDXYZ.h"
#include "Scheduler/SCHEDULER.h"
#include "TaskSystem/TASKSYSTEM.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Compass/COMPASSREAD.h"
#include "FailSafe/FAILSAFE.h"
#include "GPS/GPSUBLOX.h"
#include "PID/RCPID.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDPARAMS.h"

GCSClass GCS;

//#define MACHINE_CYCLE

uint8_t SerialCheckSum;
uint8_t ProtocolCommand;
uint8_t SerialInputBuffer[64];
uint8_t SerialOutputBuffer[128];
uint8_t SerialOutputBufferSizeCount;
uint8_t VectorCount;
uint8_t SerialBuffer;
uint8_t SerialAvailableGuard;
uint8_t ProtocolTaskOrder;
uint8_t SerialOffSet;
uint8_t SerialDataSize;
uint8_t PreviousProtocolTaskOrder;

struct _Essential_First_Packet_Parameters
{
    int16_t SendAttitudePitch;
    int16_t SendAttitudeRoll;
    int16_t SendAttitudeYaw;
    uint8_t DevicesOnBoard;
    uint16_t SendThrottleValue;
    uint16_t SendYawValue;
    uint16_t SendPitchValue;
    uint16_t SendRollValue;
    uint16_t SendAuxOneValue;
    uint16_t SendAuxTwoValue;
    uint16_t SendAuxThreeValue;
    uint16_t SendAuxFourValue;
    uint16_t SendAuxFiveValue;
    uint16_t SendAuxSixValue;
    uint16_t SendAuxSevenValue;
    uint16_t SendAuxEightValue;
    uint8_t SendGPSNumberOfSat;
    int32_t SendGPSLatitude;
    int32_t SendGPSLongitude;
    int32_t SendHomePointLatitude;
    int32_t SendHomePointLongitude;
    int32_t SendBarometerValue;
    uint8_t SendFailSafeState;
    uint16_t SendBatteryVoltageValue;
    uint8_t SendBatteryPercentageValue;
    uint8_t SendArmDisarmState;
    uint16_t SendHDOPValue;
    uint16_t SendCurrentValue;
    uint32_t SendWattsValue;
    int16_t SendDeclinationValue;
    uint8_t SendActualFlightMode;
    uint8_t SendFrameType;
    uint8_t SendHomePointState;
    uint8_t SendTemperature;
    uint16_t SendHomePointDistance;
    uint16_t SendCurrentInMah;
    uint16_t SendCourseOverGround;
    int16_t SendBearing;
    int16_t SendAccGForce;
    uint8_t SendAccImageBitMap;
    int16_t SendCompassRoll;
    int16_t SendCompassPitch;
    int16_t SendCompassYaw;
} Essential_First_Packet_Parameters;

struct _Essential_Second_Packet_Parameters
{
    uint16_t SendActualThrottleValue;
    uint16_t SendActualYawValue;
    uint16_t SendActualPitchValue;
    uint16_t SendActualRollValue;
    uint16_t SendActualAuxOneValue;
    uint16_t SendActualAuxTwoValue;
    uint16_t SendActualAuxThreeValue;
    uint16_t SendActualAuxFourValue;
    uint16_t SendActualAuxFiveValue;
    uint16_t SendActualAuxSixValue;
    uint16_t SendActualAuxSevenValue;
    uint16_t SendActualAuxEightValue;
    int16_t SendAttitudeThrottleValue;
    int16_t SendAttitudeYawValue;
    int16_t SendAttitudePitchValue;
    int16_t SendAttitudeRollValue;
    uint16_t SendMemoryRamUsed;
    uint8_t SendMemoryRamUsedPercent;
    int16_t SendAccXNotFiltered;
    int16_t SendAccYNotFiltered;
    int16_t SendAccZNotFiltered;
    int16_t SendAccXFiltered;
    int16_t SendAccYFiltered;
    int16_t SendAccZFiltered;
    int16_t SendGyroXNotFiltered;
    int16_t SendGyroYNotFiltered;
    int16_t SendGyroZNotFiltered;
    int16_t SendGyroXFiltered;
    int16_t SendGyroYFiltered;
    int16_t SendGyroZFiltered;
    uint16_t SendGPSGroundSpeed;
    int16_t SendI2CError;
    uint16_t SendAirSpeedValue;
    uint8_t SendCPULoad;
} Essential_Second_Packet_Parameters;

struct _Send_User_Basic_Parameters
{
    uint8_t SendFrameType;
    uint8_t SendReceiverType;
    uint8_t SendGimbalType;
    uint8_t SendParachuteType;
    uint8_t SendSPIType;
    uint8_t SendUART_NUMB_2Type;
    uint8_t SendUartNumb1Type;
    uint8_t SendCompassRotationType;
    uint8_t SendRTHAltitudeType;
    uint8_t SendAcroType;
    uint8_t SendAltitudeHoldType;
    uint8_t SendPositionHoldType;
    uint8_t SendSimpleControlType;
    uint8_t SendReturnToHomeType;
    uint8_t SendAtackType;
    uint8_t SendAutomaticFlipType;
    uint8_t SendAutomaticMissonType;
    uint8_t SendArmDisarmType;
    uint8_t SendAutoLandType;
    uint8_t SendSafeBtnState;
    uint8_t SendAirSpeedState;
    int16_t SendAccRollAdjust;
    int16_t SendAccPitchAdjust;
    int16_t SendAccYawAdjust;
} Send_User_Basic_Parameters;

struct _Get_User_Basic_Parameters
{
    uint8_t GetFrameType;
    uint8_t GetReceiverType;
    uint8_t GetGimbalType;
    uint8_t GetParachuteType;
    uint8_t GetSPIType;
    uint8_t GetUART_NUMB_2Type;
    uint8_t GetUartNumb1Type;
    uint8_t GetCompassRotationType;
    uint8_t GetRTHAltitudeType;
    uint8_t GetAcroType;
    uint8_t GetAltitudeHoldType;
    uint8_t GetPositionHoldType;
    uint8_t GetSimpleControlType;
    uint8_t GetReturnToHomeType;
    uint8_t GetAtackType;
    uint8_t GetAutomaticFlipType;
    uint8_t GetAutomaticMissonType;
    uint8_t GetArmDisarmType;
    uint8_t GetAutoLandType;
    uint8_t GetSafeBtnState;
    uint8_t GetAirSpeedState;
    int16_t GetAccRollAdjust;
    int16_t GetAccPitchAdjust;
    int16_t GetAccYawAdjust;
} Get_User_Basic_Parameters;

struct _Send_Radio_Control_Parameters
{
    uint8_t SendThrottleMiddle;
    uint8_t SendThrottleExpo;
    uint8_t SendRCRate;
    uint8_t SendRCExpo;
    uint8_t SendYawRate;
    int16_t SendRCPulseMin;
    int16_t SendRCPulseMax;
    uint8_t SendAHDeadZone;
    uint8_t SendAHSafeAltitude;
    uint8_t SendAHMinVelVertical;
    int16_t SendThrottleMin;
    int16_t SendYawMin;
    int16_t SendPitchMin;
    int16_t SendRollMin;
    int16_t SendThrottleMax;
    int16_t SendYawMax;
    int16_t SendPitchMax;
    int16_t SendRollMax;
    uint8_t SendThrottleDeadZone;
    uint8_t SendYawDeadZone;
    uint8_t SendPitchDeadZone;
    uint8_t SendRollDeadZone;
    uint8_t SendChannelsReverse;
    int16_t SendServo1Rate;
    int16_t SendServo2Rate;
    int16_t SendServo3Rate;
    int16_t SendServo4Rate;
    uint8_t SendServosReverse;
    int16_t SendServo1Min;
    int16_t SendServo2Min;
    int16_t SendServo3Min;
    int16_t SendServo4Min;
    int16_t SendServo1Med;
    int16_t SendServo2Med;
    int16_t SendServo3Med;
    int16_t SendServo4Med;
    int16_t SendServo1Max;
    int16_t SendServo2Max;
    int16_t SendServo3Max;
    int16_t SendServo4Max;
    int16_t SendFailSafeValue;
} Send_Radio_Control_Parameters;

struct _Get_Radio_Control_Parameters
{
    uint8_t GetThrottleMiddle;
    uint8_t GetThrottleExpo;
    uint8_t GetRCRate;
    uint8_t GetRCExpo;
    uint8_t GetYawRate;
    int16_t GetRCPulseMin;
    int16_t GetRCPulseMax;
    uint8_t GetAHDeadZone;
    uint8_t GetAHSafeAltitude;
    uint8_t GetAHMinVelVertical;
    int16_t GetThrottleMin;
    int16_t GetYawMin;
    int16_t GetPitchMin;
    int16_t GetRollMin;
    int16_t GetThrottleMax;
    int16_t GetYawMax;
    int16_t GetPitchMax;
    int16_t GetRollMax;
    uint8_t GetThrottleDeadZone;
    uint8_t GetYawDeadZone;
    uint8_t GetPitchDeadZone;
    uint8_t GetRollDeadZone;
    uint8_t GetChannelsReverse;
    int16_t GetFailSafeValue;
} Get_Radio_Control_Parameters;

struct _Get_Servos_Parameters
{
    int16_t GetServo1Rate;
    int16_t GetServo2Rate;
    int16_t GetServo3Rate;
    int16_t GetServo4Rate;
    uint8_t GetServosReverse;
    int16_t GetServo1Min;
    int16_t GetServo2Min;
    int16_t GetServo3Min;
    int16_t GetServo4Min;
    int16_t GetServo1Med;
    int16_t GetServo2Med;
    int16_t GetServo3Med;
    int16_t GetServo4Med;
    int16_t GetServo1Max;
    int16_t GetServo2Max;
    int16_t GetServo3Max;
    int16_t GetServo4Max;
} Get_Servos_Parameters;

struct _Send_User_Medium_Parameters
{
    uint8_t SendTPAInPercent;
    int16_t SendBreakPointValue;
    uint8_t SendGyroLPF;
    int16_t SendDerivativeLPF;
    int16_t SendRCLPF;
    uint8_t SendKalmanState;
    int16_t SendBiQuadAccLPF;
    int16_t SendBiQuadGyroLPF;
    int16_t SendBiQuadAccNotch;
    int16_t SendBiQuadGyroNotch;
    uint8_t SendMotorCompensationState;
    uint8_t SendProportionalPitch;
    uint8_t SendIntegralPitch;
    uint8_t SendDerivativePitch;
    uint8_t SendProportionalRoll;
    uint8_t SendIntegralRoll;
    uint8_t SendDerivativeRoll;
    uint8_t SendProportionalYaw;
    uint8_t SendIntegralYaw;
    uint8_t SendDerivativeYaw;
    uint8_t SendProportionalAltitudeHold;
    uint8_t SendProportionalGPSHold;
    uint8_t SendIntegralGPSHold;
    int16_t SendServosLPF;
    uint8_t SendCDOrFFRoll;
    uint8_t SendCDOrFFPitch;
    uint8_t SendCDOrFFYaw;
    uint8_t SendAutoLevelProportional;
    uint8_t SendAutoLevelIntegral;
    uint8_t SendHeadingHoldRate;
    uint8_t SendHeadingHoldRateLimit;
    uint8_t SendRollBankMax;
    uint8_t SendPitchBankMin;
    uint8_t SendPitchBankMax;
    uint8_t SendAttackBank;
    uint8_t SendGPSBank;
} Send_User_Medium_Parameters;

struct _Get_User_Medium_Parameters
{
    uint8_t GetTPAInPercent;
    int16_t GetBreakPointValue;
    uint8_t GetGyroLPF;
    int16_t GetDerivativeLPF;
    int16_t GetRCLPF;
    uint8_t GetKalmanState;
    int16_t GetBiquadAccLPF;
    int16_t GetBiquadGyroLPF;
    int16_t GetBiquadAccNotch;
    int16_t GetBiquadGyroNotch;
    uint8_t GetMotorCompensationState;
    uint8_t GetProportionalPitch;
    uint8_t GetIntegralPitch;
    uint8_t GetDerivativePitch;
    uint8_t GetProportionalRoll;
    uint8_t GetIntegralRoll;
    uint8_t GetDerivativeRoll;
    uint8_t GetProportionalYaw;
    uint8_t GetIntegralYaw;
    uint8_t GetDerivativeYaw;
    uint8_t GetProportionalAltitudeHold;
    uint8_t GetProportionalGPSHold;
    uint8_t GetIntegralGPSHold;
    int16_t GetServosLPF;
    uint8_t GetCDOrFFRoll;
    uint8_t GetCDOrFFPitch;
    uint8_t GetCDOrFFYaw;
    uint8_t GetAutoLevelProportional;
    uint8_t GetAutoLevelIntegral;
    uint8_t GetHeadingHoldRate;
    uint8_t GetHeadingHoldRateLimit;
    uint8_t GetRollBankMax;
    uint8_t GetPitchBankMin;
    uint8_t GetPitchBankMax;
    uint8_t GetAttackBank;
    uint8_t GetGPSBank;
} Get_User_Medium_Parameters;

struct _Send_WayPoint_Coordinates
{
    int32_t SendLatitudeOne;
    int32_t SendLatitudeTwo;
    int32_t SendLatitudeThree;
    int32_t SendLatitudeFour;
    int32_t SendLatitudeFive;
    int32_t SendLatitudeSix;
    int32_t SendLatitudeSeven;
    int32_t SendLatitudeEight;
    int32_t SendLatitudeNine;
    int32_t SendLatitudeTen;
    int32_t SendLongitudeOne;
    int32_t SendLongitudeTwo;
    int32_t SendLongitudeThree;
    int32_t SendLongitudeFour;
    int32_t SendLongitudeFive;
    int32_t SendLongitudeSix;
    int32_t SendLongitudeSeven;
    int32_t SendLongitudeEight;
    int32_t SendLongitudeNine;
    int32_t SendLongitudeTen;
} Send_WayPoint_Coordinates;

struct _Send_WayPoint_Misc_Parameters
{
    uint8_t SendAltitudeOne;
    uint8_t SendAltitudeTwo;
    uint8_t SendAltitudeThree;
    uint8_t SendAltitudeFour;
    uint8_t SendAltitudeFive;
    uint8_t SendAltitudeSix;
    uint8_t SendAltitudeSeven;
    uint8_t SendAltitudeEight;
    uint8_t SendAltitudeNine;
    uint8_t SendAltitudeTen;
    uint8_t SendFlightModeOne;
    uint8_t SendFlightModeTwo;
    uint8_t SendFlightModeThree;
    uint8_t SendFlightModeFour;
    uint8_t SendFlightModeFive;
    uint8_t SendFlightModeSix;
    uint8_t SendFlightModeSeven;
    uint8_t SendFlightModeEight;
    uint8_t SendFlightModeNine;
    uint8_t SendFlightModeTen;
    uint8_t SendGPSHoldTimedOne;
    uint8_t SendGPSHoldTimedTwo;
    uint8_t SendGPSHoldTimedThree;
    uint8_t SendGPSHoldTimedFour;
    uint8_t SendGPSHoldTimedFive;
    uint8_t SendGPSHoldTimedSix;
    uint8_t SendGPSHoldTimedSeven;
    uint8_t SendGPSHoldTimedEight;
    uint8_t SendGPSHoldTimedNine;
    uint8_t SendGPSHoldTimedTen;
} Send_WayPoint_Misc_Parameters;

#ifdef __AVR_ATmega2560__

static void Send_Data_To_GCS(uint8_t Buffer)
{
    FASTSERIAL.StoreTX(UART_NUMB_0, Buffer);
    SerialCheckSum ^= Buffer;

#elif defined ESP32 || defined __arm__

static void Send_Data_To_GCS(int32_t Buffer, uint8_t Length)
{
    switch (Length)
    {
    case VAR_8BITS:
        uint8_t MemGet8BitsBuffer;
        memcpy(&MemGet8BitsBuffer, &Buffer, sizeof(uint8_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet8BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet8BitsBuffer & 0xff;
        break;

    case VAR_16BITS:
        int16_t MemGet16BitsBuffer;
        memcpy(&MemGet16BitsBuffer, &Buffer, sizeof(int16_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet16BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet16BitsBuffer & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet16BitsBuffer >> 8) & 0xff;
        SerialCheckSum ^= (MemGet16BitsBuffer >> 8) & 0xff;
        break;

    case VAR_32BITS:
        int32_t MemGet32BitsBuffer;
        memcpy(&MemGet32BitsBuffer, &Buffer, sizeof(int32_t));
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = MemGet32BitsBuffer & 0xff;
        SerialCheckSum ^= MemGet32BitsBuffer & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 8) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 8) & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 16) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 16) & 0xff;
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = (MemGet32BitsBuffer >> 24) & 0xff;
        SerialCheckSum ^= (MemGet32BitsBuffer >> 24) & 0xff;
        break;
    }

#endif
}

static void Communication_Passed(bool Error, uint8_t Buffer)
{
#ifdef __AVR_ATmega2560__

    FASTSERIAL.StoreTX(UART_NUMB_0, 0x4A);
    SerialCheckSum ^= 0x4A;
    FASTSERIAL.StoreTX(UART_NUMB_0, 0x43);
    SerialCheckSum ^= 0x43;
    FASTSERIAL.StoreTX(UART_NUMB_0, Error ? 0x21 : 0x46);
    SerialCheckSum ^= Error ? 0x21 : 0x46;
    SerialCheckSum = 0;
    FASTSERIAL.StoreTX(UART_NUMB_0, Buffer);
    SerialCheckSum ^= Buffer;
    FASTSERIAL.StoreTX(UART_NUMB_0, ProtocolCommand);
    SerialCheckSum ^= ProtocolCommand;

#elif defined ESP32 || defined __arm__

    SerialOutputBuffer[SerialOutputBufferSizeCount++] = 0x4A;
    SerialCheckSum ^= 0x4A;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = 0x43;
    SerialCheckSum ^= 0x43;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = Error ? 0x21 : 0x46;
    SerialCheckSum ^= Error ? 0x21 : 0x46;
    SerialCheckSum = 0;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = Buffer;
    SerialCheckSum ^= Buffer;
    SerialOutputBuffer[SerialOutputBufferSizeCount++] = ProtocolCommand;
    SerialCheckSum ^= ProtocolCommand;

#endif
}

#ifdef __AVR_ATmega2560__

static void Send_Struct_Params_To_GCS(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    Communication_Passed(false, SizeOfBuffer);
    while (SizeOfBuffer--)
    {
        Send_Data_To_GCS(*CheckBuffer++);
    }
    Send_Data_To_GCS(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);
}

#endif

static void __attribute__((noinline)) Get_Struct_Params_To_GCS(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    while (SizeOfBuffer--)
    {
        *CheckBuffer++ = SerialInputBuffer[VectorCount++] & 0xff;
    }
}

void GCSClass::Send_String_To_GCS(const char *String)
{
#ifdef __AVR_ATmega2560__

    Communication_Passed(false, strlen_P(String));
    for (const char *StringCount = String; ProgMemReadByte(StringCount); StringCount++)
    {
        Send_Data_To_GCS(ProgMemReadByte(StringCount));
    }
    Send_Data_To_GCS(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);

#elif defined ESP32 || defined __arm__

    Communication_Passed(false, strlen_P(String));
    for (const char *StringCount = String; ProgMemReadByte(StringCount); StringCount++)
    {
        Send_Data_To_GCS(ProgMemReadByte(StringCount), VAR_8BITS);
    }
    Send_Data_To_GCS(SerialCheckSum, VAR_8BITS);

#endif
}

void GCSClass::Serial_Parse_Protocol()
{
    if (GCS.CliMode)
    {
        return;
    }

    SerialAvailableGuard = FASTSERIAL.Available(UART_NUMB_0);
    while (SerialAvailableGuard--)
    {
        if (FASTSERIAL.UsedTXBuffer(UART_NUMB_0) > 78)
        {
            return;
        }

        SerialBuffer = FASTSERIAL.Read(UART_NUMB_0);
        ProtocolTaskOrder = PreviousProtocolTaskOrder;

        switch (ProtocolTaskOrder)
        {

        case 0:
            if (SerialBuffer == 0x4A)
            {
                ProtocolTaskOrder = 1;
            }
            if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && SerialBuffer == 0x23)
            {
                GCS.CliMode = true;
            }
            break;

        case 1:
            ProtocolTaskOrder = (SerialBuffer == 0x43) ? 2 : 0;
            break;

        case 2:
            ProtocolTaskOrder = (SerialBuffer == 0x3C) ? 3 : 0;
            break;

        case 3:
            if (SerialBuffer > 78)
            {
                ProtocolTaskOrder = 0;
                continue;
            }
            SerialDataSize = SerialBuffer;
            SerialCheckSum = SerialBuffer;
            SerialOffSet = 0;
            VectorCount = 0;
            ProtocolTaskOrder = 4;
            break;

        case 4:
            ProtocolCommand = SerialBuffer;
            SerialCheckSum ^= SerialBuffer;
            ProtocolTaskOrder = 5;
            break;

        case 5:
            if (SerialOffSet < SerialDataSize)
            {
                SerialCheckSum ^= SerialBuffer;
                SerialInputBuffer[SerialOffSet++] = SerialBuffer;
            }
            else
            {
                if (SerialCheckSum == SerialBuffer)
                {
                    GCS.Update_BiDirect_Protocol(ProtocolCommand);
                }
                ProtocolTaskOrder = 0;
                SerialAvailableGuard = 0;
            }
            break;
        }
        PreviousProtocolTaskOrder = ProtocolTaskOrder;
    }
#if defined(ESP32) || (defined __arm__)

    while (SerialOutputBufferSizeCount > 0)
    {
        SerialOutputBufferSizeCount--;
        FASTSERIAL.Write(UART_NUMB_0, SerialOutputBuffer[VectorCount++]);
    }

#endif
}

void GCSClass::Update_BiDirect_Protocol(uint8_t TaskOrderGCS)
{
#ifdef __AVR_ATmega2560__

    switch (TaskOrderGCS)
    {

    case 1:
        GCS.WayPoint_Request_Coordinates_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Send_WayPoint_Coordinates, sizeof(_Send_WayPoint_Coordinates));
        break;

    case 2:
        GCS.WayPoint_Request_Misc_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Send_WayPoint_Misc_Parameters, sizeof(_Send_WayPoint_Misc_Parameters));
        break;

    case 3:
        EEPROM_Function = 1;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 4:
        EEPROM_Function = 2;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 5:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointGCSParameters, sizeof(_GetWayPointGCSParameters));
        break;

    case 6:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointGCSParametersTwo, sizeof(_GetWayPointGCSParametersTwo));
        break;

    case 7:
        GCS.First_Packet_Request_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Essential_First_Packet_Parameters, sizeof(_Essential_First_Packet_Parameters));
        break;

    case 8:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_User_Basic_Parameters, sizeof(_Send_User_Basic_Parameters));
        break;

    case 9:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_User_Medium_Parameters, sizeof(_Send_User_Medium_Parameters));
        break;

    case 10:
        GCS.Second_Packet_Request_Parameters();
        Send_Struct_Params_To_GCS((uint8_t *)&Essential_Second_Packet_Parameters, sizeof(_Essential_Second_Packet_Parameters));
        break;

    case 11:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            StartAccCalibration();
        }
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 12:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && I2CResources.Found.Compass)
        {
            IMU.Compass.Calibrating = true;
        }
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 13:
        GCS.ConfigFlight = true;
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 14:
        GCS.ConfigFlight = false;
        Communication_Passed(0, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 15:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Basic_Parameters, sizeof(_Get_User_Basic_Parameters));
        break;

    case 16:
        GCS.Save_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(0, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 17:
        GCS.Default_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 18:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Medium_Parameters, sizeof(_Get_User_Medium_Parameters));
        break;

    case 19:
        GCS.Save_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 20:
        GCS.Default_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 21:
        GCS.Send_String_To_GCS(PlatformName);
        break;

    case 22:
        GCS.Send_String_To_GCS(FirwareName);
        break;

    case 23:
        GCS.Send_String_To_GCS(FirmwareVersion);
        break;

    case 24:
        GCS.Send_String_To_GCS(CompilerVersion);
        break;

    case 25:
        GCS.Send_String_To_GCS(BuildDate);
        break;

    case 26:
        GCS.Send_String_To_GCS(BuildTime);
        break;

    case 27:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 30:
        Send_Struct_Params_To_GCS((uint8_t *)&Send_Radio_Control_Parameters, sizeof(_Send_Radio_Control_Parameters));
        break;

    case 31:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Radio_Control_Parameters, sizeof(_Get_Radio_Control_Parameters));
        break;

    case 32:
        GCS.Save_Radio_Control_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 33:
        Communication_Passed(false, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Servos_Parameters, sizeof(_Get_Servos_Parameters));
        break;

    default:
        Communication_Passed(true, 0);
        Send_Data_To_GCS(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;
    }

#elif defined ESP32 || defined __arm__

    switch (TaskOrderGCS)
    {

    case 3:
        EEPROM_Function = 1;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 4:
        EEPROM_Function = 2;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 5:
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointGCSParameters, sizeof(_GetWayPointGCSParameters));
        break;

    case 6:
        Get_Struct_Params_To_GCS((uint8_t *)&GetWayPointGCSParametersTwo, sizeof(_GetWayPointGCSParametersTwo));
        break;

    case 7:
        GCS.First_Packet_Request_Parameters();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 10) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 27) + //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
                                        (sizeof(int32_t) * 6));  //NÚMERO TOTAL DE VARIAVEIS DE 32 BITS CONTIDO AQUI
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudePitch, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudeRoll, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAttitudeYaw, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.DevicesOnBoard, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendPitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxOneValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxTwoValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxThreeValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxFourValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxFiveValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxSixValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxSevenValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAuxEightValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSNumberOfSat, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSLatitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendGPSLongitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointLatitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointLongitude, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBarometerValue, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendFailSafeState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBatteryVoltageValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBatteryPercentageValue, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendArmDisarmState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHDOPValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCurrentValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendWattsValue, VAR_32BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendDeclinationValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendActualFlightMode, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendFrameType, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointState, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendTemperature, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendHomePointDistance, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCurrentInMah, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCourseOverGround, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendBearing, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAccGForce, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendAccImageBitMap, VAR_8BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassRoll, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassPitch, VAR_16BITS);
        Send_Data_To_GCS(Essential_First_Packet_Parameters.SendCompassYaw, VAR_16BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 8:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 21) +    //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 3)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendFrameType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendReceiverType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendGimbalType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendParachuteType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendSPIType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendUART_NUMB_2Type, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendUartNumb1Type, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendCompassRotationType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendRTHAltitudeType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAcroType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAltitudeHoldType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendPositionHoldType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendSimpleControlType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendReturnToHomeType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAtackType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutomaticFlipType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutomaticMissonType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendArmDisarmType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAutoLandType, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendSafeBtnState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAirSpeedState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAccRollAdjust, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAccPitchAdjust, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Basic_Parameters.SendAccYawAdjust, VAR_16BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 9:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 28) +    //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 8)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendTPAInPercent, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBreakPointValue, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendGyroLPF, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendRCLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendKalmanState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadAccLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadGyroLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadAccNotch, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendBiQuadGyroNotch, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendMotorCompensationState, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativePitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendDerivativeYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalAltitudeHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendProportionalGPSHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendIntegralGPSHold, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendServosLPF, VAR_16BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFRoll, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFPitch, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendCDOrFFYaw, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAutoLevelProportional, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAutoLevelIntegral, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendHeadingHoldRate, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendHeadingHoldRateLimit, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendRollBankMax, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendPitchBankMin, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendPitchBankMax, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendAttackBank, VAR_8BITS);
        Send_Data_To_GCS(Send_User_Medium_Parameters.SendGPSBank, VAR_8BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 10:
        GCS.Second_Packet_Request_Parameters();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 2) +      //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 32)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualPitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxOneValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxTwoValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxThreeValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxFourValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxFiveValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxSixValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxSevenValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendActualAuxEightValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeThrottleValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeYawValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudePitchValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAttitudeRollValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendMemoryRamUsed, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendMemoryRamUsedPercent, VAR_8BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccXNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccYNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccZNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccXFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccYFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAccZFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroXNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroYNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroZNotFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroXFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroYFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGyroZFiltered, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendGPSGroundSpeed, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendI2CError, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendAirSpeedValue, VAR_16BITS);
        Send_Data_To_GCS(Essential_Second_Packet_Parameters.SendCPULoad, VAR_8BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 11:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            StartAccCalibration();
        }
        break;

    case 12:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM) && I2CResources.Found.Compass)
        {
            IMU.Compass.Calibrating = true;
        }
        break;

    case 13:
        GCS.ConfigFlight = true;
        break;

    case 14:
        GCS.ConfigFlight = false;
        break;

    case 15:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Basic_Parameters, sizeof(_Get_User_Basic_Parameters));
        break;

    case 16:
        GCS.Save_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 17:
        GCS.Default_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 18:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_User_Medium_Parameters, sizeof(_Get_User_Medium_Parameters));
        break;

    case 19:
        GCS.Save_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 20:
        GCS.Default_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 21:
        GCS.Send_String_To_GCS(PlatformName);
        break;

    case 22:
        GCS.Send_String_To_GCS(FirwareName);
        break;

    case 23:
        GCS.Send_String_To_GCS(FirmwareVersion);
        break;

    case 24:
        GCS.Send_String_To_GCS(CompilerVersion);
        break;

    case 25:
        GCS.Send_String_To_GCS(BuildDate);
        break;

    case 26:
        GCS.Send_String_To_GCS(BuildTime);
        break;

    case 27:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!IS_STATE_ACTIVE(PRIMARY_ARM_DISARM))
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl_Configuration();
        break;

    case 30:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 14) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 27)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMiddle, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleExpo, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCRate, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCExpo, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawRate, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCPulseMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRCPulseMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAHDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAHSafeAltitude, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendAHMinVelVertical, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollMin, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollMax, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendThrottleDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendYawDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendPitchDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendRollDeadZone, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendChannelsReverse, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Rate, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServosReverse, VAR_8BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Min, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Med, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo1Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo2Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo3Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendServo4Max, VAR_16BITS);
        Send_Data_To_GCS(Send_Radio_Control_Parameters.SendFailSafeValue, VAR_16BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 31:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Radio_Control_Parameters, sizeof(_Get_Radio_Control_Parameters));
        break;

    case 32:
        GCS.Save_Radio_Control_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        break;

    case 33:
        Get_Struct_Params_To_GCS((uint8_t *)&Get_Servos_Parameters, sizeof(_Get_Servos_Parameters));
        break;
    }

#endif
}

void GCSClass::First_Packet_Request_Parameters()
{
    //ENVIA OS PARAMETROS FUNDAMENTAIS PARA O GCS
    Essential_First_Packet_Parameters.SendAttitudePitch = Constrain_16Bits(ATTITUDE.AngleOut[PITCH], -900, 900);
    Essential_First_Packet_Parameters.SendAttitudeRoll = Constrain_16Bits(ATTITUDE.AngleOut[ROLL], -900, 900);
    if (I2CResources.Found.Compass)
    {
        Essential_First_Packet_Parameters.SendAttitudeYaw = ATTITUDE.AngleOut[YAW];
    }
    else
    {
        Essential_First_Packet_Parameters.SendAttitudeYaw = WRap_18000(GPS_Ground_Course * 10) / 10;
    }
    Essential_First_Packet_Parameters.DevicesOnBoard = CHECKSUM.GetDevicesActived();
    Essential_First_Packet_Parameters.SendThrottleValue = Throttle.Output;
    Essential_First_Packet_Parameters.SendYawValue = Yaw.Output;
    Essential_First_Packet_Parameters.SendPitchValue = Pitch.Output;
    Essential_First_Packet_Parameters.SendRollValue = Roll.Output;
    Essential_First_Packet_Parameters.SendAuxOneValue = AuxiliarOne.Output;
    Essential_First_Packet_Parameters.SendAuxTwoValue = AuxiliarTwo.Output;
    Essential_First_Packet_Parameters.SendAuxThreeValue = AuxiliarThree.Output;
    Essential_First_Packet_Parameters.SendAuxFourValue = AuxiliarFour.Output;
    Essential_First_Packet_Parameters.SendAuxFiveValue = AuxiliarFive.Output;
    Essential_First_Packet_Parameters.SendAuxSixValue = AuxiliarSix.Output;
    Essential_First_Packet_Parameters.SendAuxSevenValue = AuxiliarSeven.Output;
    Essential_First_Packet_Parameters.SendAuxEightValue = AuxiliarEight.Output;
    Essential_First_Packet_Parameters.SendGPSNumberOfSat = GPS_NumberOfSatellites;
    Essential_First_Packet_Parameters.SendGPSLatitude = GPS_Coordinates_Vector[COORD_LATITUDE];
    Essential_First_Packet_Parameters.SendGPSLongitude = GPS_Coordinates_Vector[COORD_LONGITUDE];
    Essential_First_Packet_Parameters.SendHomePointLatitude = Stored_Coordinates_Home_Point[COORD_LATITUDE];
    Essential_First_Packet_Parameters.SendHomePointLongitude = Stored_Coordinates_Home_Point[COORD_LONGITUDE];
    if (I2CResources.Found.Barometer)
    {
        Essential_First_Packet_Parameters.SendBarometerValue = GetAltitudeForGCS();
    }
    else
    {
        Essential_First_Packet_Parameters.SendBarometerValue = (GPS_Altitude - GPS_Altitude_For_Plane) * 100;
    }
    Essential_First_Packet_Parameters.SendFailSafeState = SystemInFailSafe();
    Essential_First_Packet_Parameters.SendBatteryVoltageValue = BATTERY.Get_Actual_Voltage() * 100;
    Essential_First_Packet_Parameters.SendBatteryPercentageValue = BATTERY.GetPercentage();
    Essential_First_Packet_Parameters.SendArmDisarmState = IS_STATE_ACTIVE(PRIMARY_ARM_DISARM);
    Essential_First_Packet_Parameters.SendHDOPValue = GPS_HDOP;
    Essential_First_Packet_Parameters.SendCurrentValue = BATTERY.Get_Actual_Current();
    Essential_First_Packet_Parameters.SendWattsValue = BATTERY.GetWatts();
    Essential_First_Packet_Parameters.SendDeclinationValue = (int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100);
    Essential_First_Packet_Parameters.SendActualFlightMode = FlightMode;
    Essential_First_Packet_Parameters.SendFrameType = FrameType;
    Essential_First_Packet_Parameters.SendHomePointState = Home_Point;
    Essential_First_Packet_Parameters.SendTemperature = Barometer.Raw.Temperature / 100;
    Essential_First_Packet_Parameters.SendHomePointDistance = DistanceToHome;
    Essential_First_Packet_Parameters.SendCurrentInMah = BATTERY.Get_Current_In_Mah();
#ifndef MACHINE_CYCLE
    Essential_First_Packet_Parameters.SendCourseOverGround = GPS_Ground_Course;
#else
    Essential_First_Packet_Parameters.SendCourseOverGround = GetTaskDeltaTime(TASK_INTEGRAL_LOOP);
#endif
    Essential_First_Packet_Parameters.SendBearing = Target_Bearing;
    Essential_First_Packet_Parameters.SendAccGForce = IMU.Accelerometer.GravityForce.Value;
    Essential_First_Packet_Parameters.SendAccImageBitMap = GetImageToGCS();
    Essential_First_Packet_Parameters.SendCompassRoll = IMU.Compass.Read[ROLL];
    Essential_First_Packet_Parameters.SendCompassPitch = IMU.Compass.Read[PITCH];
    Essential_First_Packet_Parameters.SendCompassYaw = IMU.Compass.Read[YAW];
}

void GCSClass::Second_Packet_Request_Parameters()
{
    Essential_Second_Packet_Parameters.SendActualThrottleValue = DECODE.DirectRadioControllRead[THROTTLE];
    Essential_Second_Packet_Parameters.SendActualYawValue = DECODE.DirectRadioControllRead[YAW];
    Essential_Second_Packet_Parameters.SendActualPitchValue = DECODE.DirectRadioControllRead[PITCH];
    Essential_Second_Packet_Parameters.SendActualRollValue = DECODE.DirectRadioControllRead[ROLL];
    Essential_Second_Packet_Parameters.SendActualAuxOneValue = DECODE.DirectRadioControllRead[AUX1];
    Essential_Second_Packet_Parameters.SendActualAuxTwoValue = DECODE.DirectRadioControllRead[AUX2];
    Essential_Second_Packet_Parameters.SendActualAuxThreeValue = DECODE.DirectRadioControllRead[AUX3];
    Essential_Second_Packet_Parameters.SendActualAuxFourValue = DECODE.DirectRadioControllRead[AUX4];
    Essential_Second_Packet_Parameters.SendActualAuxFiveValue = DECODE.DirectRadioControllRead[AUX5];
    Essential_Second_Packet_Parameters.SendActualAuxSixValue = DECODE.DirectRadioControllRead[AUX6];
    Essential_Second_Packet_Parameters.SendActualAuxSevenValue = DECODE.DirectRadioControllRead[AUX7];
    Essential_Second_Packet_Parameters.SendActualAuxEightValue = DECODE.DirectRadioControllRead[AUX8];
    Essential_Second_Packet_Parameters.SendAttitudeThrottleValue = RCController[THROTTLE];
    Essential_Second_Packet_Parameters.SendAttitudeYawValue = PIDXYZ.CalcedRateTargetYawToGCS;
    Essential_Second_Packet_Parameters.SendAttitudePitchValue = PIDXYZ.CalcedRateTargetPitchToGCS;
    Essential_Second_Packet_Parameters.SendAttitudeRollValue = PIDXYZ.CalcedRateTargetRollToGCS;
    Essential_Second_Packet_Parameters.SendMemoryRamUsed = MEMORY.Check();
    Essential_Second_Packet_Parameters.SendMemoryRamUsedPercent = MEMORY.GetPercentageRAMUsed();
    Essential_Second_Packet_Parameters.SendAccXNotFiltered = IMU.Accelerometer.ReadNotFiltered[ROLL];
    Essential_Second_Packet_Parameters.SendAccYNotFiltered = IMU.Accelerometer.ReadNotFiltered[PITCH];
    Essential_Second_Packet_Parameters.SendAccZNotFiltered = IMU.Accelerometer.ReadNotFiltered[YAW];
    Essential_Second_Packet_Parameters.SendAccXFiltered = IMU.Accelerometer.Read[ROLL];
    Essential_Second_Packet_Parameters.SendAccYFiltered = IMU.Accelerometer.Read[PITCH];
    Essential_Second_Packet_Parameters.SendAccZFiltered = IMU.Accelerometer.Read[YAW];
    Essential_Second_Packet_Parameters.SendGyroXNotFiltered = IMU.Gyroscope.ReadNotFiltered[ROLL];
    Essential_Second_Packet_Parameters.SendGyroYNotFiltered = IMU.Gyroscope.ReadNotFiltered[PITCH];
    Essential_Second_Packet_Parameters.SendGyroZNotFiltered = IMU.Gyroscope.ReadNotFiltered[YAW];
    Essential_Second_Packet_Parameters.SendGyroXFiltered = IMU.Gyroscope.Read[ROLL];
    Essential_Second_Packet_Parameters.SendGyroYFiltered = IMU.Gyroscope.Read[PITCH];
    Essential_Second_Packet_Parameters.SendGyroZFiltered = IMU.Gyroscope.Read[YAW];
    Essential_Second_Packet_Parameters.SendGPSGroundSpeed = GPS_Ground_Speed;
    Essential_Second_Packet_Parameters.SendI2CError = I2CResources.Error.Count;
    Essential_Second_Packet_Parameters.SendAirSpeedValue = AirSpeed.Raw.IASPressureInCM;
    Essential_Second_Packet_Parameters.SendCPULoad = SystemLoadPercent;
}

void GCSClass::WayPoint_Request_Coordinates_Parameters()
{
    Send_WayPoint_Coordinates.SendLatitudeOne = STORAGEMANAGER.Read_32Bits(704);
    Send_WayPoint_Coordinates.SendLatitudeTwo = STORAGEMANAGER.Read_32Bits(708);
    Send_WayPoint_Coordinates.SendLatitudeThree = STORAGEMANAGER.Read_32Bits(712);
    Send_WayPoint_Coordinates.SendLatitudeFour = STORAGEMANAGER.Read_32Bits(716);
    Send_WayPoint_Coordinates.SendLatitudeFive = STORAGEMANAGER.Read_32Bits(720);
    Send_WayPoint_Coordinates.SendLatitudeSix = STORAGEMANAGER.Read_32Bits(724);
    Send_WayPoint_Coordinates.SendLatitudeSeven = STORAGEMANAGER.Read_32Bits(728);
    Send_WayPoint_Coordinates.SendLatitudeEight = STORAGEMANAGER.Read_32Bits(732);
    Send_WayPoint_Coordinates.SendLatitudeNine = STORAGEMANAGER.Read_32Bits(736);
    Send_WayPoint_Coordinates.SendLatitudeTen = STORAGEMANAGER.Read_32Bits(740);
    Send_WayPoint_Coordinates.SendLongitudeOne = STORAGEMANAGER.Read_32Bits(744);
    Send_WayPoint_Coordinates.SendLongitudeTwo = STORAGEMANAGER.Read_32Bits(748);
    Send_WayPoint_Coordinates.SendLongitudeThree = STORAGEMANAGER.Read_32Bits(752);
    Send_WayPoint_Coordinates.SendLongitudeFour = STORAGEMANAGER.Read_32Bits(756);
    Send_WayPoint_Coordinates.SendLongitudeFive = STORAGEMANAGER.Read_32Bits(760);
    Send_WayPoint_Coordinates.SendLongitudeSix = STORAGEMANAGER.Read_32Bits(764);
    Send_WayPoint_Coordinates.SendLongitudeSeven = STORAGEMANAGER.Read_32Bits(768);
    Send_WayPoint_Coordinates.SendLongitudeEight = STORAGEMANAGER.Read_32Bits(772);
    Send_WayPoint_Coordinates.SendLongitudeNine = STORAGEMANAGER.Read_32Bits(776);
    Send_WayPoint_Coordinates.SendLongitudeTen = STORAGEMANAGER.Read_32Bits(780);
}

void GCSClass::WayPoint_Request_Misc_Parameters()
{
    Send_WayPoint_Misc_Parameters.SendAltitudeOne = STORAGEMANAGER.Read_8Bits(804);
    Send_WayPoint_Misc_Parameters.SendAltitudeTwo = STORAGEMANAGER.Read_8Bits(805);
    Send_WayPoint_Misc_Parameters.SendAltitudeThree = STORAGEMANAGER.Read_8Bits(806);
    Send_WayPoint_Misc_Parameters.SendAltitudeFour = STORAGEMANAGER.Read_8Bits(807);
    Send_WayPoint_Misc_Parameters.SendAltitudeFive = STORAGEMANAGER.Read_8Bits(808);
    Send_WayPoint_Misc_Parameters.SendAltitudeSix = STORAGEMANAGER.Read_8Bits(809);
    Send_WayPoint_Misc_Parameters.SendAltitudeSeven = STORAGEMANAGER.Read_8Bits(810);
    Send_WayPoint_Misc_Parameters.SendAltitudeEight = STORAGEMANAGER.Read_8Bits(811);
    Send_WayPoint_Misc_Parameters.SendAltitudeNine = STORAGEMANAGER.Read_8Bits(812);
    Send_WayPoint_Misc_Parameters.SendAltitudeTen = STORAGEMANAGER.Read_8Bits(813);
    Send_WayPoint_Misc_Parameters.SendFlightModeOne = STORAGEMANAGER.Read_8Bits(794);
    Send_WayPoint_Misc_Parameters.SendFlightModeTwo = STORAGEMANAGER.Read_8Bits(795);
    Send_WayPoint_Misc_Parameters.SendFlightModeThree = STORAGEMANAGER.Read_8Bits(796);
    Send_WayPoint_Misc_Parameters.SendFlightModeFour = STORAGEMANAGER.Read_8Bits(797);
    Send_WayPoint_Misc_Parameters.SendFlightModeFive = STORAGEMANAGER.Read_8Bits(798);
    Send_WayPoint_Misc_Parameters.SendFlightModeSix = STORAGEMANAGER.Read_8Bits(799);
    Send_WayPoint_Misc_Parameters.SendFlightModeSeven = STORAGEMANAGER.Read_8Bits(800);
    Send_WayPoint_Misc_Parameters.SendFlightModeEight = STORAGEMANAGER.Read_8Bits(801);
    Send_WayPoint_Misc_Parameters.SendFlightModeNine = STORAGEMANAGER.Read_8Bits(802);
    Send_WayPoint_Misc_Parameters.SendFlightModeTen = STORAGEMANAGER.Read_8Bits(803);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedOne = STORAGEMANAGER.Read_8Bits(784);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedTwo = STORAGEMANAGER.Read_8Bits(785);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedThree = STORAGEMANAGER.Read_8Bits(786);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedFour = STORAGEMANAGER.Read_8Bits(787);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedFive = STORAGEMANAGER.Read_8Bits(788);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedSix = STORAGEMANAGER.Read_8Bits(789);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedSeven = STORAGEMANAGER.Read_8Bits(790);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedEight = STORAGEMANAGER.Read_8Bits(791);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedNine = STORAGEMANAGER.Read_8Bits(792);
    Send_WayPoint_Misc_Parameters.SendGPSHoldTimedTen = STORAGEMANAGER.Read_8Bits(793);
}

void GCSClass::Save_Basic_Configuration()
{
    STORAGEMANAGER.Write_8Bits(FRAMETYPE_ADDR, Get_User_Basic_Parameters.GetFrameType);
    STORAGEMANAGER.Write_8Bits(RECEIVER_ADDR, Get_User_Basic_Parameters.GetReceiverType);
    STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, Get_User_Basic_Parameters.GetGimbalType);
    STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, Get_User_Basic_Parameters.GetParachuteType);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_3_ADDR, Get_User_Basic_Parameters.GetSPIType);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, Get_User_Basic_Parameters.GetUART_NUMB_2Type);
    STORAGEMANAGER.Write_8Bits(UART_NUMB_1_ADDR, Get_User_Basic_Parameters.GetUartNumb1Type);
    STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, Get_User_Basic_Parameters.GetCompassRotationType);
    STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, Get_User_Basic_Parameters.GetRTHAltitudeType);
    STORAGEMANAGER.Write_8Bits(STABLIZE_ADDR, Get_User_Basic_Parameters.GetAcroType);
    STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, Get_User_Basic_Parameters.GetAltitudeHoldType);
    STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, Get_User_Basic_Parameters.GetPositionHoldType);
    STORAGEMANAGER.Write_8Bits(SIMPLE_ADDR, Get_User_Basic_Parameters.GetSimpleControlType);
    STORAGEMANAGER.Write_8Bits(RTH_ADDR, Get_User_Basic_Parameters.GetReturnToHomeType);
    STORAGEMANAGER.Write_8Bits(ATACK_ADDR, Get_User_Basic_Parameters.GetAtackType);
    STORAGEMANAGER.Write_8Bits(AUTOFLIP_ADDR, Get_User_Basic_Parameters.GetAutomaticFlipType);
    STORAGEMANAGER.Write_8Bits(AUTOMISSION_ADDR, Get_User_Basic_Parameters.GetAutomaticMissonType);
    STORAGEMANAGER.Write_8Bits(ARMDISARM_ADDR, Get_User_Basic_Parameters.GetArmDisarmType);
    STORAGEMANAGER.Write_8Bits(AUTOLAND_ADDR, Get_User_Basic_Parameters.GetAutoLandType);
    STORAGEMANAGER.Write_8Bits(DISP_PASSIVES_ADDR, Get_User_Basic_Parameters.GetSafeBtnState);
    STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, Get_User_Basic_Parameters.GetAirSpeedState);
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADJUST_ADDR, Get_User_Basic_Parameters.GetAccRollAdjust);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADJUST_ADDR, Get_User_Basic_Parameters.GetAccPitchAdjust);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_ADJUST_ADDR, Get_User_Basic_Parameters.GetAccYawAdjust);

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Save_Radio_Control_Configuration()
{
    STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, Get_Radio_Control_Parameters.GetThrottleMiddle);
    STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, Get_Radio_Control_Parameters.GetThrottleExpo);
    STORAGEMANAGER.Write_8Bits(RC_RATE_ADDR, Get_Radio_Control_Parameters.GetRCRate);
    STORAGEMANAGER.Write_8Bits(RC_EXPO_ADDR, Get_Radio_Control_Parameters.GetRCExpo);
    STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, Get_Radio_Control_Parameters.GetYawRate);
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MIN_ADDR, Get_Radio_Control_Parameters.GetRCPulseMin);
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MAX_ADDR, Get_Radio_Control_Parameters.GetRCPulseMax);
    STORAGEMANAGER.Write_8Bits(AH_DEADZONE_ADDR, Get_Radio_Control_Parameters.GetAHDeadZone);
    STORAGEMANAGER.Write_8Bits(AH_SAFE_ALT_ADDR, Get_Radio_Control_Parameters.GetAHSafeAltitude);
    STORAGEMANAGER.Write_8Bits(AH_MIN_VEL_VERT_ADDR, Get_Radio_Control_Parameters.GetAHMinVelVertical);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MIN_ADDR, Get_Radio_Control_Parameters.GetThrottleMin);
    STORAGEMANAGER.Write_16Bits(YAW_MIN_ADDR, Get_Radio_Control_Parameters.GetYawMin);
    STORAGEMANAGER.Write_16Bits(PITCH_MIN_ADDR, Get_Radio_Control_Parameters.GetPitchMin);
    STORAGEMANAGER.Write_16Bits(ROLL_MIN_ADDR, Get_Radio_Control_Parameters.GetRollMin);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MAX_ADDR, Get_Radio_Control_Parameters.GetThrottleMax);
    STORAGEMANAGER.Write_16Bits(YAW_MAX_ADDR, Get_Radio_Control_Parameters.GetYawMax);
    STORAGEMANAGER.Write_16Bits(PITCH_MAX_ADDR, Get_Radio_Control_Parameters.GetPitchMax);
    STORAGEMANAGER.Write_16Bits(ROLL_MAX_ADDR, Get_Radio_Control_Parameters.GetRollMax);
    STORAGEMANAGER.Write_8Bits(THROTTLE_DZ_ADDR, Get_Radio_Control_Parameters.GetThrottleDeadZone);
    STORAGEMANAGER.Write_8Bits(YAW_DZ_ADDR, Get_Radio_Control_Parameters.GetYawDeadZone);
    STORAGEMANAGER.Write_8Bits(PITCH_DZ_ADDR, Get_Radio_Control_Parameters.GetPitchDeadZone);
    STORAGEMANAGER.Write_8Bits(ROLL_DZ_ADDR, Get_Radio_Control_Parameters.GetRollDeadZone);
    STORAGEMANAGER.Write_8Bits(CH_REVERSE_ADDR, Get_Radio_Control_Parameters.GetChannelsReverse);
    STORAGEMANAGER.Write_16Bits(SERVO1_RATE_ADDR, Get_Servos_Parameters.GetServo1Rate);
    STORAGEMANAGER.Write_16Bits(SERVO2_RATE_ADDR, Get_Servos_Parameters.GetServo2Rate);
    STORAGEMANAGER.Write_16Bits(SERVO3_RATE_ADDR, Get_Servos_Parameters.GetServo3Rate);
    STORAGEMANAGER.Write_16Bits(SERVO4_RATE_ADDR, Get_Servos_Parameters.GetServo4Rate);
    STORAGEMANAGER.Write_8Bits(SERVOS_REVERSE_ADDR, Get_Servos_Parameters.GetServosReverse);
    STORAGEMANAGER.Write_16Bits(SERVO1_MIN_ADDR, Get_Servos_Parameters.GetServo1Min);
    STORAGEMANAGER.Write_16Bits(SERVO2_MIN_ADDR, Get_Servos_Parameters.GetServo2Min);
    STORAGEMANAGER.Write_16Bits(SERVO3_MIN_ADDR, Get_Servos_Parameters.GetServo3Min);
    STORAGEMANAGER.Write_16Bits(SERVO4_MIN_ADDR, Get_Servos_Parameters.GetServo4Min);
    STORAGEMANAGER.Write_16Bits(SERVO1_MID_ADDR, Get_Servos_Parameters.GetServo1Med);
    STORAGEMANAGER.Write_16Bits(SERVO2_MID_ADDR, Get_Servos_Parameters.GetServo2Med);
    STORAGEMANAGER.Write_16Bits(SERVO3_MID_ADDR, Get_Servos_Parameters.GetServo3Med);
    STORAGEMANAGER.Write_16Bits(SERVO4_MID_ADDR, Get_Servos_Parameters.GetServo4Med);
    STORAGEMANAGER.Write_16Bits(SERVO1_MAX_ADDR, Get_Servos_Parameters.GetServo1Max);
    STORAGEMANAGER.Write_16Bits(SERVO2_MAX_ADDR, Get_Servos_Parameters.GetServo2Max);
    STORAGEMANAGER.Write_16Bits(SERVO3_MAX_ADDR, Get_Servos_Parameters.GetServo3Max);
    STORAGEMANAGER.Write_16Bits(SERVO4_MAX_ADDR, Get_Servos_Parameters.GetServo4Max);
    STORAGEMANAGER.Write_16Bits(FAILSAFE_VAL_ADDR, Get_Radio_Control_Parameters.GetFailSafeValue);
}

void GCSClass::Save_Medium_Configuration()
{
    STORAGEMANAGER.Write_8Bits(TPA_PERCENT_ADDR, Get_User_Medium_Parameters.GetTPAInPercent);
    STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, Get_User_Medium_Parameters.GetBreakPointValue);
    STORAGEMANAGER.Write_8Bits(GYRO_LPF_ADDR, Get_User_Medium_Parameters.GetGyroLPF);
    STORAGEMANAGER.Write_16Bits(DERIVATIVE_LPF_ADDR, Get_User_Medium_Parameters.GetDerivativeLPF);
    STORAGEMANAGER.Write_16Bits(RC_LPF_ADDR, Get_User_Medium_Parameters.GetRCLPF);
    STORAGEMANAGER.Write_8Bits(KALMAN_ADDR, Get_User_Medium_Parameters.GetKalmanState);
    STORAGEMANAGER.Write_16Bits(BI_ACC_LPF_ADDR, Get_User_Medium_Parameters.GetBiquadAccLPF);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_LPF_ADDR, Get_User_Medium_Parameters.GetBiquadGyroLPF);
    STORAGEMANAGER.Write_16Bits(BI_ACC_NOTCH_ADDR, Get_User_Medium_Parameters.GetBiquadAccNotch);
    STORAGEMANAGER.Write_16Bits(BI_GYRO_NOTCH_ADDR, Get_User_Medium_Parameters.GetBiquadGyroNotch);
    STORAGEMANAGER.Write_8Bits(MOTCOMP_STATE_ADDR, Get_User_Medium_Parameters.GetMotorCompensationState);
    STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, Get_User_Medium_Parameters.GetProportionalPitch);
    STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, Get_User_Medium_Parameters.GetIntegralPitch);
    STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, Get_User_Medium_Parameters.GetDerivativePitch);
    STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, Get_User_Medium_Parameters.GetProportionalRoll);
    STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, Get_User_Medium_Parameters.GetIntegralRoll);
    STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, Get_User_Medium_Parameters.GetDerivativeRoll);
    STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, Get_User_Medium_Parameters.GetProportionalYaw);
    STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, Get_User_Medium_Parameters.GetIntegralYaw);
    STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, Get_User_Medium_Parameters.GetDerivativeYaw);
    STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, Get_User_Medium_Parameters.GetProportionalAltitudeHold);
    STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, Get_User_Medium_Parameters.GetProportionalGPSHold);
    STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, Get_User_Medium_Parameters.GetIntegralGPSHold);
    STORAGEMANAGER.Write_16Bits(SERVOS_LPF_ADDR, Get_User_Medium_Parameters.GetServosLPF);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, Get_User_Medium_Parameters.GetCDOrFFRoll);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, Get_User_Medium_Parameters.GetCDOrFFPitch);
    STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, Get_User_Medium_Parameters.GetCDOrFFYaw);
    STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, Get_User_Medium_Parameters.GetAutoLevelProportional);
    STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, Get_User_Medium_Parameters.GetAutoLevelIntegral);
    STORAGEMANAGER.Write_8Bits(KP_HEADING_HOLD_ADDR, Get_User_Medium_Parameters.GetHeadingHoldRate);
    STORAGEMANAGER.Write_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR, Get_User_Medium_Parameters.GetHeadingHoldRateLimit);
    STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, Get_User_Medium_Parameters.GetRollBankMax);
    STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, Get_User_Medium_Parameters.GetPitchBankMin);
    STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, Get_User_Medium_Parameters.GetPitchBankMax);
    STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, Get_User_Medium_Parameters.GetAttackBank);
    STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, Get_User_Medium_Parameters.GetGPSBank);

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Default_Basic_Configuration()
{
    //LIMPA TODAS AS CONFIGURAÇÕES SALVAS
    STORAGEMANAGER.Write_8Bits(SIMPLE_ADDR, 0);            //LIMPA A CONFIGURAÇÃO DO MODO SIMPLES
    STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO ALTITUDE-HOLD
    STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO GPS-HOLD
    STORAGEMANAGER.Write_8Bits(RTH_ADDR, 0);               //LIMPA A CONFIGURAÇÃO DO MODO RTH
    STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, 0);         //LIMPA A CONFIGURAÇÃO DO PARACHUTE
    STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, 0);            //LIMPA A CONFIGURAÇÃO DO CONTROLE DO GIMBAL
    STORAGEMANAGER.Write_8Bits(FRAMETYPE_ADDR, 0);         //LIMPA A CONFIGURAÇÃO DO TIPO DE FRAME
    STORAGEMANAGER.Write_8Bits(RECEIVER_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODULO RECEPTOR
    STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, 0);       //LIMPA A CONFIGURAÇÃO DA UART_NUMB_2
    STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, 0);  //LIMPA A CONFIGURAÇÃO DE ROTAÇÃO DO COMPASS
    STORAGEMANAGER.Write_8Bits(UART_NUMB_1_ADDR, 0);       //LIMPA A CONFIGURAÇÃO DA UART_NUMB_1
    STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, 0);      //LIMPA A CONFIGURAÇÃO DA ALTITUDE AO FAZER O RTH
    STORAGEMANAGER.Write_8Bits(UART_NUMB_3_ADDR, 0);       //LIMPA A CONFIGURAÇÃO DA SPI
    STORAGEMANAGER.Write_8Bits(STABLIZE_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO ACRO
    STORAGEMANAGER.Write_8Bits(ATACK_ADDR, 0);             //LIMPA A CONFIGURAÇÃO DO MODO ATAQUE
    STORAGEMANAGER.Write_8Bits(AUTOFLIP_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO AUTO-FLIP
    STORAGEMANAGER.Write_8Bits(AUTOMISSION_ADDR, 0);       //LIMPA A CONFIGURAÇÃO DO MODO AUTO
    STORAGEMANAGER.Write_8Bits(ARMDISARM_ADDR, 0);         //LIMPA A CONFIGURAÇÃO DO ARMDISARM VIA CHAVE AUX
    STORAGEMANAGER.Write_8Bits(AUTOLAND_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO AUTO LAND
    STORAGEMANAGER.Write_8Bits(DISP_PASSIVES_ADDR, 0);     //LIMPA A CONFIGURAÇÃO DO SAFE BUTTON
    STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, 0);     //LIMPA A CONFIGURAÇÃO DO AIR-SPEED
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADJUST_ADDR, 0);  //LIMPA A CONFIGURAÇÃO DO AJUSTE DO ACC NO ROLL
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADJUST_ADDR, 0); //LIMPA A CONFIGURAÇÃO DO AJUSTE DO ACC NO PITCH
    STORAGEMANAGER.Write_16Bits(ACC_YAW_ADJUST_ADDR, 0);   //LIMPA A CONFIGURAÇÃO DO AJUSTE DO ACC NO YAW
}

void GCSClass::Default_RadioControl_Configuration()
{
    //LIMPA TODAS AS CONFIGURAÇÕES SALVAS DO RÁDIO E DOS SERVOS
    STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, 50);
    STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, 0);
    STORAGEMANAGER.Write_8Bits(RC_EXPO_ADDR, 65);
    STORAGEMANAGER.Write_8Bits(RC_RATE_ADDR, 90);
    STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, 20);
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MAX_ADDR, 1900);
    STORAGEMANAGER.Write_8Bits(AH_DEADZONE_ADDR, 70);
    STORAGEMANAGER.Write_8Bits(AH_SAFE_ALT_ADDR, 5);
    STORAGEMANAGER.Write_8Bits(AH_MIN_VEL_VERT_ADDR, 50);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(YAW_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(PITCH_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(ROLL_MIN_ADDR, 1050);
    STORAGEMANAGER.Write_16Bits(THROTTLE_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(YAW_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(PITCH_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_16Bits(ROLL_MAX_ADDR, 1950);
    STORAGEMANAGER.Write_8Bits(THROTTLE_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(YAW_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(PITCH_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(ROLL_DZ_ADDR, 45);
    STORAGEMANAGER.Write_8Bits(CH_REVERSE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(SERVO1_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO2_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO3_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO4_MIN_ADDR, 1000);
    STORAGEMANAGER.Write_16Bits(SERVO1_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO2_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO3_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO4_MID_ADDR, 1500);
    STORAGEMANAGER.Write_16Bits(SERVO1_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO2_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO3_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO4_MAX_ADDR, 2000);
    STORAGEMANAGER.Write_16Bits(SERVO1_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO2_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO3_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(SERVO4_RATE_ADDR, 100);
    STORAGEMANAGER.Write_16Bits(FAILSAFE_VAL_ADDR, 975);
}

void GCSClass::Default_Medium_Configuration()
{
    //LIMPA AS CONFIGURAÇÕES SALVAS
    STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, 1500);   //VOLTA O BREAK-POINT AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_8Bits(TPA_PERCENT_ADDR, 0);      //VOLTA O DYNAMICPID AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_8Bits(GYRO_LPF_ADDR, 0);         //VOLTA O GYROLPF AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(DERIVATIVE_LPF_ADDR, 40); //VOLTA O DERIVATIVELPF AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(RC_LPF_ADDR, 50);         //VOLTA O RCSMOOTH AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_8Bits(KALMAN_ADDR, 0);           //VOLTA O KALMANSTATE AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(BI_ACC_LPF_ADDR, 15);     //VOLTA O ACCLPF AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(BI_GYRO_LPF_ADDR, 60);    //VOLTA O GYROLPF AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(BI_ACC_NOTCH_ADDR, 0);    //VOLTA O ACCNOTCH AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(BI_GYRO_NOTCH_ADDR, 0);   //VOLTA O GYRONOTCH AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_8Bits(MOTCOMP_STATE_ADDR, 0);    //VOLTA O COMPENSATION SPEED AO PADRÃO DE FABRICA
    STORAGEMANAGER.Write_16Bits(SERVOS_LPF_ADDR, 50);     //VOLTA O FILTRO LPF DOS SERVOS AO PADRÃO DE FABRICA

    if (GetFrameStateOfMultirotor())
    {
        //PITCH
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 40);
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 30);
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 23);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 60);

        //ROLL
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 40);
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 30);
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 23);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 60);

        //YAW
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 85);
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 45);
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

        //AUTO-NÍVEL
        STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
        STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 15);

        //ÂNGULOS DE NAVEGAÇÃO
        STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 30);
        STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 30);
        STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 30);
        STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 40);
        STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);
    }
    else if (GetFrameStateOfAirPlane())
    {
        //PITCH
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 5);
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 7);
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 0);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 50);

        //ROLL
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 5);
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 7);
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 0);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 50);

        //YAW
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 6);
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 10);
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);
        STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

        //AUTO-NÍVEL
        STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
        STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 5);

        //ÂNGULOS DE NAVEGAÇÃO
        STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 45);
        STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 20);
        STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 25);
        STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 40);
        STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);
    }

    //ALTITUDE-HOLD
    STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, 50);

    //GPS-HOLD
    STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, 100);
    STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, 90);

    //HEADING-HOLD
    STORAGEMANAGER.Write_8Bits(KP_HEADING_HOLD_ADDR, 60);
    STORAGEMANAGER.Write_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR, 90);

    //ATUALIZA OS PARAMETROS DO PID
    GET_SET[PID_UPDATED].State = false;
}

void GCSClass::Default_All_Configs()
{
    GCS.Default_Basic_Configuration();
    GCS.Default_Medium_Configuration();
    GCS.Default_RadioControl_Configuration();
}

void GCSClass::UpdateParametersToGCS()
{
    //ATUALIZA OS PARAMETROS BASICOS AJUSTAVEIS PELO USUARIO
    Send_User_Basic_Parameters.SendFrameType = FrameType;
    Send_User_Basic_Parameters.SendReceiverType = STORAGEMANAGER.Read_8Bits(RECEIVER_ADDR);
    Send_User_Basic_Parameters.SendGimbalType = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);
    Send_User_Basic_Parameters.SendParachuteType = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR);
    Send_User_Basic_Parameters.SendSPIType = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR);
    Send_User_Basic_Parameters.SendUART_NUMB_2Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR);
    Send_User_Basic_Parameters.SendUartNumb1Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_1_ADDR);
    Send_User_Basic_Parameters.SendCompassRotationType = STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR);
    Send_User_Basic_Parameters.SendRTHAltitudeType = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
    Send_User_Basic_Parameters.SendAcroType = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);
    Send_User_Basic_Parameters.SendAltitudeHoldType = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR);
    Send_User_Basic_Parameters.SendPositionHoldType = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);
    Send_User_Basic_Parameters.SendSimpleControlType = STORAGEMANAGER.Read_8Bits(SIMPLE_ADDR);
    Send_User_Basic_Parameters.SendReturnToHomeType = STORAGEMANAGER.Read_8Bits(RTH_ADDR);
    Send_User_Basic_Parameters.SendAtackType = STORAGEMANAGER.Read_8Bits(ATACK_ADDR);
    Send_User_Basic_Parameters.SendAutomaticFlipType = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);
    Send_User_Basic_Parameters.SendAutomaticMissonType = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR);
    Send_User_Basic_Parameters.SendArmDisarmType = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);
    Send_User_Basic_Parameters.SendAutoLandType = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);
    Send_User_Basic_Parameters.SendSafeBtnState = STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR);
    Send_User_Basic_Parameters.SendAirSpeedState = STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR);
    Send_User_Basic_Parameters.SendAccRollAdjust = STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADJUST_ADDR);
    Send_User_Basic_Parameters.SendAccPitchAdjust = STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADJUST_ADDR);
    Send_User_Basic_Parameters.SendAccYawAdjust = STORAGEMANAGER.Read_16Bits(ACC_YAW_ADJUST_ADDR);

    //ATUALIZA OS PARAMETROS DO RADIO CONTROLE
    Send_Radio_Control_Parameters.SendThrottleMiddle = STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR);
    Send_Radio_Control_Parameters.SendThrottleExpo = STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR);
    Send_Radio_Control_Parameters.SendRCRate = STORAGEMANAGER.Read_8Bits(RC_RATE_ADDR);
    Send_Radio_Control_Parameters.SendRCExpo = STORAGEMANAGER.Read_8Bits(RC_EXPO_ADDR);
    Send_Radio_Control_Parameters.SendYawRate = STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR);
    Send_Radio_Control_Parameters.SendRCPulseMin = STORAGEMANAGER.Read_16Bits(RC_PULSE_MIN_ADDR);
    Send_Radio_Control_Parameters.SendRCPulseMax = STORAGEMANAGER.Read_16Bits(RC_PULSE_MAX_ADDR);
    Send_Radio_Control_Parameters.SendAHDeadZone = STORAGEMANAGER.Read_8Bits(AH_DEADZONE_ADDR);
    Send_Radio_Control_Parameters.SendAHSafeAltitude = STORAGEMANAGER.Read_8Bits(AH_SAFE_ALT_ADDR);
    Send_Radio_Control_Parameters.SendAHMinVelVertical = STORAGEMANAGER.Read_8Bits(AH_MIN_VEL_VERT_ADDR);
    Send_Radio_Control_Parameters.SendThrottleMin = STORAGEMANAGER.Read_16Bits(THROTTLE_MIN_ADDR);
    Send_Radio_Control_Parameters.SendYawMin = STORAGEMANAGER.Read_16Bits(YAW_MIN_ADDR);
    Send_Radio_Control_Parameters.SendPitchMin = STORAGEMANAGER.Read_16Bits(PITCH_MIN_ADDR);
    Send_Radio_Control_Parameters.SendRollMin = STORAGEMANAGER.Read_16Bits(ROLL_MIN_ADDR);
    Send_Radio_Control_Parameters.SendThrottleMax = STORAGEMANAGER.Read_16Bits(THROTTLE_MAX_ADDR);
    Send_Radio_Control_Parameters.SendYawMax = STORAGEMANAGER.Read_16Bits(YAW_MAX_ADDR);
    Send_Radio_Control_Parameters.SendPitchMax = STORAGEMANAGER.Read_16Bits(PITCH_MAX_ADDR);
    Send_Radio_Control_Parameters.SendRollMax = STORAGEMANAGER.Read_16Bits(ROLL_MAX_ADDR);
    Send_Radio_Control_Parameters.SendThrottleDeadZone = STORAGEMANAGER.Read_8Bits(THROTTLE_DZ_ADDR);
    Send_Radio_Control_Parameters.SendYawDeadZone = STORAGEMANAGER.Read_8Bits(YAW_DZ_ADDR);
    Send_Radio_Control_Parameters.SendPitchDeadZone = STORAGEMANAGER.Read_8Bits(PITCH_DZ_ADDR);
    Send_Radio_Control_Parameters.SendRollDeadZone = STORAGEMANAGER.Read_8Bits(ROLL_DZ_ADDR);
    Send_Radio_Control_Parameters.SendChannelsReverse = STORAGEMANAGER.Read_8Bits(CH_REVERSE_ADDR);
    Send_Radio_Control_Parameters.SendServo1Rate = STORAGEMANAGER.Read_16Bits(SERVO1_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo2Rate = STORAGEMANAGER.Read_16Bits(SERVO2_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo3Rate = STORAGEMANAGER.Read_16Bits(SERVO3_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServo4Rate = STORAGEMANAGER.Read_16Bits(SERVO4_RATE_ADDR);
    Send_Radio_Control_Parameters.SendServosReverse = STORAGEMANAGER.Read_8Bits(SERVOS_REVERSE_ADDR);
    Send_Radio_Control_Parameters.SendServo1Min = STORAGEMANAGER.Read_16Bits(SERVO1_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo2Min = STORAGEMANAGER.Read_16Bits(SERVO2_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo3Min = STORAGEMANAGER.Read_16Bits(SERVO3_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo4Min = STORAGEMANAGER.Read_16Bits(SERVO4_MIN_ADDR);
    Send_Radio_Control_Parameters.SendServo1Med = STORAGEMANAGER.Read_16Bits(SERVO1_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo2Med = STORAGEMANAGER.Read_16Bits(SERVO2_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo3Med = STORAGEMANAGER.Read_16Bits(SERVO3_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo4Med = STORAGEMANAGER.Read_16Bits(SERVO4_MID_ADDR);
    Send_Radio_Control_Parameters.SendServo1Max = STORAGEMANAGER.Read_16Bits(SERVO1_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo2Max = STORAGEMANAGER.Read_16Bits(SERVO2_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo3Max = STORAGEMANAGER.Read_16Bits(SERVO3_MAX_ADDR);
    Send_Radio_Control_Parameters.SendServo4Max = STORAGEMANAGER.Read_16Bits(SERVO4_MAX_ADDR);
    Send_Radio_Control_Parameters.SendFailSafeValue = STORAGEMANAGER.Read_16Bits(FAILSAFE_VAL_ADDR);

    //ATUALIZA OS PARAMETROS MEDIOS AJUSTAVEIS PELO USUARIO
    Send_User_Medium_Parameters.SendTPAInPercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR);
    Send_User_Medium_Parameters.SendBreakPointValue = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
    Send_User_Medium_Parameters.SendGyroLPF = STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR);
    Send_User_Medium_Parameters.SendDerivativeLPF = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
    Send_User_Medium_Parameters.SendRCLPF = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);
    Send_User_Medium_Parameters.SendKalmanState = STORAGEMANAGER.Read_8Bits(KALMAN_ADDR);
    Send_User_Medium_Parameters.SendBiQuadAccLPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
    Send_User_Medium_Parameters.SendBiQuadGyroLPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
    Send_User_Medium_Parameters.SendBiQuadAccNotch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
    Send_User_Medium_Parameters.SendBiQuadGyroNotch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
    Send_User_Medium_Parameters.SendMotorCompensationState = STORAGEMANAGER.Read_8Bits(MOTCOMP_STATE_ADDR);
    Send_User_Medium_Parameters.SendProportionalPitch = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    Send_User_Medium_Parameters.SendIntegralPitch = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    Send_User_Medium_Parameters.SendDerivativePitch = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    Send_User_Medium_Parameters.SendProportionalRoll = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    Send_User_Medium_Parameters.SendIntegralRoll = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    Send_User_Medium_Parameters.SendDerivativeRoll = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    Send_User_Medium_Parameters.SendProportionalYaw = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    Send_User_Medium_Parameters.SendIntegralYaw = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    Send_User_Medium_Parameters.SendDerivativeYaw = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    Send_User_Medium_Parameters.SendProportionalAltitudeHold = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    Send_User_Medium_Parameters.SendProportionalGPSHold = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    Send_User_Medium_Parameters.SendIntegralGPSHold = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    Send_User_Medium_Parameters.SendServosLPF = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFRoll = STORAGEMANAGER.Read_8Bits(FF_OR_CD_ROLL_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFPitch = STORAGEMANAGER.Read_8Bits(FF_OR_CD_PITCH_ADDR);
    Send_User_Medium_Parameters.SendCDOrFFYaw = STORAGEMANAGER.Read_8Bits(FF_OR_CD_YAW_ADDR);
    Send_User_Medium_Parameters.SendAutoLevelProportional = STORAGEMANAGER.Read_8Bits(KP_AUTOLEVEL_ADDR);
    Send_User_Medium_Parameters.SendAutoLevelIntegral = STORAGEMANAGER.Read_8Bits(KI_AUTOLEVEL_ADDR);
    Send_User_Medium_Parameters.SendHeadingHoldRate = STORAGEMANAGER.Read_8Bits(KP_HEADING_HOLD_ADDR);
    Send_User_Medium_Parameters.SendHeadingHoldRateLimit = STORAGEMANAGER.Read_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR);
    Send_User_Medium_Parameters.SendRollBankMax = STORAGEMANAGER.Read_8Bits(ROLL_BANK_ADDR);
    Send_User_Medium_Parameters.SendPitchBankMin = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MIN_ADDR);
    Send_User_Medium_Parameters.SendPitchBankMax = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MAX_ADDR);
    Send_User_Medium_Parameters.SendAttackBank = STORAGEMANAGER.Read_8Bits(ATTACK_BANK_ADDR);
    Send_User_Medium_Parameters.SendGPSBank = STORAGEMANAGER.Read_8Bits(GPS_BANK_ADDR);
}