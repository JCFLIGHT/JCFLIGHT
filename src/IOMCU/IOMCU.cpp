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
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "I2C/I2C.h"
#include "AHRS/AHRS.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/RCCONFIG.h"
#include "Barometer/BAROREAD.h"
#include "BatteryMonitor/BATTERY.h"
#include "LedRGB/LEDRGB.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "Buzzer/BUZZER.h"
#include "GPSNavigation/MULTIROTORNAVIGATION.h"
#include "GPSNavigation/AIRPLANENAVIGATION.h"
#include "IMU/GFORCE.h"
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

struct _GCSParameters
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
    int16_t SendCrosstrack;
    int16_t SendAccGForce;
    uint8_t SendAccImageBitMap;
    int16_t SendCompassRoll;
    int16_t SendCompassPitch;
    int16_t SendCompassYaw;
} GCSParameters;

struct _GCSParameters_Two
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
} GCSParameters_Two;

struct _SendUserBasicGCSParameters
{
    uint8_t SendFrameType;
    uint8_t SendReceiverType;
    uint8_t SendGimbalType;
    uint8_t SendParachuteType;
    uint8_t SendSPIType;
    uint8_t SendUART_NUMB_2Type;
    uint8_t SendCompassType;
    uint8_t SendCompassRotationType;
    uint8_t SendRTHAltitudeType;
    uint8_t SendMotorSpeedType;
    uint8_t SendAcroType;
    uint8_t SendAltitudeHoldType;
    uint8_t SendPositionHoldType;
    uint8_t SendInteligentOrientationControlType;
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
    uint8_t SendThrottleMiddle;
    uint8_t SendThrottleExpo;
    uint8_t SendRCRate;
    uint8_t SendRCExpo;
    uint8_t SendRollRate;
    uint8_t SendPitchRate;
    uint8_t SendYawRate;
    int16_t SendRCPulseMin;
    int16_t SendRCPulseMax;
    int16_t SendAHThrottleRover;
    uint8_t SendAHDeadZone;
    uint8_t SendAHSafeAltitude;
    uint8_t SendAHMinVelVertical;
} SendUserBasicGCSParameters;

struct _GetUserBasicGCSParameters
{
    uint8_t GetFrameType;
    uint8_t GetReceiverType;
    uint8_t GetGimbalType;
    uint8_t GetParachuteType;
    uint8_t GetSPIType;
    uint8_t GetUART_NUMB_2Type;
    uint8_t GetCompassType;
    uint8_t GetCompassRotationType;
    uint8_t GetRTHAltitudeType;
    uint8_t GetMotorSpeedType;
    uint8_t GetAcroType;
    uint8_t GetAltitudeHoldType;
    uint8_t GetPositionHoldType;
    uint8_t GetInteligentOrientationControlType;
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
    uint8_t GetThrottleMiddle;
    uint8_t GetThrottleExpo;
    uint8_t GetRCRate;
    uint8_t GetRCExpo;
    uint8_t GetRollRate;
    uint8_t GetPitchRate;
    uint8_t GetYawRate;
    int16_t GetRCPulseMin;
    int16_t GetRCPulseMax;
    int16_t GetAHThrottleRover;
    uint8_t GetAHDeadZone;
    uint8_t GetAHSafeAltitude;
    uint8_t GetAHMinVelVertical;
} GetUserBasicGCSParameters;

struct _SendUserMediumGCSParameters
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
} SendUserMediumGCSParameters;

struct _GetUserMediumGCSParameters
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
} GetUserMediumGCSParameters;

struct _SendWayPointGCSCoordinates
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
} SendWayPointGCSCoordinates;

struct _SendWayPointGCSOthersParameters
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
} SendWayPointGCSOthersParameters;

#ifdef __AVR_ATmega2560__

static void GCS_Send_Data(uint8_t Buffer)
{
    FASTSERIAL.StoreTX(UART_NUMB_0, Buffer);
    SerialCheckSum ^= Buffer;

#elif defined ESP32 || defined __arm__

static void GCS_Send_Data(int32_t Buffer, uint8_t Length)
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

    FASTSERIAL.StoreTX(UART_NUMB_0, 0x4a);
    SerialCheckSum ^= 0x4a;
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

    SerialOutputBuffer[SerialOutputBufferSizeCount++] = 0x4a;
    SerialCheckSum ^= 0x4a;
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

static void GCS_Send_Struct_Params(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    Communication_Passed(false, SizeOfBuffer);
    while (SizeOfBuffer--)
    {
        GCS_Send_Data(*CheckBuffer++);
    }
    GCS_Send_Data(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);
}

#endif

static void __attribute__((noinline)) GCS_Get_Struct_Params(uint8_t *CheckBuffer, uint8_t SizeOfBuffer)
{
    while (SizeOfBuffer--)
    {
        *CheckBuffer++ = SerialInputBuffer[VectorCount++] & 0xff;
    }
}

void GCSClass::SendStringToGCS(const char *String)
{
#ifdef __AVR_ATmega2560__

    Communication_Passed(false, strlen_P(String));
    for (const char *StringCount = String; ProgMemReadByte(StringCount); StringCount++)
    {
        GCS_Send_Data(ProgMemReadByte(StringCount));
    }
    GCS_Send_Data(SerialCheckSum);
    FASTSERIAL.UartSendData(UART_NUMB_0);

#elif defined ESP32 || defined __arm__

    Communication_Passed(false, strlen_P(String));
    for (const char *StringCount = String; ProgMemReadByte(StringCount); StringCount++)
    {
        GCS_Send_Data(ProgMemReadByte(StringCount), VAR_8BITS);
    }
    GCS_Send_Data(SerialCheckSum, VAR_8BITS);

#elif defined __arm__

#endif
}

void GCSClass::Serial_Parse_Protocol()
{
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
            if (SerialBuffer == 0x4a)
            {
                ProtocolTaskOrder = 1;
            }
            break;

        case 1:
            ProtocolTaskOrder = (SerialBuffer == 0x43) ? 2 : 0;
            break;

        case 2:
            ProtocolTaskOrder = (SerialBuffer == 0x3c) ? 3 : 0;
            break;

        case 3:
            if (SerialBuffer > 64)
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
                    BiDirectionalCommunication(ProtocolCommand);
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

void GCSClass::BiDirectionalCommunication(uint8_t TaskOrderGCS)
{
#ifdef __AVR_ATmega2560__

    switch (TaskOrderGCS)
    {

    case 1:
        WayPoint_Request_Coordinates_Parameters();
        GCS_Send_Struct_Params((uint8_t *)&SendWayPointGCSCoordinates, sizeof(_SendWayPointGCSCoordinates));
        break;

    case 2:
        WayPoint_Request_Others_Parameters();
        GCS_Send_Struct_Params((uint8_t *)&SendWayPointGCSOthersParameters, sizeof(_SendWayPointGCSOthersParameters));
        break;

    case 3:
        EEPROM_Function = 1;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 4:
        EEPROM_Function = 2;
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 5:
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        GCS_Get_Struct_Params((uint8_t *)&GetWayPointGCSParameters, sizeof(_GetWayPointGCSParameters));
        break;

    case 6:
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        GCS_Get_Struct_Params((uint8_t *)&GetWayPointGCSParametersTwo, sizeof(_GetWayPointGCSParametersTwo));
        break;

    case 7:
        GCS_Request_Parameters();
        GCS_Send_Struct_Params((uint8_t *)&GCSParameters, sizeof(_GCSParameters));
        break;

    case 8:
        GCS_Send_Struct_Params((uint8_t *)&SendUserBasicGCSParameters, sizeof(_SendUserBasicGCSParameters));
        break;

    case 9:
        GCS_Send_Struct_Params((uint8_t *)&SendUserMediumGCSParameters, sizeof(_SendUserMediumGCSParameters));
        break;

    case 10:
        GCS_Request_Parameters_Two();
        GCS_Send_Struct_Params((uint8_t *)&GCSParameters_Two, sizeof(_GCSParameters_Two));
        break;

    case 11:
        if (!COMMAND_ARM_DISARM)
        {
            CalibratingAccelerometer = 512;
        }
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 12:
        if (!COMMAND_ARM_DISARM)
        {
            CalibratingCompass = true;
        }
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 13:
        GCS.ConfigFlight = true;
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 14:
        GCS.ConfigFlight = false;
        Communication_Passed(0, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 15:
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        GCS_Get_Struct_Params((uint8_t *)&GetUserBasicGCSParameters, sizeof(_GetUserBasicGCSParameters));
        break;

    case 16:
        GCS.Save_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(0, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 17:
        GCS.Default_Basic_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 18:
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        GCS_Get_Struct_Params((uint8_t *)&GetUserMediumGCSParameters, sizeof(_GetUserMediumGCSParameters));
        break;

    case 19:
        GCS.Save_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 20:
        GCS.Default_Medium_Configuration();
        BEEPER.Play(BEEPER_ACTION_SUCCESS);
        Communication_Passed(false, 0);
        GCS_Send_Data(SerialCheckSum);
        FASTSERIAL.UartSendData(UART_NUMB_0);
        break;

    case 21:
        SendStringToGCS(PlatformName);
        break;

    case 22:
        SendStringToGCS(FirwareName);
        break;

    case 23:
        SendStringToGCS(FirmwareVersion);
        break;

    case 24:
        SendStringToGCS(CompilerVersion);
        break;

    case 25:
        SendStringToGCS(BuildDate);
        break;

    case 26:
        SendStringToGCS(BuildTime);
        break;

    case 27:
        if (!COMMAND_ARM_DISARM)
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!COMMAND_ARM_DISARM)
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl();
        break;

    default:
        Communication_Passed(true, 0);
        GCS_Send_Data(SerialCheckSum);
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
        GCS_Get_Struct_Params((uint8_t *)&GetWayPointGCSParameters, sizeof(_GetWayPointGCSParameters));
        break;

    case 6:
        GCS_Get_Struct_Params((uint8_t *)&GetWayPointGCSParametersTwo, sizeof(_GetWayPointGCSParametersTwo));
        break;

    case 7:
        GCS.GCS_Request_Parameters();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 10) +     //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 27) + //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
                                        (sizeof(int32_t) * 6));  //NÚMERO TOTAL DE VARIAVEIS DE 32 BITS CONTIDO AQUI
        GCS_Send_Data(GCSParameters.SendAttitudePitch, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAttitudeRoll, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAttitudeYaw, VAR_16BITS);
        GCS_Send_Data(GCSParameters.DevicesOnBoard, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendThrottleValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendYawValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendPitchValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendRollValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxOneValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxTwoValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxThreeValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxFourValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxFiveValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxSixValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxSevenValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAuxEightValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendGPSNumberOfSat, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendGPSLatitude, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendGPSLongitude, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendHomePointLatitude, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendHomePointLongitude, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendBarometerValue, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendFailSafeState, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendBatteryVoltageValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendBatteryPercentageValue, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendArmDisarmState, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendHDOPValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCurrentValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendWattsValue, VAR_32BITS);
        GCS_Send_Data(GCSParameters.SendDeclinationValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendActualFlightMode, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendFrameType, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendHomePointState, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendTemperature, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendHomePointDistance, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCurrentInMah, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCourseOverGround, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCrosstrack, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAccGForce, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendAccImageBitMap, VAR_8BITS);
        GCS_Send_Data(GCSParameters.SendCompassRoll, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCompassPitch, VAR_16BITS);
        GCS_Send_Data(GCSParameters.SendCompassYaw, VAR_16BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 8:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 32) +    //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 6)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        GCS_Send_Data(SendUserBasicGCSParameters.SendFrameType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendReceiverType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendGimbalType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendParachuteType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendSPIType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendUART_NUMB_2Type, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendCompassType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendCompassRotationType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRTHAltitudeType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendMotorSpeedType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAcroType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAltitudeHoldType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendPositionHoldType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendInteligentOrientationControlType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendReturnToHomeType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAtackType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAutomaticFlipType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAutomaticMissonType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendArmDisarmType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAutoLandType, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendSafeBtnState, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAirSpeedState, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAccRollAdjust, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAccPitchAdjust, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAccYawAdjust, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendThrottleMiddle, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendThrottleExpo, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRCRate, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRCExpo, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRollRate, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendPitchRate, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendYawRate, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRCPulseMin, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendRCPulseMax, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAHThrottleRover, VAR_16BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAHDeadZone, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAHSafeAltitude, VAR_8BITS);
        GCS_Send_Data(SendUserBasicGCSParameters.SendAHMinVelVertical, VAR_8BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 9:
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 16) +    //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 8)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        GCS_Send_Data(SendUserMediumGCSParameters.SendTPAInPercent, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendBreakPointValue, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendGyroLPF, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendDerivativeLPF, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendRCLPF, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendKalmanState, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendBiQuadAccLPF, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendBiQuadGyroLPF, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendBiQuadAccNotch, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendBiQuadGyroNotch, VAR_16BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendMotorCompensationState, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendProportionalPitch, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendIntegralPitch, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendDerivativePitch, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendProportionalRoll, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendIntegralRoll, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendDerivativeRoll, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendProportionalYaw, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendIntegralYaw, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendDerivativeYaw, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendProportionalAltitudeHold, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendProportionalGPSHold, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendIntegralGPSHold, VAR_8BITS);
        GCS_Send_Data(SendUserMediumGCSParameters.SendServosLPF, VAR_16BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 10:
        GCS.GCS_Request_Parameters_Two();
        //RESETA E CALCULA O TAMANHO DO NOVO BUFFER
        SerialOutputBufferSizeCount = 0;
        VectorCount = 0;
        Communication_Passed(false, (sizeof(uint8_t) * 2) +      //NÚMERO TOTAL DE VARIAVEIS DE 8 BITS CONTIDO AQUI
                                        (sizeof(int16_t) * 32)); //NÚMERO TOTAL DE VARIAVEIS DE 16 BITS CONTIDO AQUI
        GCS_Send_Data(GCSParameters_Two.SendActualThrottleValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualYawValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualPitchValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualRollValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxOneValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxTwoValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxThreeValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxFourValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxFiveValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxSixValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxSevenValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendActualAuxEightValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAttitudeThrottleValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAttitudeYawValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAttitudePitchValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAttitudeRollValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendMemoryRamUsed, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendMemoryRamUsedPercent, VAR_8BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccXNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccYNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccZNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccXFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccYFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAccZFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroXNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroYNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroZNotFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroXFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroYFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGyroZFiltered, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendGPSGroundSpeed, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendI2CError, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendAirSpeedValue, VAR_16BITS);
        GCS_Send_Data(GCSParameters_Two.SendCPULoad, VAR_8BITS);
        //SOMA DO BUFFER
        SerialOutputBuffer[SerialOutputBufferSizeCount++] = SerialCheckSum;
        SerialCheckSum ^= SerialCheckSum;
        break;

    case 11:
        if (!COMMAND_ARM_DISARM)
        {
            CalibratingAccelerometer = 512;
        }
        break;

    case 12:
        if (!COMMAND_ARM_DISARM && I2C.CompassFound)
        {
            CalibratingCompass = true;
        }
        break;

    case 13:
        GCS.ConfigFlight = true;
        break;

    case 14:
        GCS.ConfigFlight = false;
        break;

    case 15:
        GCS_Get_Struct_Params((uint8_t *)&GetUserBasicGCSParameters, sizeof(_GetUserBasicGCSParameters));
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
        GCS_Get_Struct_Params((uint8_t *)&GetUserMediumGCSParameters, sizeof(_GetUserMediumGCSParameters));
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
        SendStringToGCS(PlatformName);
        break;

    case 22:
        SendStringToGCS(FirwareName);
        break;

    case 23:
        SendStringToGCS(FirmwareVersion);
        break;

    case 24:
        SendStringToGCS(CompilerVersion);
        break;

    case 25:
        SendStringToGCS(BuildDate);
        break;

    case 26:
        SendStringToGCS(BuildTime);
        break;

    case 27:
        if (!COMMAND_ARM_DISARM)
        {
            PREARM.UpdateGCSErrorText(PREARM.Checking());
        }
        break;

    case 28:
        if (!COMMAND_ARM_DISARM)
        {
            WATCHDOG.Reboot();
        }
        break;

    case 29:
        GCS.Default_RadioControl();
        break;
    }

#endif
}

void GCSClass::GCS_Request_Parameters()
{
    //ENVIA OS PARAMETROS FUNDAMENTAIS PARA O GCS
    GCSParameters.SendAttitudePitch = Constrain_16Bits(ATTITUDE.AngleOut[PITCH], -900, 900);
    if (CALIBRATION.AccelerometerZero[ROLL] > 1000)
    {
        GCSParameters.SendAttitudeRoll = 0xBB8; //INDICA PARA O GCS QUE A IMU NÃO ESTÁ CALIBRADA
    }
    else
    {
        GCSParameters.SendAttitudeRoll = Constrain_16Bits(ATTITUDE.AngleOut[ROLL], -900, 900);
    }
    if (I2C.CompassFound)
    {
        GCSParameters.SendAttitudeYaw = ATTITUDE.AngleOut[YAW];
    }
    else
    {
        GCSParameters.SendAttitudeYaw = GPS_Ground_Course / 10;
    }
    GCSParameters.DevicesOnBoard = GCS.GetDevicesActived();
    GCSParameters.SendThrottleValue = Throttle.Output;
    GCSParameters.SendYawValue = Yaw.Output;
    GCSParameters.SendPitchValue = Pitch.Output;
    GCSParameters.SendRollValue = Roll.Output;
    GCSParameters.SendAuxOneValue = AuxiliarOne.Output;
    GCSParameters.SendAuxTwoValue = AuxiliarTwo.Output;
    GCSParameters.SendAuxThreeValue = AuxiliarThree.Output;
    GCSParameters.SendAuxFourValue = AuxiliarFour.Output;
    GCSParameters.SendAuxFiveValue = AuxiliarFive.Output;
    GCSParameters.SendAuxSixValue = AuxiliarSix.Output;
    GCSParameters.SendAuxSevenValue = AuxiliarSeven.Output;
    GCSParameters.SendAuxEightValue = AuxiliarEight.Output;
    GCSParameters.SendGPSNumberOfSat = GPS_NumberOfSatellites;
    GCSParameters.SendGPSLatitude = GPS_Coordinates_Vector[0];
    GCSParameters.SendGPSLongitude = GPS_Coordinates_Vector[1];
    GCSParameters.SendHomePointLatitude = Stored_Coordinates_Home_Point[0];
    GCSParameters.SendHomePointLongitude = Stored_Coordinates_Home_Point[1];
    if (I2C.BarometerFound)
    {
        GCSParameters.SendBarometerValue = GetAltitudeForGCS();
    }
    else
    {
        GCSParameters.SendBarometerValue = (GPS_Altitude - GPS_Altitude_For_Plane) * 100;
    }
    GCSParameters.SendFailSafeState = Fail_Safe_Event;
    GCSParameters.SendBatteryVoltageValue = BATTERY.Voltage * 100;
    GCSParameters.SendBatteryPercentageValue = BATTERY.GetPercentage();
    GCSParameters.SendArmDisarmState = COMMAND_ARM_DISARM;
    GCSParameters.SendHDOPValue = GPS_HDOP;
    GCSParameters.SendCurrentValue = BATTERY.Total_Current;
    GCSParameters.SendWattsValue = BATTERY.GetWatts();
    GCSParameters.SendDeclinationValue = (int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100);
    GCSParameters.SendActualFlightMode = FlightMode;
    GCSParameters.SendFrameType = FrameType;
    GCSParameters.SendHomePointState = Home_Point;
    GCSParameters.SendTemperature = BaroTemperatureRaw / 100;
    GCSParameters.SendHomePointDistance = DistanceToHome;
    GCSParameters.SendCurrentInMah = BATTERY.Get_Current_In_Mah();
#ifndef MACHINE_CYCLE
    GCSParameters.SendCourseOverGround = GPS_Ground_Course;
#else
    GCSParameters.SendCourseOverGround = Loop_Integral_Time;
#endif
    GCSParameters.SendCrosstrack = Target_Bearing;
    GCSParameters.SendAccGForce = GetGForce() * 100;
    GCSParameters.SendAccImageBitMap = GetImageToGCS();
    GCSParameters.SendCompassRoll = IMU.CompassRead[ROLL];
    GCSParameters.SendCompassPitch = IMU.CompassRead[PITCH];
    GCSParameters.SendCompassYaw = IMU.CompassRead[YAW];
}

void GCSClass::GCS_Request_Parameters_Two()
{
    GCSParameters_Two.SendActualThrottleValue = DirectRadioControllRead[THROTTLE];
    GCSParameters_Two.SendActualYawValue = DirectRadioControllRead[YAW];
    GCSParameters_Two.SendActualPitchValue = DirectRadioControllRead[PITCH];
    GCSParameters_Two.SendActualRollValue = DirectRadioControllRead[ROLL];
    GCSParameters_Two.SendActualAuxOneValue = DirectRadioControllRead[AUX1];
    GCSParameters_Two.SendActualAuxTwoValue = DirectRadioControllRead[AUX2];
    GCSParameters_Two.SendActualAuxThreeValue = DirectRadioControllRead[AUX3];
    GCSParameters_Two.SendActualAuxFourValue = DirectRadioControllRead[AUX4];
    GCSParameters_Two.SendActualAuxFiveValue = DirectRadioControllRead[AUX5];
    GCSParameters_Two.SendActualAuxSixValue = DirectRadioControllRead[AUX6];
    GCSParameters_Two.SendActualAuxSevenValue = DirectRadioControllRead[AUX7];
    GCSParameters_Two.SendActualAuxEightValue = DirectRadioControllRead[AUX8];
    GCSParameters_Two.SendAttitudeThrottleValue = RCController[THROTTLE];
    GCSParameters_Two.SendAttitudeYawValue = CalcedRateTargetYaw;
    GCSParameters_Two.SendAttitudePitchValue = CalcedRateTargetPitch;
    GCSParameters_Two.SendAttitudeRollValue = CalcedRateTargetRoll;
    GCSParameters_Two.SendMemoryRamUsed = MEMORY.Check();
    GCSParameters_Two.SendMemoryRamUsedPercent = MEMORY.GetPercentageRAMUsed();
    GCSParameters_Two.SendAccXNotFiltered = IMU.AccelerometerReadNotFiltered[ROLL];
    GCSParameters_Two.SendAccYNotFiltered = IMU.AccelerometerReadNotFiltered[PITCH];
    GCSParameters_Two.SendAccZNotFiltered = IMU.AccelerometerReadNotFiltered[YAW];
    GCSParameters_Two.SendAccXFiltered = IMU.AccelerometerRead[ROLL];
    GCSParameters_Two.SendAccYFiltered = IMU.AccelerometerRead[PITCH];
    GCSParameters_Two.SendAccZFiltered = IMU.AccelerometerRead[YAW];
    GCSParameters_Two.SendGyroXNotFiltered = IMU.GyroscopeReadNotFiltered[ROLL];
    GCSParameters_Two.SendGyroYNotFiltered = IMU.GyroscopeReadNotFiltered[PITCH];
    GCSParameters_Two.SendGyroZNotFiltered = IMU.GyroscopeReadNotFiltered[YAW];
    GCSParameters_Two.SendGyroXFiltered = IMU.GyroscopeRead[ROLL];
    GCSParameters_Two.SendGyroYFiltered = IMU.GyroscopeRead[PITCH];
    GCSParameters_Two.SendGyroZFiltered = IMU.GyroscopeRead[YAW];
    GCSParameters_Two.SendGPSGroundSpeed = GPS_Ground_Speed;
    GCSParameters_Two.SendI2CError = ErrorI2C;
    GCSParameters_Two.SendAirSpeedValue = AirSpeedCalcedInCM;
    GCSParameters_Two.SendCPULoad = SystemLoadPercent;
}

void GCSClass::WayPoint_Request_Coordinates_Parameters()
{
    SendWayPointGCSCoordinates.SendLatitudeOne = STORAGEMANAGER.Read_32Bits(704);
    SendWayPointGCSCoordinates.SendLatitudeTwo = STORAGEMANAGER.Read_32Bits(708);
    SendWayPointGCSCoordinates.SendLatitudeThree = STORAGEMANAGER.Read_32Bits(712);
    SendWayPointGCSCoordinates.SendLatitudeFour = STORAGEMANAGER.Read_32Bits(716);
    SendWayPointGCSCoordinates.SendLatitudeFive = STORAGEMANAGER.Read_32Bits(720);
    SendWayPointGCSCoordinates.SendLatitudeSix = STORAGEMANAGER.Read_32Bits(724);
    SendWayPointGCSCoordinates.SendLatitudeSeven = STORAGEMANAGER.Read_32Bits(728);
    SendWayPointGCSCoordinates.SendLatitudeEight = STORAGEMANAGER.Read_32Bits(732);
    SendWayPointGCSCoordinates.SendLatitudeNine = STORAGEMANAGER.Read_32Bits(736);
    SendWayPointGCSCoordinates.SendLatitudeTen = STORAGEMANAGER.Read_32Bits(740);
    SendWayPointGCSCoordinates.SendLongitudeOne = STORAGEMANAGER.Read_32Bits(744);
    SendWayPointGCSCoordinates.SendLongitudeTwo = STORAGEMANAGER.Read_32Bits(748);
    SendWayPointGCSCoordinates.SendLongitudeThree = STORAGEMANAGER.Read_32Bits(752);
    SendWayPointGCSCoordinates.SendLongitudeFour = STORAGEMANAGER.Read_32Bits(756);
    SendWayPointGCSCoordinates.SendLongitudeFive = STORAGEMANAGER.Read_32Bits(760);
    SendWayPointGCSCoordinates.SendLongitudeSix = STORAGEMANAGER.Read_32Bits(764);
    SendWayPointGCSCoordinates.SendLongitudeSeven = STORAGEMANAGER.Read_32Bits(768);
    SendWayPointGCSCoordinates.SendLongitudeEight = STORAGEMANAGER.Read_32Bits(772);
    SendWayPointGCSCoordinates.SendLongitudeNine = STORAGEMANAGER.Read_32Bits(776);
    SendWayPointGCSCoordinates.SendLongitudeTen = STORAGEMANAGER.Read_32Bits(780);
}

void GCSClass::WayPoint_Request_Others_Parameters()
{
    SendWayPointGCSOthersParameters.SendAltitudeOne = STORAGEMANAGER.Read_8Bits(804);
    SendWayPointGCSOthersParameters.SendAltitudeTwo = STORAGEMANAGER.Read_8Bits(805);
    SendWayPointGCSOthersParameters.SendAltitudeThree = STORAGEMANAGER.Read_8Bits(806);
    SendWayPointGCSOthersParameters.SendAltitudeFour = STORAGEMANAGER.Read_8Bits(807);
    SendWayPointGCSOthersParameters.SendAltitudeFive = STORAGEMANAGER.Read_8Bits(808);
    SendWayPointGCSOthersParameters.SendAltitudeSix = STORAGEMANAGER.Read_8Bits(809);
    SendWayPointGCSOthersParameters.SendAltitudeSeven = STORAGEMANAGER.Read_8Bits(810);
    SendWayPointGCSOthersParameters.SendAltitudeEight = STORAGEMANAGER.Read_8Bits(811);
    SendWayPointGCSOthersParameters.SendAltitudeNine = STORAGEMANAGER.Read_8Bits(812);
    SendWayPointGCSOthersParameters.SendAltitudeTen = STORAGEMANAGER.Read_8Bits(813);
    SendWayPointGCSOthersParameters.SendFlightModeOne = STORAGEMANAGER.Read_8Bits(794);
    SendWayPointGCSOthersParameters.SendFlightModeTwo = STORAGEMANAGER.Read_8Bits(795);
    SendWayPointGCSOthersParameters.SendFlightModeThree = STORAGEMANAGER.Read_8Bits(796);
    SendWayPointGCSOthersParameters.SendFlightModeFour = STORAGEMANAGER.Read_8Bits(797);
    SendWayPointGCSOthersParameters.SendFlightModeFive = STORAGEMANAGER.Read_8Bits(798);
    SendWayPointGCSOthersParameters.SendFlightModeSix = STORAGEMANAGER.Read_8Bits(799);
    SendWayPointGCSOthersParameters.SendFlightModeSeven = STORAGEMANAGER.Read_8Bits(800);
    SendWayPointGCSOthersParameters.SendFlightModeEight = STORAGEMANAGER.Read_8Bits(801);
    SendWayPointGCSOthersParameters.SendFlightModeNine = STORAGEMANAGER.Read_8Bits(802);
    SendWayPointGCSOthersParameters.SendFlightModeTen = STORAGEMANAGER.Read_8Bits(803);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedOne = STORAGEMANAGER.Read_8Bits(784);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedTwo = STORAGEMANAGER.Read_8Bits(785);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedThree = STORAGEMANAGER.Read_8Bits(786);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedFour = STORAGEMANAGER.Read_8Bits(787);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedFive = STORAGEMANAGER.Read_8Bits(788);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedSix = STORAGEMANAGER.Read_8Bits(789);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedSeven = STORAGEMANAGER.Read_8Bits(790);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedEight = STORAGEMANAGER.Read_8Bits(791);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedNine = STORAGEMANAGER.Read_8Bits(792);
    SendWayPointGCSOthersParameters.SendGPSHoldTimedTen = STORAGEMANAGER.Read_8Bits(793);
}

uint8_t GCSClass::GetDevicesActived()
{
    const bool CompassDetect = I2C.CompassFound;
    const bool ParachuteDetect = ParachuteDetectTrigger > 0 ? true : false;
    const bool MatekDetect = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR) == 1 ? true : false;
    const bool PitotDetect = Get_AirSpeed_State();
    uint8_t CheckDevices = CompassDetect | ParachuteDetect << 1 | MatekDetect << 2 | PitotDetect << 3;
    return CheckDevices;
}

void GCSClass::Save_Basic_Configuration()
{
    if (GetUserBasicGCSParameters.GetFrameType != STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(FRAMETYPE_ADDR, GetUserBasicGCSParameters.GetFrameType);
    }

    if (GetUserBasicGCSParameters.GetReceiverType != STORAGEMANAGER.Read_8Bits(RECEIVER_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(RECEIVER_ADDR, GetUserBasicGCSParameters.GetReceiverType);
    }

    if (GetUserBasicGCSParameters.GetGimbalType != STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, GetUserBasicGCSParameters.GetGimbalType);
    }

    if (GetUserBasicGCSParameters.GetParachuteType != STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, GetUserBasicGCSParameters.GetParachuteType);
    }

    if (GetUserBasicGCSParameters.GetSPIType != STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(UART_NUMB_3_ADDR, GetUserBasicGCSParameters.GetSPIType);
    }

    if (GetUserBasicGCSParameters.GetUART_NUMB_2Type != STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, GetUserBasicGCSParameters.GetUART_NUMB_2Type);
    }

    if (GetUserBasicGCSParameters.GetCompassType != STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(COMPASS_TYPE_ADDR, GetUserBasicGCSParameters.GetCompassType);
    }

    if (GetUserBasicGCSParameters.GetCompassRotationType != STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, GetUserBasicGCSParameters.GetCompassRotationType);
    }

    if (GetUserBasicGCSParameters.GetRTHAltitudeType != STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, GetUserBasicGCSParameters.GetRTHAltitudeType);
    }

    if (GetUserBasicGCSParameters.GetMotorSpeedType != STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(MOTORSPEED_ADDR, GetUserBasicGCSParameters.GetMotorSpeedType);
    }

    if (GetUserBasicGCSParameters.GetAcroType != STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(STABLIZE_ADDR, GetUserBasicGCSParameters.GetAcroType);
    }

    if (GetUserBasicGCSParameters.GetAltitudeHoldType != STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, GetUserBasicGCSParameters.GetAltitudeHoldType);
    }

    if (GetUserBasicGCSParameters.GetPositionHoldType != STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, GetUserBasicGCSParameters.GetPositionHoldType);
    }

    if (GetUserBasicGCSParameters.GetInteligentOrientationControlType != STORAGEMANAGER.Read_8Bits(IOC_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(IOC_ADDR, GetUserBasicGCSParameters.GetInteligentOrientationControlType);
    }

    if (GetUserBasicGCSParameters.GetReturnToHomeType != STORAGEMANAGER.Read_8Bits(RTH_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(RTH_ADDR, GetUserBasicGCSParameters.GetReturnToHomeType);
    }

    if (GetUserBasicGCSParameters.GetAtackType != STORAGEMANAGER.Read_8Bits(ATACK_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(ATACK_ADDR, GetUserBasicGCSParameters.GetAtackType);
    }

    if (GetUserBasicGCSParameters.GetAutomaticFlipType != STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AUTOFLIP_ADDR, GetUserBasicGCSParameters.GetAutomaticFlipType);
    }

    if (GetUserBasicGCSParameters.GetAutomaticMissonType != STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AUTOMISSION_ADDR, GetUserBasicGCSParameters.GetAutomaticMissonType);
    }

    if (GetUserBasicGCSParameters.GetArmDisarmType != STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(ARMDISARM_ADDR, GetUserBasicGCSParameters.GetArmDisarmType);
    }

    if (GetUserBasicGCSParameters.GetAutoLandType != STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AUTOLAND_ADDR, GetUserBasicGCSParameters.GetAutoLandType);
    }

    if (GetUserBasicGCSParameters.GetSafeBtnState != STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(DISP_PASSIVES_ADDR, GetUserBasicGCSParameters.GetSafeBtnState);
    }

    if (GetUserBasicGCSParameters.GetAirSpeedState != STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AIRSPEED_TYPE_ADDR, GetUserBasicGCSParameters.GetAirSpeedState);
    }

    if (GetUserBasicGCSParameters.GetAccRollAdjust != STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADJUST_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADJUST_ADDR, GetUserBasicGCSParameters.GetAccRollAdjust);
    }

    if (GetUserBasicGCSParameters.GetAccPitchAdjust != STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADJUST_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADJUST_ADDR, GetUserBasicGCSParameters.GetAccPitchAdjust);
    }

    if (GetUserBasicGCSParameters.GetAccYawAdjust != STORAGEMANAGER.Read_16Bits(ACC_YAW_ADJUST_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(ACC_YAW_ADJUST_ADDR, GetUserBasicGCSParameters.GetAccYawAdjust);
    }

    if (GetUserBasicGCSParameters.GetThrottleMiddle != STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, GetUserBasicGCSParameters.GetThrottleMiddle);
    }

    if (GetUserBasicGCSParameters.GetThrottleExpo != STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, GetUserBasicGCSParameters.GetThrottleExpo);
    }

    if (GetUserBasicGCSParameters.GetRCRate != STORAGEMANAGER.Read_8Bits(RC_RATE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(RC_RATE_ADDR, GetUserBasicGCSParameters.GetRCRate);
    }

    if (GetUserBasicGCSParameters.GetRCExpo != STORAGEMANAGER.Read_8Bits(RC_EXPO_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(RC_EXPO_ADDR, GetUserBasicGCSParameters.GetRCExpo);
    }

    if (GetUserBasicGCSParameters.GetRollRate != STORAGEMANAGER.Read_8Bits(ROLL_RATE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(ROLL_RATE_ADDR, GetUserBasicGCSParameters.GetRollRate);
    }

    if (GetUserBasicGCSParameters.GetPitchRate != STORAGEMANAGER.Read_8Bits(PITCH_RATE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(PITCH_RATE_ADDR, GetUserBasicGCSParameters.GetPitchRate);
    }

    if (GetUserBasicGCSParameters.GetYawRate != STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, GetUserBasicGCSParameters.GetYawRate);
    }

    if (GetUserBasicGCSParameters.GetRCPulseMin != STORAGEMANAGER.Read_16Bits(RC_PULSE_MIN_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(RC_PULSE_MIN_ADDR, GetUserBasicGCSParameters.GetRCPulseMin);
    }

    if (GetUserBasicGCSParameters.GetRCPulseMax != STORAGEMANAGER.Read_16Bits(RC_PULSE_MAX_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(RC_PULSE_MAX_ADDR, GetUserBasicGCSParameters.GetRCPulseMax);
    }

    if (GetUserBasicGCSParameters.GetAHThrottleRover != STORAGEMANAGER.Read_16Bits(AH_THROTTLE_ROVER_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(AH_THROTTLE_ROVER_ADDR, GetUserBasicGCSParameters.GetAHThrottleRover);
    }

    if (GetUserBasicGCSParameters.GetAHDeadZone != STORAGEMANAGER.Read_8Bits(AH_DEADZONE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AH_DEADZONE_ADDR, GetUserBasicGCSParameters.GetAHDeadZone);
    }

    if (GetUserBasicGCSParameters.GetAHSafeAltitude != STORAGEMANAGER.Read_8Bits(AH_SAFE_ALT_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AH_SAFE_ALT_ADDR, GetUserBasicGCSParameters.GetAHSafeAltitude);
    }

    if (GetUserBasicGCSParameters.GetAHMinVelVertical != STORAGEMANAGER.Read_8Bits(AH_MIN_VEL_VERT_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(AH_MIN_VEL_VERT_ADDR, GetUserBasicGCSParameters.GetAHMinVelVertical);
    }
}

void GCSClass::Save_Medium_Configuration()
{
    if (GetUserMediumGCSParameters.GetTPAInPercent != STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(TPA_PERCENT_ADDR, GetUserMediumGCSParameters.GetTPAInPercent);
    }

    if (GetUserMediumGCSParameters.GetBreakPointValue != STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(BREAKPOINT_ADDR, GetUserMediumGCSParameters.GetBreakPointValue);
    }

    if (GetUserMediumGCSParameters.GetGyroLPF != STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(GYRO_LPF_ADDR, GetUserMediumGCSParameters.GetGyroLPF);
    }

    if (GetUserMediumGCSParameters.GetDerivativeLPF != STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(DERIVATIVE_LPF_ADDR, GetUserMediumGCSParameters.GetDerivativeLPF);
    }

    if (GetUserMediumGCSParameters.GetRCLPF != STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(RC_LPF_ADDR, GetUserMediumGCSParameters.GetRCLPF);
    }

    if (GetUserMediumGCSParameters.GetKalmanState != STORAGEMANAGER.Read_8Bits(KALMAN_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(KALMAN_ADDR, GetUserMediumGCSParameters.GetKalmanState);
    }

    if (GetUserMediumGCSParameters.GetBiquadAccLPF != STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(BI_ACC_LPF_ADDR, GetUserMediumGCSParameters.GetBiquadAccLPF);
    }

    if (GetUserMediumGCSParameters.GetBiquadGyroLPF != STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(BI_GYRO_LPF_ADDR, GetUserMediumGCSParameters.GetBiquadGyroLPF);
    }

    if (GetUserMediumGCSParameters.GetBiquadAccNotch != STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(BI_ACC_NOTCH_ADDR, GetUserMediumGCSParameters.GetBiquadAccNotch);
    }

    if (GetUserMediumGCSParameters.GetBiquadGyroNotch != STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(BI_GYRO_NOTCH_ADDR, GetUserMediumGCSParameters.GetBiquadGyroNotch);
    }

    if (GetUserMediumGCSParameters.GetMotorCompensationState != STORAGEMANAGER.Read_8Bits(MOTCOMP_STATE_ADDR))
    {
        STORAGEMANAGER.Write_8Bits(MOTCOMP_STATE_ADDR, GetUserMediumGCSParameters.GetMotorCompensationState);
    }

    PID[PITCH].ProportionalVector = GetUserMediumGCSParameters.GetProportionalPitch;
    PID[PITCH].IntegratorVector = GetUserMediumGCSParameters.GetIntegralPitch;
    PID[PITCH].DerivativeVector = GetUserMediumGCSParameters.GetDerivativePitch;
    PID[ROLL].ProportionalVector = GetUserMediumGCSParameters.GetProportionalRoll;
    PID[ROLL].IntegratorVector = GetUserMediumGCSParameters.GetIntegralRoll;
    PID[ROLL].DerivativeVector = GetUserMediumGCSParameters.GetDerivativeRoll;
    PID[YAW].ProportionalVector = GetUserMediumGCSParameters.GetProportionalYaw;
    PID[YAW].IntegratorVector = GetUserMediumGCSParameters.GetIntegralYaw;
    PID[YAW].DerivativeVector = GetUserMediumGCSParameters.GetDerivativeYaw;
    PID[PIDALTITUDE].ProportionalVector = GetUserMediumGCSParameters.GetProportionalAltitudeHold;
    PID[PIDGPSPOSITION].ProportionalVector = GetUserMediumGCSParameters.GetProportionalGPSHold;
    PID[PIDGPSPOSITION].IntegratorVector = GetUserMediumGCSParameters.GetIntegralGPSHold;

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != PID[PITCH].ProportionalVector)
    {
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, PID[PITCH].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != PID[PITCH].IntegratorVector)
    {
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, PID[PITCH].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != PID[PITCH].DerivativeVector)
    {
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, PID[PITCH].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != PID[ROLL].ProportionalVector)
    {
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, PID[ROLL].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != PID[ROLL].IntegratorVector)
    {
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, PID[ROLL].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != PID[ROLL].DerivativeVector)
    {
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, PID[ROLL].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != PID[YAW].ProportionalVector)
    {
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, PID[YAW].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != PID[YAW].IntegratorVector)
    {
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, PID[YAW].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != PID[YAW].DerivativeVector)
    {
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, PID[YAW].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != PID[PIDALTITUDE].ProportionalVector)
    {
        STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, PID[PIDALTITUDE].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != PID[PIDGPSPOSITION].ProportionalVector)
    {
        STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, PID[PIDGPSPOSITION].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != PID[PIDGPSPOSITION].IntegratorVector)
    {
        STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, PID[PIDGPSPOSITION].IntegratorVector);
    }

    if (GetUserMediumGCSParameters.GetServosLPF != STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR))
    {
        STORAGEMANAGER.Write_16Bits(SERVOS_LPF_ADDR, GetUserMediumGCSParameters.GetServosLPF);
    }

    //ATUALIZA OS PARAMETROS
    GCS.UpdatePID = false;
}

void GCSClass::Default_Basic_Configuration()
{
    //LIMPA TODAS AS CONFIGURAÇÕES SALVAS
    STORAGEMANAGER.Write_8Bits(IOC_ADDR, 0);               //LIMPA A CONFIGURAÇÃO DO MODO IOC
    STORAGEMANAGER.Write_8Bits(ALT_HOLD_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO ALTITUDE-HOLD
    STORAGEMANAGER.Write_8Bits(GPS_HOLD_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODO GPS-HOLD
    STORAGEMANAGER.Write_8Bits(RTH_ADDR, 0);               //LIMPA A CONFIGURAÇÃO DO MODO RTH
    STORAGEMANAGER.Write_8Bits(PARACHUTE_ADDR, 0);         //LIMPA A CONFIGURAÇÃO DO PARACHUTE
    STORAGEMANAGER.Write_8Bits(GIMBAL_ADDR, 0);            //LIMPA A CONFIGURAÇÃO DO CONTROLE DO GIMBAL
    STORAGEMANAGER.Write_8Bits(FRAMETYPE_ADDR, 0);         //LIMPA A CONFIGURAÇÃO DO TIPO DE FRAME
    STORAGEMANAGER.Write_8Bits(RECEIVER_ADDR, 0);          //LIMPA A CONFIGURAÇÃO DO MODULO RECEPTOR PPM
    STORAGEMANAGER.Write_8Bits(MOTORSPEED_ADDR, 0);        //LIMPA A CONFIGURAÇÃO DO MOTOR SPEED
    STORAGEMANAGER.Write_8Bits(UART_NUMB_2_ADDR, 0);       //LIMPA A CONFIGURAÇÃO DA UART_NUMB_2
    STORAGEMANAGER.Write_8Bits(COMPASS_ROTATION_ADDR, 0);  //LIMPA A CONFIGURAÇÃO DE ROTAÇÃO DO COMPASS
    STORAGEMANAGER.Write_8Bits(COMPASS_TYPE_ADDR, 0);      //LIMPA A CONFIGURAÇÃO DO MODELO DO COMPASS
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

void GCSClass::Default_RadioControl()
{
    STORAGEMANAGER.Write_8Bits(THROTTLE_MIDDLE_ADDR, 50);      //VOLTA A CONFIGURAÇÃO DE FABRICA DO THROTTLE MÉDIO
    STORAGEMANAGER.Write_8Bits(THROTTLE_EXPO_ADDR, 35);        //VOLTA A CONFIGURAÇÃO DE FABRICA DO EXPO DO THROTTLE
    STORAGEMANAGER.Write_8Bits(RC_EXPO_ADDR, 60);              //VOLTA A CONFIGURAÇÃO DE FABRICA DO EXPO DO YAW,PITCH E ROLL
    STORAGEMANAGER.Write_8Bits(RC_RATE_ADDR, 70);              //VOLTA A CONFIGURAÇÃO DE FABRICA DO RATE DO ROLL E DO PITCH DO PID
    STORAGEMANAGER.Write_8Bits(ROLL_RATE_ADDR, 0);             //VOLTA A CONFIGURAÇÃO DE FABRICA DO RATE DO ROLL DO PID DINAMICO
    STORAGEMANAGER.Write_8Bits(PITCH_RATE_ADDR, 0);            //VOLTA A CONFIGURAÇÃO DE FABRICA DO RATE DO PITCH DO PID DINAMICO
    STORAGEMANAGER.Write_8Bits(YAW_RATE_ADDR, 0);              //VOLTA A CONFIGURAÇÃO DE FABRICA DO RATE DO YAW DO PID DINAMICO
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MIN_ADDR, 1100);      //VOLTA A CONFIGURAÇÃO DE FABRICA DO PULSO MINIMO DOS CANAIS RC
    STORAGEMANAGER.Write_16Bits(RC_PULSE_MAX_ADDR, 1900);      //VOLTA A CONFIGURAÇÃO DE FABRICA DO PULSO MAXIMO DOS CANAIS RC
    STORAGEMANAGER.Write_16Bits(AH_THROTTLE_ROVER_ADDR, 1500); //VOLTA A CONFIGURAÇÃO DE FABRICA DO THROTTLE ROVER DO ALT-HOLD
    STORAGEMANAGER.Write_8Bits(AH_DEADZONE_ADDR, 70);          //VOLTA A CONFIGURAÇÃO DE FABRICA DO DEADZONE DO ALT-HOLD
    STORAGEMANAGER.Write_8Bits(AH_SAFE_ALT_ADDR, 5);           //VOLTA A CONFIGURAÇÃO DE FABRICA DO SAFE-ALT DO ALT-HOLD
    STORAGEMANAGER.Write_8Bits(AH_MIN_VEL_VERT_ADDR, 50);      //VOLTA A CONFIGURAÇÃO DE FABRICA DA VELOCIDADE VERTICAL MINIMA DO ALT-HOLD
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
    //VOLTA TODO O PID PARA O PADRÃO DE FABRICA
    //PITCH
    STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 35);
    STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 25);
    STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 26);
    //ROLL
    STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 35);
    STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 25);
    STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 26);
    //YAW
    STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 69);
    STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 50);
    STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);
    //ALTITUDE-HOLD
    STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, 50);
    //GPS-HOLD
    STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, 100);
    STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, 90);
    //ATUALIZA OS PARAMETROS
    GCS.UpdatePID = false;
}

void GCSClass::Default_All_Configs()
{
    Default_Basic_Configuration();
    Default_Medium_Configuration();
    Default_RadioControl();
}

void GCSClass::UpdateParametersToGCS()
{
    //ENVIA OS PARAMETROS BASICOS AJUSTAVEIS PELO USUARIO
    SendUserBasicGCSParameters.SendFrameType = FrameType;
    SendUserBasicGCSParameters.SendReceiverType = STORAGEMANAGER.Read_8Bits(RECEIVER_ADDR);
    SendUserBasicGCSParameters.SendGimbalType = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);
    SendUserBasicGCSParameters.SendParachuteType = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR);
    SendUserBasicGCSParameters.SendSPIType = STORAGEMANAGER.Read_8Bits(UART_NUMB_3_ADDR);
    SendUserBasicGCSParameters.SendUART_NUMB_2Type = STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR);
    SendUserBasicGCSParameters.SendCompassType = STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR);
    SendUserBasicGCSParameters.SendCompassRotationType = STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR);
    SendUserBasicGCSParameters.SendRTHAltitudeType = STORAGEMANAGER.Read_8Bits(RTH_ALTITUDE_ADDR);
    SendUserBasicGCSParameters.SendMotorSpeedType = STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR);
    SendUserBasicGCSParameters.SendAcroType = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);
    SendUserBasicGCSParameters.SendAltitudeHoldType = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR);
    SendUserBasicGCSParameters.SendPositionHoldType = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);
    SendUserBasicGCSParameters.SendInteligentOrientationControlType = STORAGEMANAGER.Read_8Bits(IOC_ADDR);
    SendUserBasicGCSParameters.SendReturnToHomeType = STORAGEMANAGER.Read_8Bits(RTH_ADDR);
    SendUserBasicGCSParameters.SendAtackType = STORAGEMANAGER.Read_8Bits(ATACK_ADDR);
    SendUserBasicGCSParameters.SendAutomaticFlipType = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);
    SendUserBasicGCSParameters.SendAutomaticMissonType = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR);
    SendUserBasicGCSParameters.SendArmDisarmType = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);
    SendUserBasicGCSParameters.SendAutoLandType = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);
    SendUserBasicGCSParameters.SendSafeBtnState = STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR);
    SendUserBasicGCSParameters.SendAirSpeedState = STORAGEMANAGER.Read_8Bits(AIRSPEED_TYPE_ADDR);
    SendUserBasicGCSParameters.SendAccRollAdjust = STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADJUST_ADDR);
    SendUserBasicGCSParameters.SendAccPitchAdjust = STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADJUST_ADDR);
    SendUserBasicGCSParameters.SendAccYawAdjust = STORAGEMANAGER.Read_16Bits(ACC_YAW_ADJUST_ADDR);
    SendUserBasicGCSParameters.SendThrottleMiddle = STORAGEMANAGER.Read_8Bits(THROTTLE_MIDDLE_ADDR);
    SendUserBasicGCSParameters.SendThrottleExpo = STORAGEMANAGER.Read_8Bits(THROTTLE_EXPO_ADDR);
    SendUserBasicGCSParameters.SendRCRate = STORAGEMANAGER.Read_8Bits(RC_RATE_ADDR);
    SendUserBasicGCSParameters.SendRCExpo = STORAGEMANAGER.Read_8Bits(RC_EXPO_ADDR);
    SendUserBasicGCSParameters.SendRollRate = STORAGEMANAGER.Read_8Bits(ROLL_RATE_ADDR);
    SendUserBasicGCSParameters.SendPitchRate = STORAGEMANAGER.Read_8Bits(PITCH_RATE_ADDR);
    SendUserBasicGCSParameters.SendYawRate = STORAGEMANAGER.Read_8Bits(YAW_RATE_ADDR);
    SendUserBasicGCSParameters.SendRCPulseMin = STORAGEMANAGER.Read_16Bits(RC_PULSE_MIN_ADDR);
    SendUserBasicGCSParameters.SendRCPulseMax = STORAGEMANAGER.Read_16Bits(RC_PULSE_MAX_ADDR);
    SendUserBasicGCSParameters.SendAHThrottleRover = STORAGEMANAGER.Read_16Bits(AH_THROTTLE_ROVER_ADDR);
    SendUserBasicGCSParameters.SendAHDeadZone = STORAGEMANAGER.Read_8Bits(AH_DEADZONE_ADDR);
    SendUserBasicGCSParameters.SendAHSafeAltitude = STORAGEMANAGER.Read_8Bits(AH_SAFE_ALT_ADDR);
    SendUserBasicGCSParameters.SendAHMinVelVertical = STORAGEMANAGER.Read_8Bits(AH_MIN_VEL_VERT_ADDR);

    //ENVIA OS PARAMETROS MEDIOS AJUSTAVEIS PELO USUARIO
    SendUserMediumGCSParameters.SendTPAInPercent = STORAGEMANAGER.Read_8Bits(TPA_PERCENT_ADDR);
    SendUserMediumGCSParameters.SendBreakPointValue = STORAGEMANAGER.Read_16Bits(BREAKPOINT_ADDR);
    SendUserMediumGCSParameters.SendGyroLPF = STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR);
    SendUserMediumGCSParameters.SendDerivativeLPF = STORAGEMANAGER.Read_16Bits(DERIVATIVE_LPF_ADDR);
    SendUserMediumGCSParameters.SendRCLPF = STORAGEMANAGER.Read_16Bits(RC_LPF_ADDR);
    SendUserMediumGCSParameters.SendKalmanState = STORAGEMANAGER.Read_8Bits(KALMAN_ADDR);
    SendUserMediumGCSParameters.SendBiQuadAccLPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
    SendUserMediumGCSParameters.SendBiQuadGyroLPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
    SendUserMediumGCSParameters.SendBiQuadAccNotch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
    SendUserMediumGCSParameters.SendBiQuadGyroNotch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
    SendUserMediumGCSParameters.SendMotorCompensationState = STORAGEMANAGER.Read_8Bits(MOTCOMP_STATE_ADDR);
    SendUserMediumGCSParameters.SendProportionalPitch = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    SendUserMediumGCSParameters.SendIntegralPitch = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    SendUserMediumGCSParameters.SendDerivativePitch = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    SendUserMediumGCSParameters.SendProportionalRoll = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    SendUserMediumGCSParameters.SendIntegralRoll = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    SendUserMediumGCSParameters.SendDerivativeRoll = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    SendUserMediumGCSParameters.SendProportionalYaw = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    SendUserMediumGCSParameters.SendIntegralYaw = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    SendUserMediumGCSParameters.SendDerivativeYaw = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    SendUserMediumGCSParameters.SendProportionalAltitudeHold = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    SendUserMediumGCSParameters.SendProportionalGPSHold = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    SendUserMediumGCSParameters.SendIntegralGPSHold = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    SendUserMediumGCSParameters.SendServosLPF = STORAGEMANAGER.Read_16Bits(SERVOS_LPF_ADDR);
}