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

#include "LOOPS.h"
#include "Common/COMMON.h"

void Slow_Loop()
{
        Pre_Arm();
        CurvesRC_SetValues();
        CHECKSUM.UpdateChannelsReverse();
        AUXFLIGHT.LoadEEPROM();
        COMPASS.Constant_Read();
        RTH_Altitude_EEPROM();
        IMU_Filters_Update();
        PID_DerivativeLPF_Update();
        UpdateValuesOfPID();
        AIR_PLANE.UpdateServosMinAndMax();
        AIR_PLANE.UpdateServosMiddlePoint();
        AIR_PLANE.UpdateServosDirection();
        AltitudeHold_Update_Params();
        GCS.UpdateParametersToGCS();
        PushWayPointParameters();
}

void Medium_Loop()
{
        DecodeAllReceiverChannels();
        RCCONFIG.Set_Pulse();
        RCCONFIG.Update_Channels();
        Desarm_LowThrottle();
        FailSafeCheck();
        RCSticks_Update();
        Barometer_Update();
        GPS_Process_FlightModes();
        AUXFLIGHT.SelectMode();
        AUXFLIGHT.FlightModesAuxSelect();
        FlightModesUpdate();
        PrintlnParameters();
        Servo_Rate_Update();
        Auto_Throttle_Flight_Mode(SetFlightModes[ALTITUDE_HOLD_MODE]);
        BATTERY.Read_Voltage();
        BATTERY.Read_Current();
}

void Fast_Medium_Loop()
{
        BEEPER.Run();
        Pre_Arm_Leds();
        GimbalControll();
        CrashCheck();
        PARACHUTE.Manual_Detect_Channel();
        PARACHUTE.Manual_Do_Now();
        FlipModeRun();
        WayPointRun();
        GCS.Serial_Parse_Protocol();

#ifdef __AVR_ATmega2560__
        Fast_Loop();
#endif
}

void Fast_Loop()
{
        Update_PrecisionLand();

#ifdef __AVR_ATmega2560__
        Super_Fast_Loop();
#endif
}

void Super_Fast_Loop()
{
        RGB.Update();
        SAFETYBUTTON.UpdateRoutine();
        SBUS_Update();
        IBUS_Update();
        GPS_Serial_Read();
        AHRS_Update();
        DynamicPID();
        Auto_Launch_Update();
        CalculateAccelerationXYZ();
        INS_Calculate_AccelerationZ();
        CalculateXY_INS();
        AirSpeed_Update();
        Apply_Controll_For_Throttle();
        GPS_Orientation_Update();
        AIR_PLANE.Servo_Rate_Adjust_And_Apply_LPF();
        ApplyPWMInAllComponents();
        Switch_Flag();
        BATTERY.Calculate_Total_Mah();

#ifdef __AVR_ATmega2560__
        Integral_Loop();
#endif
}

void Integral_Loop()
{
        Acc_ReadBufferData();
        Gyro_ReadBufferData();
        Update_Loop_Time();
        ServoAutoTrimRun();
        PID_Update();
        PID_Reset_Integral_Accumulators();
        PID_MixMotors();
}