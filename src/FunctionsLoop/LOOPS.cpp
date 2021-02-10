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
        COMPASS.Constant_Read();
        PushWayPointParameters();
        UpdateValuesOfPID();
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
        Servo_Rate_Update();
        Auto_Throttle_Flight_Mode();
        BATTERY.Read_Voltage();
        BATTERY.Read_Current();
        PrintlnParameters();
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
        Auto_Launch_Update();
        CalculateAccelerationXYZ();
        INS_Calculate_AccelerationZ();
        CalculateXY_INS();
        AirSpeed_Update();
        Apply_Controll_For_Throttle();
        GPS_Orientation_Update();
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
        PID_Dynamic();
        PID_Update();
        ServoAutoTrimRun();
        AIR_PLANE.Servo_Rate_Adjust_And_Apply_LPF();
        PID_MixMotors();
}