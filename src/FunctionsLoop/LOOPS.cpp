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
        STICKS.Pre_Arm();
        COMPASS.Constant_Read();
        PushWayPointParameters();
        UpdateValuesOfPID();
}

void Medium_Loop()
{
        DECODE.Update();
        RCCONFIG.Set_Pulse();
        RCCONFIG.Update_Channels();
        DESARMLOWTHROTTLE.Update();
        FailSafeCheck();
        STICKS.Update();
        Barometer_Update();
        GPS_Serial_Read();
        GPS_Process_FlightModes();
        AUXFLIGHT.SelectMode();
        AUXFLIGHT.FlightModesAuxSelect();
        FlightModesUpdate();
        AirSpeed_Update_Auto_Throttle();
        BATTERY.Read_Voltage();
        BATTERY.Read_Current();
        PRINTF.ParamsToConsole();
}

void Fast_Medium_Loop()
{
        BEEPER.Run();
        STICKS.Pre_Arm_Leds();
        Gimbal_Controll();
        CrashCheck();
        PARACHUTE.Manual_Detect_Channel();
        PARACHUTE.Manual_Do_Now();
        GCS.Serial_Parse_Protocol();
}

void Fast_Loop()
{
        Update_PrecisionLand();
}

void Super_Fast_Loop()
{
        RGB.Update();
        SAFETYBUTTON.UpdateRoutine();
        SBUS_Update();
        IBUS_Update();
        EarthFrame_Calculate_AccelerationXYZ();
        INERTIALNAVIGATION.Calculate_AccelerationXY();
        INERTIALNAVIGATION.Calculate_AccelerationZ();
        AirSpeed_Update();
        Switch_Flag();
        BATTERY.Calculate_Total_Mah();
}

void Integral_Loop()
{
        int32_t CycleTimeUs = GetTaskDeltaTime(TASK_INTEGRAL_LOOP);
        const float DeltaTime = (float)CycleTimeUs * 0.000001f;

#ifdef __AVR_ATmega2560__
        Fast_Medium_Loop();
        Fast_Loop();
        Super_Fast_Loop();
#endif

        Acc_ReadBufferData();
        Gyro_ReadBufferData();
        AHRS.Update(DeltaTime);
        PID_Dynamic();
        GPS_Orientation_Update();
        FlipModeRun();
        WayPointRun();
        Auto_Launch_Update();
        AirSpeed_Apply_Auto_Throttle_Control();
        PIDXYZ.Update(CycleTimeUs);
        AIR_PLANE.Servo_Rate_Adjust_And_Apply_LPF();
        ServoAutoTrimRun();
        ApplyMixingForMotorsAndServos();
        ApplyPWMControlForMotorsAndServos();
}