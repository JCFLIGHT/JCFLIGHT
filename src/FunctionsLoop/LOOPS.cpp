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
#ifdef __AVR_ATmega2560__
        static Scheduler_Struct Slow_Loop;
        if (SchedulerTimer(&Slow_Loop, SCHEDULER_PERIOD_HZ(10, "Hz")))
        {
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(SLOW_LOOP);
#endif

                Pre_Arm();
                CurvesRC_SetValues();
                AUXFLIGHT.LoadEEPROM();
                COMPASS.Constant_Read();
                RTH_Altitude_EEPROM();
                IMU_Filters_Update();
                PID_DerivativeLPF_Update();
                UpdateValuesOfPID();
                UpdateServosDirection();
                GCS.UpdateParametersToGCS();
                PushWayPointParameters();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif

#ifdef __AVR_ATmega2560__
        }
#endif
}

void Medium_Loop()
{
#ifdef __AVR_ATmega2560__
        static Scheduler_Struct Medium_Loop;
        if (SchedulerTimer(&Medium_Loop, SCHEDULER_PERIOD_HZ(50, "Hz")))
        {
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(MEDIUM_LOOP);
#endif

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
                Manual_Trim_Servo_Update();
                Auto_Throttle_Flight_Mode(SetFlightModes[ALTITUDE_HOLD_MODE]);

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif

#ifdef __AVR_ATmega2560__
        }
#endif
}

void Fast_Medium_Loop()
{
#ifdef __AVR_ATmega2560__
        static Scheduler_Struct Fast_Medium_Loop;
        if (SchedulerTimer(&Fast_Medium_Loop, SCHEDULER_PERIOD_HZ(100, "Hz")))
        {
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(FAST_MEDIUM_LOOP);
#endif

                BEEPER.Run();
                BATTERY.Read_Voltage();
                BATTERY.Read_Current();
                Pre_Arm_Leds();
                GimbalControll();
                CrashCheck();
                PARACHUTE.Manual_Detect_Channel();
                PARACHUTE.Manual_Do_Now();
                FlipModeRun();
                WayPointRun();
                GCS.Serial_Parse_Protocol();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif

#ifdef __AVR_ATmega2560__
        }
#endif
}

void Fast_Loop()
{
#ifdef __AVR_ATmega2560__
        static Scheduler_Struct Fast_Loop;
        if (SchedulerTimer(&Fast_Loop, SCHEDULER_PERIOD_HZ(400, "Hz")))
        {
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(FAST_LOOP);
#endif

                Update_PrecisionLand();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif

#ifdef __AVR_ATmega2560__
        }
#endif
}

void Integral_Loop()
{
#ifdef __AVR_ATmega2560__
#ifdef THIS_LOOP_FREQUENCY
        static Scheduler_Struct Integral_Loop;
        if (SchedulerTimer(&Integral_Loop, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz ou KHz")))
        {
#endif
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(TOTAL_LOOP);
#endif

                RGB.Update();
                SAFETYBUTTON.UpdateRoutine();
                SBUS_Update();
                IBUS_Update();
                Acc_ReadBufferData();
                Gyro_ReadBufferData();
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
                Update_Loop_Time();
                PID_Update();
                PID_Reset_Integral_Accumulators();
                PID_MixMotors();
                Servo_Rate_Adjust();
                ApplyPWMInAllComponents();
                Switch_Flag();
                BATTERY.Calculate_Total_Mah();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif

#ifdef __AVR_ATmega2560__
#ifdef THIS_LOOP_FREQUENCY
        }
#endif
#endif
}