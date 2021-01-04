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

#include "MACHINEMAIN.h"
#include "TaskSystem/TASKSYSTEM.h"

#ifdef __AVR_ATmega2560__

void MachineInit()

#elif defined __arm__ || defined ESP32

void setup()

#endif
{
    UART2Mode_Initialization();
    FASTSERIAL.Initialization();
    RGB.Initialization();
    AUXFLIGHT.LoadEEPROM();
    SPEEDMOTORS.LoadEEPROM();
    FullParamsListInitialization();
    //CARREGA OS PARAMETROS DO RADIO CONTROLE
    CurvesRC_SetValues();
    CurvesRC_CalculeValue();
    TPA_Initialization();
    //ATIVA O LED VERMELHO
    RED_LED_ON;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    //CONFIGURA OS REGISTRADORES PARA A ROTINA DE INTERRUPÇÃO DA CAPTURA DO SINAL PPM
    ConfigurePPMRegisters();
    //CHECA SE A IMU ESTÁ CALIBRADA E CARREGA OS VALORES DE CALIBRAÇÃO DA MESMA E DO COMPASS
    CheckAndUpdateIMUCalibration();
    //CARREGA OS VALORES DE PID
    LoadPID();
    //INICIALIZA OS DISPOSITIVOS I2C
    AllI2CInitialization();
    //CARREGA OS PARAMETROS DO GPS
    LoadGPSParameters();
    //INICIALIZA OS FILTROS
    KALMAN.Init();
    IMU_Filters_Initialization();
    //CONFIGURA OS 12 CANAIS
    RCCONFIG.Init();
    //CALIBRAÇÃO DOS ESC'S
    ESC.Calibration();
    //AJUSTA O RATE DOS SERVOS
    Manual_Trim_Servo_Initializate();
    //CARREGA TODOS OS PARAMETROS DO MODO WAYPOINT
    WayPoint_Initialization();
    //RECOLHE AS PRIMEIRAS AMOSTRAS DO AIR-SPEED PARA CALIBRAR
    AirSpeed_Initialization();
    //CALIBRA O GYRO
    CalibratingGyroscope = 512;
    //INICIA O BUZZER EM OPERAÇÃO NORMAL
    ESC.BeeperMode = NORMAL_OPERATION_MODE;
    //INICIALIZA O AHRS
    AHRS_Initialization();
    //INICIALIZA O BOTÃO DE SEGURANÇA
    SAFETYBUTTON.Initialization();
    //DESATIVA TODOS OS LEDS
    RED_LED_OFF;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    for (uint8_t LedCount = 0; LedCount < 6; LedCount++)
    {
        RED_LED_ON;
        GREEN_LED_OFF;
        BLUE_LED_OFF;
        SCHEDULERTIME.Sleep(133);
        RED_LED_OFF;
        GREEN_LED_ON;
        BLUE_LED_OFF;
        SCHEDULERTIME.Sleep(133);
        RED_LED_OFF;
        GREEN_LED_OFF;
        BLUE_LED_ON;
        SCHEDULERTIME.Sleep(133);
        //133 * 3 * 5 = 1.995 SEGUNDO
    }
    //DESATIVA TODOS OS LEDS
    RED_LED_OFF;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    //DECLARA OS PINOS GERAIS DE SAÍDA
    ConfigureRegisters();
#if defined(__arm__) || defined(ESP32)
    TaskSystemInitialization();
#endif
}

#ifdef __AVR_ATmega2560__

void MachineRun()

#elif defined __arm__ || defined ESP32

void loop()

#endif
{
#ifdef __AVR_ATmega2560__

    Slow_Loop();
    Medium_Loop();
    Fast_Medium_Loop();
    Fast_Loop();
    Super_Fast_Loop();
    Integral_Loop();

#elif defined __arm__ || defined ESP32

    TaskSystemRun();

#endif
}