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

void MachineInit()
{
    RGB.Initialization();
    //ATIVA O LED VERMELHO
    RED_LED_ON;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    CheckFirstLinkOrganizeEEPROM();
    UART2Mode_Initialization();
    FASTSERIAL.Initialization();
    AUXFLIGHT.LoadEEPROM();
    SPEEDMOTORS.LoadEEPROM();
    FullParamsListInitialization();
    //CARREGA OS PARAMETROS DO RADIO CONTROLE
    CurvesRC_SetValues();
    CurvesRC_CalculeValue();
    TPA_Initialization();
    //CARREGA OS PARAMETROS DO ALTITUDE-HOLD
    AltitudeHold_Update_Params();
    //CONFIGURA OS REGISTRADORES PARA A ROTINA DE INTERRUPÇÃO DA CAPTURA DO SINAL PPM
    ConfigurePPMRegisters();
    //CHECA SE A IMU ESTÁ CALIBRADA E CARREGA OS VALORES DE CALIBRAÇÃO DA MESMA E DO COMPASS
    CheckAndUpdateIMUCalibration();
    //CARREGA OS VALORES DE PID
    LoadPID();
    //INICIALIZA OS DISPOSITIVOS I2C
    All_I2C_Initialization();
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
        SCHEDULER.Sleep(133);
        RED_LED_OFF;
        GREEN_LED_ON;
        BLUE_LED_OFF;
        SCHEDULER.Sleep(133);
        RED_LED_OFF;
        GREEN_LED_OFF;
        BLUE_LED_ON;
        SCHEDULER.Sleep(133);
        //133 * 3 * 5 = 1.995 SEGUNDO
    }
    //DESATIVA TODOS OS LEDS
    RED_LED_OFF;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    //DECLARA OS PINOS GERAIS DE SAÍDA
    ConfigureRegisters();
    //INICIA O SISTEMA DE TASKS
    TaskSystemInitialization();
}

void MachineRun()
{
    TaskSystemRun();
}