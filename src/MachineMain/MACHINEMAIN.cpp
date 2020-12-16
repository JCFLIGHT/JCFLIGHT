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
    UART2Mode_Initialization();
    FASTSERIAL.Initialization();
    RGB.Initialization();
    SPEEDMOTORS.LoadEEPROM();
    FullParamsListInitialization();
    CurvesRC_Update();
    CurvesRC_Initialization();
    TPA_Initialization();
    PORTB |= 1 << 4;    //PINO DIGITAL 10
    PORTB &= ~(1 << 5); //PINO DIGITAL 11
    PORTB &= ~(1 << 6); //PINO DIGITAL 12
    //CONFIGURA OS REGISTRADORES PARA A ROTINA DE INTERRUPÇÃO DA CAPTURA DO SINAL PPM
    ConfigurePPMRegisters();
    //CHECA SE A IMU ESTÁ CALIBRADA E CARREGA OS VALORES DE CALIBRAÇÃO DA MESMA E DO COMPASS
    CheckAndUpdateIMUCalibration();
    //CARREGA OS VALORES DE PID
    LoadPID();
    //CARREGA OS PARAMETROS DO RADIO CONTROLE
    CurvesRC_Update();
    //INICIALIZA OS DISPOSITIVOS I2C
    AllI2CInitialization();
    //CARREGA OS PARAMETROS DO GPS
    LoadGPSParameters();
    //INICIALIZA O KALMAN
    KALMAN.Init();
    IMU_Filters_Initialization();
    DerivativeLPF_Initialization();
    //CONFIGURA OS 12 CANAIS
    RCCONFIG.Init();
    //CALIBRAÇÃO DOS ESC'S
    ESC.Calibration();
    //AJUSTA O RATE DOS SERVOS
    Trim_Servo_Initializate();
    //CARREGA TODOS OS PARAMETROS DO MODO WAYPOINT
    WayPoint_Initialization();
    //RECOLHE AS PRIMEIRAS AMOSTRAS DO AIR-SPEED PARA CALIBRAR
    AirSpeed_Initialization();
    //CALIBRA O GYRO
    CalibratingGyroscope = 512;
    //NORMALIZA O BUZZER PARA OPERAÇÃO NORMAL
    ESC.CalibratingEscBeep = -10;
    //INICIALIZA O AHRS
    AHRS_Initialization();
    //INICIALIZA O BOTÃO DE SEGURANÇA
    SAFETYBUTTON.Initialization();
    PORTB &= ~(1 << 4); //PINO DIGITAL 10
    PORTB &= ~(1 << 5); //PINO DIGITAL 11
    PORTB &= ~(1 << 6); //PINO DIGITAL 12
    for (uint8_t LedCount = 0; LedCount < 6; LedCount++)
    {
        //LED TESTE
        PORTB |= 1 << 4;    //PINO DIGITAL 10
        PORTB &= ~(1 << 5); //PINO DIGITAL 11
        PORTB &= ~(1 << 6); //PINO DIGITAL 12
        AVRTIME.SchedulerSleep(133);
        PORTB &= ~(1 << 4); //PINO DIGITAL 10
        PORTB |= 1 << 5;    //PINO DIGITAL 11
        PORTB &= ~(1 << 6); //PINO DIGITAL 12
        AVRTIME.SchedulerSleep(133);
        PORTB &= ~(1 << 4); //PINO DIGITAL 10
        PORTB &= ~(1 << 5); //PINO DIGITAL 11
        PORTB |= 1 << 6;    //PINO DIGITAL 12
        AVRTIME.SchedulerSleep(133);
        //133 * 3 * 5 = 1.995 SEGUNDO
    }
    PORTB &= ~(1 << 4); //PINO DIGITAL 10
    PORTB &= ~(1 << 5); //PINO DIGITAL 11
    PORTB &= ~(1 << 6); //PINO DIGITAL 12
    //DECLARA OS PINOS GERAIS DE SAÍDA
    ConfigureRegisters();
}

void MachineRun()
{
    Slow_Loop();
    Medium_Loop();
    Fast_Medium_Loop();
    Fast_Loop();
    Integral_Loop();
}