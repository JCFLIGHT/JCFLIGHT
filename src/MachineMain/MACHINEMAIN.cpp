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
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

void MachineInit()
{
    //OBTÉM O VALOR INICIAL DE MILLIS PARA CALCULAR O TEMPO DE INICIALIZAÇÃO
    SetInitialTimeToInitTheMachine(&MachineInitTimeNow);
    //INICIALIZA A SERIAL
    FASTSERIAL.Initialization();
    PRINTF.Initialization();
    //INICIALIZA O LED RGB
    RGB.Initialization();
    //ATIVA O LED VERMELHO
    RED_LED_ON;
    GREEN_LED_OFF;
    BLUE_LED_OFF;
    //CHECA SE É A PRIMEIRA VEZ QUE O FIRMWARE FOI CARREGADO
    CheckFirstLinkOrganizeEEPROM();
    //INICIALIZA OS PARAMETROS GERAIS
    GeneralSettingsInitialization();
    //CALIBRAÇÃO DOS ESCS
    ESC.Calibration();
    //CHECA SE A IMU ESTÁ CALIBRADA E CARREGA OS VALORES DE CALIBRAÇÃO DA MESMA
    CheckAndUpdateIMUCalibration();
    //CARREGA OS VALORES DE CALIBRAÇÃO DO COMPASS
    COMPASS.UpdateCompassCalibration();
    //INICIALIZA OS DISPOSITIVOS I2C
    I2C.All_Initialization();
    //INICIA O BUZZER EM OPERAÇÃO NORMAL
    ESC.BeeperMode = NORMAL_OPERATION_MODE;
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
    ConfigureRegisters(false);
    //INICIA O SISTEMA DE TASKS
    TaskSystemInitialization();
    //CALCULA E IMPRIME O TEMPO GASTO PELA INICIALIZAÇÃO
    CalculeTheFinalTimeToInitTheMachine(&MachineInitTimeNow);
    DEBUG("LOG: Sistema Inicializado! Tempo Gasto:%ld Segundos\n", GetTheFinalTimeToInitTheMachine(&MachineInitTimeNow));
}

void MachineRun()
{
    //SISTEMA DE TASK EM LOOP INFINITO
    TaskSystemRun();
}