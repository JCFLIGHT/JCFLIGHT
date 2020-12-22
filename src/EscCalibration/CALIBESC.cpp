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

#include "CALIBESC.h"
#include "Common/VARIABLES.h"
#include "MotorsControl/MOTORS.h"
#include "LedRGB/LEDRGB.h"
#include "PPM/PPM.h"
#include "RadioControl/RCCONFIG.h"
#include "Buzzer/BUZZER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"

ClassESC ESC;

#define THROTTLEFAIL RadioControllOutput[THROTTLE] < 1600   //VALOR MAXIMO TOLERADO PARA NÃO ENTRAR NO MODO CALIBRAÇÃO DOS ESC'S
#define THROTTLESUCESS RadioControllOutput[THROTTLE] > 1700 //VALOR MINIMO TOLERADO PARA ENTRAR NO MODO CALIBRAÇÃO DOS ESC'S

bool ArmTest = false;
uint8_t ArmCount = 0;
uint32_t EscLoopRefresh = 0;

void ClassESC::Calibration(void)
{
  if (THROTTLEFAIL) //FAÇA UMA RAPIDA SAIDA DA FUNÇÃO CASO O USUARIO NÃO QUEIRA CALIBRAR OS ESC'S
    return;
  if (THROTTLESUCESS) //CHECA SE O VALOR DO ACELERADOR É SAFE
  {
    Run_Calibrate = true;   //FLAG PARA CALIBRAÇÃO DOS ESC'S
    ConfigureRegisters();   //INICIA OS REGISTRADORES DE CONFIGURAÇÃO DE SAIDA DOS PINOS PWM
    PulseInAllMotors(2000); //ENVIA PWM MAXIMO A TODOS OS ESC'S
    while (true)
    {
      static uint32_t CountDelay = AVRTIME.SchedulerMillis();
      static bool IgnoreThis = false;
      RGB.Function(CALIBRATIONESC);                       //ATIVA O LED VERMELHO
      RGB.Update();                                       //ATUALIZA O ESTADO DOS LED'S
      if (AVRTIME.SchedulerMillis() - CountDelay >= 5000) //ROTINA DE CONTAGEM DE 5 SEGUNDOS
      {
        IgnoreThis = true;
        CountDelay = AVRTIME.SchedulerMillis();
      }
      if (IgnoreThis)
        PulseInAllMotors(1000); //ENVIA PWM MINIMO A TODOS OS ESC'S
      if (IgnoreThis && AVRTIME.SchedulerMillis() - CountDelay >= 5000)
        break; //QUEBRA O WHILE
    }
    BeeperMode = ESC_FINISH_CALIBRATION_MODE;
    while (true) //FICA TRAVADO AQUI NO WHILE ATÉ QUE A CONTROLADORA SEJA REINICIADA MANUALMENTE
    {
      Run_Calibrate = false;
      if (AVRTIME.SchedulerMillis() - EscLoopRefresh >= 20) //ROTINA DE 50Hz
      {
        DecodeAllReceiverChannels(); //FAZ A LEITURA DE TODOS OS CANAIS DO RECEPTOR DO RADIO
        RCCONFIG.Set_Pulse();        //SETA A SAÍDA PARA CONFIGURAÇÃO PARA O RECEPTOR DO RADIO
        RCCONFIG.Update_Channels();  //FAZ A LEITURA DOS CANAIS DO RECEPTOR DO RADIO APÓS A CONFIGURAÇÃO
        BEEPER.Run();
        RGB.Function(CALIBRATIONESCFINISH); //ATIVA O LED VERDE
        RGB.Update();                       //ATUALIZA O ESTADO DOS LED'S
        if (BeeperMode > 0)
          BeeperMode--;
        if (RadioControllOutput[THROTTLE] < 1100 && RadioControllOutput[YAW] > 1900 &&
            RadioControllOutput[PITCH] < 1100 && RadioControllOutput[ROLL] < 1100 && !ArmTest)
          ArmCount++; //REALIZA 50 CONTAGENS = 1 SEGUNDO
        if (ArmCount >= 50)
          ArmTest = true; //ARMA OS MOTORES PARA TESTE
        if (RadioControllOutput[THROTTLE] < 1100 && RadioControllOutput[YAW] < 1100 &&
            RadioControllOutput[PITCH] > 1900 && RadioControllOutput[ROLL] < 1100 && ArmTest)
          ArmCount = ArmTest = false; //DESARMA OS MOTORES E RESETA A CONTAGEM
        EscLoopRefresh = AVRTIME.SchedulerMillis();
      }
      if (ArmTest)
        PulseInAllMotors(Constrain_16Bits(RadioControllOutput[THROTTLE], 1000, 2000)); //REALIZA O BY-PASS DO THROTTLE
      else
        PulseInAllMotors(1000); //DESLIGA OS MOTORES
    }
  }
}
