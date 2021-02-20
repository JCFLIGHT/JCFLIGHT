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
#include "RadioControl/DECODE.h"
#include "RadioControl/RCCONFIG.h"
#include "Buzzer/BUZZER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSTATES.h"
#include "FastSerial/PRINTF.h"

ClassESC ESC;

#define ESC_CAL_THROTTLE_FAIL RadioControllOutput[THROTTLE] < 1600   //VALOR MAXIMO TOLERADO PARA NÃO ENTRAR NO MODO CALIBRAÇÃO DOS ESC'S
#define ESC_CAL_THROTTLE_SUCESS RadioControllOutput[THROTTLE] > 1700 //VALOR MINIMO TOLERADO PARA ENTRAR NO MODO CALIBRAÇÃO DOS ESC'S

void ClassESC::Calibration(void)
{
  if (ESC_CAL_THROTTLE_FAIL)
  {
    LOG("O usuario pulou a etapa de calib dos escs.");
    return; //FAÇA UMA RAPIDA SAIDA DA FUNÇÃO CASO O USUARIO NÃO QUEIRA CALIBRAR OS ESC'S
  }
  if (ESC_CAL_THROTTLE_SUCESS) //CHECA SE O VALOR DO ACELERADOR É SAFE
  {
    LOG("Etapa de calib dos escs inicializada!");
    ConfigureRegisters(true);           //INICIA OS REGISTRADORES DE CONFIGURAÇÃO DE SAIDA DOS PINOS PWM
    PulseInAllMotors(MAX_STICKS_PULSE); //ENVIA PWM MAXIMO A TODOS OS ESC'S
    while (true)
    {
      //ESSE PRIMEIRO WHILE DURA 10 SEGUNDOS
      //5 SEGUNDOS ENVIANDO 2000uS PARA OS ESCS
      //5 SEGUNDOS ENVIANDO 1000uS PARA OS ESCS
      static uint32_t CountDelay = SCHEDULERTIME.GetMillis();
      static bool IgnoreThis = false;
      RGB.Function(CALIBRATIONESC);                       //ATIVA O LED VERMELHO
      RGB.Update();                                       //ATUALIZA O ESTADO DOS LED'S
      if (SCHEDULERTIME.GetMillis() - CountDelay >= 5000) //ROTINA DE CONTAGEM DE 5 SEGUNDOS
      {
        if (IgnoreThis)
        {
          break; //QUEBRA O WHILE
        }
        IgnoreThis = true;
        CountDelay = SCHEDULERTIME.GetMillis();
      }
      if (IgnoreThis)
      {
        PulseInAllMotors(MIN_STICKS_PULSE); //ENVIA PWM MINIMO A TODOS OS ESC'S
      }
    }
    BeeperMode = ESC_FINISH_CALIBRATION_MODE;
    LOG("Etapa de calib dos escs finalizada!");
    while (true) //FICA TRAVADO AQUI NO WHILE ATÉ QUE A CONTROLADORA SEJA REINICIADA MANUALMENTE
    {
      static uint32_t EscCalLoopRefresh = SCHEDULERTIME.GetMillis();
      if (SCHEDULERTIME.GetMillis() - EscCalLoopRefresh >= 20) //ROTINA DE 50Hz
      {
        DECODE.Update();            //FAZ A LEITURA DE TODOS OS CANAIS DO RECEPTOR DO RADIO
        RCCONFIG.Set_Pulse();       //SETA A SAÍDA PARA CONFIGURAÇÃO PARA O RECEPTOR DO RADIO
        RCCONFIG.Update_Channels(); //FAZ A LEITURA DOS CANAIS DO RECEPTOR DO RADIO APÓS A CONFIGURAÇÃO
        BEEPER.Run();
        RGB.Function(CALIBRATIONESCFINISH); //ATIVA O LED VERDE
        RGB.Update();                       //ATUALIZA O ESTADO DOS LED'S
        if (BeeperMode > 0)
        {
          BeeperMode--;
        }
        if (SticksStateToArm() && !EscCal_ArmTest)
        {
          EscCal_ArmCount++; //REALIZA 50 CONTAGENS = 1 SEGUNDO
          if (EscCal_ArmCount >= 50)
          {
            EscCal_ArmTest = true; //ARMA OS MOTORES PARA TESTE
            LOG("Sistema armado para teste dos motores!");
          }
        }
        if (SticksStateToDisarm() && EscCal_ArmTest)
        {
          EscCal_ArmCount = EscCal_ArmTest = false; //DESARMA OS MOTORES E RESETA A CONTAGEM
          LOG("Sistema desarmado!");
        }
        EscCalLoopRefresh = SCHEDULERTIME.GetMillis();
      }
      if (EscCal_ArmTest)
      {
        PulseInAllMotors(Constrain_16Bits(RadioControllOutput[THROTTLE], MIN_STICKS_PULSE, MAX_STICKS_PULSE)); //REALIZA O BY-PASS DO THROTTLE
      }
      else
      {
        PulseInAllMotors(MIN_STICKS_PULSE); //DESLIGA OS MOTORES
      }
    }
  }
}
