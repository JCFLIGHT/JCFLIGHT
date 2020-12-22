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

#include "SAVETRIM.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "SwitchFlag/SWITCHFLAG.h"
#include "LedRGB/LEDRGB.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "BAR/BAR.h"
#include "IMUHEALTH.h"

/*
  MODO DE USO DO SAVE-TRIM:
  CORRIJA O DRIFT DO DRONE USANDO OS TRIM'S DO RADIO CONTROLE,NÃO SÃO OS STICK'S.
  SE ESTIVER DERIVANDO PARA FRENTE,DIMINUA O TRIM DO ROLL,
  SE ESTIVER DERIVANDO PARA A DIREITA,DIMINUA O TRIM DO PITCH,
  E VICE-VERSA.
  ---------------------------------------------------------------------------------------------------------------------------------------
  PARA ATIVAR:COM A CONTROLADORA ARMADA E EM VOO,BATA A CHAVE AUX DO MODO IOC 4 VEZES
  ---------------------------------------------------------------------------------------------------------------------------------------
  PARA DESATIVAR E SALVAR:COM A CONTROLADORA ARMADA E O DRONE EM VOO,BATA A CHAVE DO IOC 2 VEZES,DESÇA O DRONE,DESARME MANUALMENTE
  OU ESPERE DESARMAR AUTOMATICAMENTE,E POR FIM ESPERE OS LEDS DE CONFIRMAÇÃO DO SAVE-TRIM.
  ---------------------------------------------------------------------------------------------------------------------------------------
  PARA DESATIVAR SEM SALVAR:POUSAR O DRONE,DESARMAR A CONTROLADORA MANUALMENTE OU ESPERAR DESARMAR AUTOMATICAMENTE,
  APÓS DESARMAR BATA A CHAVE DO MODO IOC 2 VEZES E O LED VERMELHO IRÁ PISCAR INDICANDO O ATO.
*/

bool Save_Trim_Sucess = false;
bool Save_Trim_EEPROM = false;
bool SucessOrFail = false;
uint8_t Restore_Acc_Values = 0;
uint8_t Save_Trim_Count = 0;

void Save_Trim_Init()
{
  if (SaveTrimState && COMMAND_ARM_DISARM)
  {
    if (!Save_Trim_Sucess)
    {
      Save_Trim_Count = 50; //50 CICLOS
      Restore_Acc_Values++; //INICIA A ITERAÇÃO
      if (Restore_Acc_Values > 200)
        Restore_Acc_Values = 201; //PARA EVITAR OVERFLOW NA ITERAÇÃO
    }
  }
  else if (Save_Trim_Sucess && !COMMAND_ARM_DISARM)
  { //PROCESSO PARA SALVAR O SAVE-TRIM NA EEPROM
    Restore_Acc_Values = 0;
    Save_Trim_EEPROM = true;
    Save_Trim_Sucess = false;
  }
  Save_Trim_Run();
}

void Save_Trim_Run()
{
  static int32_t Angle_Trim[3];
  static int16_t Acc_Orig_Values[3] = {0, 0, 0};
  //SALVA OS VALORES DE CALIBRAÇÃO ORIGINAIS PARA RESTAURAÇÃO CASO O SAVE-TRIM NÃO SEJA SALVO PELO USUARIO
  if (Save_Trim_Count == 50 && Restore_Acc_Values > 0 && Restore_Acc_Values < 10)
  {
    Acc_Orig_Values[PITCH] = CALIBRATION.AccelerometerCalibration[PITCH]; //SALVA OS VALORES ORIGINAIS DO EIXO PITCH
    Acc_Orig_Values[ROLL] = CALIBRATION.AccelerometerCalibration[ROLL];   //SALVA OS VALORES ORIGINAIS DO EIXO ROLL
    Acc_Orig_Values[YAW] = CALIBRATION.AccelerometerCalibration[YAW];     //SALVA OS VALORES ORIGINAIS DO EIXO YAW
  }
  if (Save_Trim_Count > 0 && Restore_Acc_Values > 10)
  {
    //RESETA O ANGLE TRIM E INICIA A CALIBRAÇÃO
    if (Save_Trim_Count == 50)
      Angle_Trim[PITCH] = 0;
    if (Save_Trim_Count == 50)
      Angle_Trim[ROLL] = 0;
    if (Save_Trim_Count == 50)
      Angle_Trim[YAW] = 0;
    Angle_Trim[PITCH] += IMU.AccelerometerRead[PITCH];
    Angle_Trim[ROLL] += IMU.AccelerometerRead[ROLL];
    Angle_Trim[YAW] += IMU.AccelerometerRead[YAW];
    //LIMPA OS VALORES DE LEITURA E DE CALIBRAÇÃO DO ACELEROMETRO PARA UMA INICIAR UMA NOVA LEITURA
    IMU.AccelerometerRead[PITCH] = 0;
    IMU.AccelerometerRead[ROLL] = 0;
    IMU.AccelerometerRead[YAW] = 0;
    CALIBRATION.AccelerometerCalibration[PITCH] = 0;
    CALIBRATION.AccelerometerCalibration[ROLL] = 0;
    CALIBRATION.AccelerometerCalibration[YAW] = 0;
    if (Save_Trim_Count == 1)
    {
      /*
        RECUPERA OS VALORES ORIGINAIS DE CALIBRAÇÃO DO ACELEROMETRO.
        SE O SAVE-TRIM FOR SALVO PELO USUARIO O SAVE-TRIM IRÁ PARA O PROXIMO PASSO (SALVAR NA EEPROM E APLICAR),
        CASO CONTRARIO OS VALORES DE CALIBRAÇÃO VOLTAM AOS VALORES ORIGINAIS DA CALIBRAÇÃO DE 6 EIXOS.
      */
      CALIBRATION.AccelerometerCalibration[ROLL] = Acc_Orig_Values[ROLL];
      CALIBRATION.AccelerometerCalibration[PITCH] = Acc_Orig_Values[PITCH];
      CALIBRATION.AccelerometerCalibration[YAW] = Acc_Orig_Values[YAW];
      if (!SaveTrimState && COMMAND_ARM_DISARM)
        Save_Trim_Sucess = true;
    }
    Save_Trim_Count--;
  }
  if (Save_Trim_EEPROM)
  {
    //O USUARIO QUIS SALVAR OS VALORES NA EEPROM?O DRONE FOI DESARMADO?SIM...
    //CALCULA A TRIMAGEM DE TODOS OS ANGULOS E CONVERTE PARA UM NOVO VALOR DE CALIBRAÇÃO E TERMINA TODO O PROCESSO DO SAVE-TRIM
    CALIBRATION.AccelerometerCalibration[ROLL] = Angle_Trim[ROLL] / 50;
    CALIBRATION.AccelerometerCalibration[PITCH] = Angle_Trim[PITCH] / 50;
    CALIBRATION.AccelerometerCalibration[YAW] = Angle_Trim[YAW] / (-462);
    //CALIBRATION.AccelerometerCalibration[YAW] = Angle_Trim[YAW] / 50 - 512;
    //SALVA TODOS OS VALORES DO SAVE-TRIM NA EEPROM
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADDR, CALIBRATION.AccelerometerCalibration[ROLL]);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADDR, CALIBRATION.AccelerometerCalibration[PITCH]);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_ADDR, CALIBRATION.AccelerometerCalibration[YAW]);
    CheckAndUpdateIMUCalibration();
    SucessOrFail = true;
    Save_Trim_EEPROM = false;
  }
  Led_Save_Trim();
}

void Led_Save_Trim()
{
  static uint8_t CountLedDelay = 0;
  static uint8_t LedToogleSaveTrim = 0;
  static uint32_t StoreTimer[2] = {0, 0};
  if (SucessOrFail)
  {
    if ((AVRTIME.SchedulerMillis() - StoreTimer[0]) > 1000)
    {
      CountLedDelay++;
      StoreTimer[0] = AVRTIME.SchedulerMillis();
    }
    if (CountLedDelay >= 0 && CountLedDelay <= 2)
    {
      RGB.Function(SAVINGVALUESSAVETRIM);
    }
    else if (CountLedDelay > 2 && CountLedDelay <= 4)
    {
      if ((AVRTIME.SchedulerMillis() - StoreTimer[1]) > 100)
      {
        LedToogleSaveTrim = !LedToogleSaveTrim;
        if (LedToogleSaveTrim)
          RGB.Function(SUCESSSAVETRIM);
        else
          RGB.Off_All_Leds();
        StoreTimer[1] = AVRTIME.SchedulerMillis();
      }
    }
    else
    {
      SucessOrFail = false;
      NotPriorit = false;
      CountLedDelay = 0;
    }
  }
  if (Restore_Acc_Values > 10 && !COMMAND_ARM_DISARM && !Save_Trim_Sucess)
  {
    if ((AVRTIME.SchedulerMillis() - StoreTimer[0]) > 1000)
    {
      CountLedDelay++;
      StoreTimer[0] = AVRTIME.SchedulerMillis();
    }
    if (CountLedDelay >= 0 && CountLedDelay <= 2)
    {
      RGB.Function(SAVINGVALUESSAVETRIM);
    }
    else if (CountLedDelay > 2 && CountLedDelay <= 4)
    {
      if ((AVRTIME.SchedulerMillis() - StoreTimer[1]) > 100)
      {
        LedToogleSaveTrim = !LedToogleSaveTrim;
        if (LedToogleSaveTrim)
          RGB.Function(FAILSAVETRIM);
        else
          RGB.Off_All_Leds();
        StoreTimer[1] = AVRTIME.SchedulerMillis();
      }
    }
    else
    {
      NotPriorit = false;
      Restore_Acc_Values = 0;
      CountLedDelay = 0;
    }
  }
}
