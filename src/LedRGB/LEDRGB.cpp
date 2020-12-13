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

#include "LEDRGB.h"
#include "Common/VARIABLES.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "IOMCU/IOMCU.h"

LEDRGB RGB;

bool NotPriorit = false;

void LEDRGB::Initialization()
{
  DDRB |= (1 << DDD4); //DEFINE A PORTA DIGITAL 10 COMO SAIDA
  DDRB |= (1 << DDD5); //DEFINE A PORTA DIGITAL 11 COMO SAIDA
  DDRB |= (1 << DDD6); //DEFINE A PORTA DIGITAL 12 COMO SAIDA
}

void LEDRGB::Update()
{
  //REGISTRADORES DE MANIPULAÇÃO DO LED RGB
  OCR2A = LedRGB[RED];   //PINO DIGITAL 10
  OCR1A = LedRGB[GREEN]; //PINO DIGITAL 11
  OCR1B = LedRGB[BLUE];  //PINO DIGITAL 12
  if ((CalibratingAccelerometer == 0) &&
      (!GCS.ConfigFlight) &&
      (!CalibratingCompass) &&
      (!NotPriorit))
    RGB.Function(GPSLED);
  if (GCS.ConfigFlight)
    RGB.Function(CONFIGFLIGHT);
}

void LEDRGB::Function(uint8_t Mode)
{
  switch (Mode)
  {

  //GPS LED É PRIORIDADE
  case GPSLED:
    RGB.GPS_Led();
    break;

  case ACCLED:
    RGB.ACC_Led();
    break;

  case MAGLED:
    RGB.MAG_Led();
    break;

  case CONFIGFLIGHT:
    RGB.ConfigFlight_Led();
    break;

  case CALIBRATIONESC:
    RGB.CalibEsc_Led();
    break;

  case CALIBRATIONESCFINISH:
    RGB.CalibEscFinish_Led();
    break;

  case SAVINGVALUESSAVETRIM:
    RGB.Saving_SaveTrim_Led();
    NotPriorit = true;
    break;

  case SUCESSSAVETRIM:
    RGB.Sucess_SaveTrim_Led();
    NotPriorit = true;
    break;

  case FAILSAVETRIM:
    RGB.Fail_SaveTrim_Led();
    NotPriorit = true;
    break;

  case PREARMINIT:
    RGB.Pre_Arm_Initializing();
    NotPriorit = true;
    break;

  case PREARMSUCESS:
    RGB.Pre_Arm_Sucess();
    NotPriorit = true;
    break;

  case PREARMFAIL:
    RGB.Pre_Arm_Fail();
    NotPriorit = true;
    break;

  case OFFLEDS:
    RGB.Off_All_Leds();
    NotPriorit = true;
    break;
  }
}

void LEDRGB::ACC_Led(void)
{
  //LED RGB
  LedRGB[RED] = 0;     //VERMELHO
  LedRGB[GREEN] = 254; //VERDE
  LedRGB[BLUE] = 220;  //AZUL
}

void LEDRGB::MAG_Led(void)
{
  //LED RGB
  LedRGB[RED] = 180;  //VERMELHO
  LedRGB[GREEN] = 0;  //VERDE
  LedRGB[BLUE] = 220; //AZUL
}

void LEDRGB::ConfigFlight_Led(void)
{
  static bool ToogleBlinkConfig = true;
  static uint32_t StoreBlinkConfig = AVRTIME.SchedulerMillis();
  if (AVRTIME.SchedulerMillis() - StoreBlinkConfig >= 500)
  {
    ToogleBlinkConfig = !ToogleBlinkConfig;
    StoreBlinkConfig = AVRTIME.SchedulerMillis();
  }
  if (ToogleBlinkConfig)
  {
    //LED RGB
    LedRGB[RED] = 190;   //VERMELHO
    LedRGB[GREEN] = 240; //VERDE
    LedRGB[BLUE] = 0;    //AZUL
  }
  else
    RGB.Off_All_Leds();
}

void LEDRGB::CalibEsc_Led(void)
{
  static uint8_t FlashLedCount = 0;
  static uint32_t FlashTimer = AVRTIME.SchedulerMillis();
  //TEMPO DE ATUALIZAÇÃO DO LED FLASHER
  if (AVRTIME.SchedulerMillis() - FlashTimer >= 170)
  {
    FlashLedCount += 1;
    FlashTimer = AVRTIME.SchedulerMillis();
  }
  //LED FLASHER POR PARTES
  switch (FlashLedCount)
  {

  case 1:
    LedRGB[RED] = 254; //VERMELHO
    LedRGB[GREEN] = 0; //VERDE
    LedRGB[BLUE] = 0;  //AZUL
    break;

  case 2:
    LedRGB[RED] = 0;     //VERMELHO
    LedRGB[GREEN] = 254; //VERDE
    LedRGB[BLUE] = 0;    //AZUL
    break;

  case 3:
    LedRGB[RED] = 0;    //VERMELHO
    LedRGB[GREEN] = 0;  //VERDE
    LedRGB[BLUE] = 254; //AZUL
    break;

  case 4:
  case 5:
    LedRGB[RED] = 0;   //VERMELHO
    LedRGB[GREEN] = 0; //VERDE
    LedRGB[BLUE] = 0;  //AZUL
    break;

  case 6:
    FlashLedCount = 1;
    break;
  }
}

void LEDRGB::CalibEscFinish_Led(void)
{
  //LED RGB
  LedRGB[RED] = 0;     //VERMELHO
  LedRGB[GREEN] = 254; //VERDE
  LedRGB[BLUE] = 0;    //AZUL
}

void LEDRGB::GPS_Led(void)
{
  //SE O NÚMERO DE SATELITES FOR MENOR OU IGUAL A 4,O LED VERMELHO IRÁ FICAR PISCANDO SEM PARAR
  static bool GPS_Fail_Toggle = false;
  static uint32_t GPS_Fail = AVRTIME.SchedulerMillis();
  if (GPS_NumberOfSatellites <= 4)
  {
    if (AVRTIME.SchedulerMillis() - GPS_Fail >= 350)
    {
      GPS_Fail_Toggle = !GPS_Fail_Toggle;
      GPS_Fail = AVRTIME.SchedulerMillis();
    }
    if (GPS_Fail_Toggle)
    {
      //LIGA O LED VERMELHO
      LedRGB[RED] = 254; //VERMELHO
      LedRGB[GREEN] = 0; //VERDE
      LedRGB[BLUE] = 0;  //AZUL
    }
    else
      RGB.Off_All_Leds();
    return; //NÃO FAÇA O QUE ESTÁ ABAIXO
  }
  //SE O NUMERO DE GPS FOR MAIOR OU IGUAL A 5,O LED VERDE IRÁ PISCAR DE ACORDO COM O NÚMERO DE SATELITES
  //5 SATELITES         = 1 PISCADA
  //6 SATELITES         = 2 PISCADAS
  //7 SATELITES         = 3 PISCADAS
  //8 SATELITES OU MAIS = 4 PISCADAS
  static uint8_t BlinkCount;
  static uint32_t BlinkTime = AVRTIME.SchedulerMillis();
  if (GPS_Flight_Mode != GPS_MODE_RTH)
  {
    if (AVRTIME.SchedulerMillis() - BlinkTime >= 150)
    {
      if (GPS_NumberOfSatellites >= 5)
      {
        if (GPS_NumberOfSatellites >= 8)
        {
          if (BlinkCount++ > 16)
            BlinkCount = 0;
        }
        else
        {
          if (BlinkCount++ > 2 * GPS_NumberOfSatellites)
            BlinkCount = 0;
        }
        if (BlinkCount >= 10 && ((BlinkCount % 2) == 0))
        {
          //INDICA O NÚM. DE SAT.(VERDE)
          LedRGB[RED] = 0;     //VERMELHO
          LedRGB[GREEN] = 254; //VERDE
          LedRGB[BLUE] = 0;    //AZUL
        }
        if (BlinkCount == 6 && ((BlinkCount % 2) == 0))
        {
          //INDICA O HOME POINT (AZUL)
          LedRGB[RED] = 0;    //VERMELHO
          LedRGB[GREEN] = 0;  //VERDE
          LedRGB[BLUE] = 254; //AZUL
        }
      }
      else
        BlinkCount = 0;
      BlinkTime = AVRTIME.SchedulerMillis();
    }
    else
      RGB.Off_All_Leds();
  }
  //INDICAÇÃO DO MODO DE VOO RTH
  if (GPS_Flight_Mode == GPS_MODE_RTH)
  {
    if (AVRTIME.SchedulerMicros() % 100000 > 50000)
    {
      LedRGB[RED] = 0;    //VERMELHO
      LedRGB[GREEN] = 0;  //VERDE
      LedRGB[BLUE] = 254; //AZUL
    }
    else
      RGB.Off_All_Leds();
  }
}

void LEDRGB::Saving_SaveTrim_Led(void)
{
  LedRGB[RED] = 0;    //VERMELHO
  LedRGB[GREEN] = 0;  //VERDE
  LedRGB[BLUE] = 254; //AZUL
}

void LEDRGB::Sucess_SaveTrim_Led(void)
{
  LedRGB[RED] = 0;     //VERMELHO
  LedRGB[GREEN] = 254; //VERDE
  LedRGB[BLUE] = 0;    //AZUL
}

void LEDRGB::Fail_SaveTrim_Led(void)
{
  LedRGB[RED] = 254; //VERMELHO
  LedRGB[GREEN] = 0; //VERDE
  LedRGB[BLUE] = 0;  //AZUL
}

void LEDRGB::Pre_Arm_Initializing(void)
{
  LedRGB[RED] = 0;    //VERMELHO
  LedRGB[GREEN] = 0;  //VERDE
  LedRGB[BLUE] = 254; //AZUL
}

void LEDRGB::Pre_Arm_Sucess(void)
{
  LedRGB[RED] = 0;     //VERMELHO
  LedRGB[GREEN] = 254; //VERDE
  LedRGB[BLUE] = 0;    //AZUL
}

void LEDRGB::Pre_Arm_Fail(void)
{
  LedRGB[RED] = 254; //VERMELHO
  LedRGB[GREEN] = 0; //VERDE
  LedRGB[BLUE] = 0;  //AZUL
}

void LEDRGB::Off_All_Leds(void)
{
  LedRGB[RED] = 0;   //VERMELHO
  LedRGB[GREEN] = 0; //VERDE
  LedRGB[BLUE] = 0;  //AZUL
}