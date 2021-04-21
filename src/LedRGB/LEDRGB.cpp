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
#include "Scheduler/SCHEDULERTIME.h"
#include "IOMCU/IOMCU.h"
#include "Build/BOARDDEFS.h"
#include "GPS/GPSSTATES.h"
#include "Compass/COMPASSREAD.h"
#include "GPS/GPSUBLOX.h"
#include "GPSNavigation/NAVIGATION.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "PerformanceCalibration/PERFORMGYRO.h"
#include "RadioControl/STICKS.h"
#include "IMU/ACCGYROREAD.h"

LEDRGB RGB;

#ifdef __AVR_ATmega2560__
#define MAX_PWM 254
#elif defined ESP32
#define MAX_PWM 4095
#elif defined __arm__
#define MAX_PWM 254
#endif

void LEDRGB::Initialization()
{
  RED_LED_PINOUT;
  GREEN_LED_PINOUT;
  BLUE_LED_PINOUT;
#if defined ESP32
  AnalogWriteSetSettings(RED_LED_PWM_REGISTER, 490, 12);
  AnalogWriteSetSettings(GREEN_LED_PWM_REGISTER, 490, 12);
  AnalogWriteSetSettings(BLUE_LED_PWM_REGISTER, 490, 12);
#endif
}

void LEDRGB::Update()
{
  if (!AccCalibrationRunning() && !GyroCalibrationRunning() && !GCS.ConfigFlight && !IMU.Compass.Calibrating && !STICKS.PreArm_Run)
  {
    RGB.Function(CALL_LED_GPS);
  }

  if (GCS.ConfigFlight)
  {
    RGB.Function(CALL_LED_CONFIG_FLIGHT);
  }

  //REGISTRADORES DE MANIPULAÇÃO PWM DO LED RGB
#ifdef __AVR_ATmega2560__

  RED_LED_PWM_REGISTER = RGB.SetColorValue[RED];     //PINO DIGITAL 10
  GREEN_LED_PWM_REGISTER = RGB.SetColorValue[GREEN]; //PINO DIGITAL 11
  BLUE_LED_PWM_REGISTER = RGB.SetColorValue[BLUE];   //PINO DIGITAL 12

#elif defined ESP32

  AnalogWriteApplyPulse(RED_LED_PWM_REGISTER, RGB.SetColorValue[RED]);     //GPIO4
  AnalogWriteApplyPulse(GREEN_LED_PWM_REGISTER, RGB.SetColorValue[GREEN]); //GPIO2
  AnalogWriteApplyPulse(BLUE_LED_PWM_REGISTER, RGB.SetColorValue[BLUE]);   //GPIO15

#elif defined __arm__

#endif
}

void LEDRGB::Function(uint8_t Mode)
{
  switch (Mode)
  {

  //GPS LED É PRIORIDADE
  case CALL_LED_GPS:
    RGB.GPS_Led();
    break;

  case CALL_LED_ACC_CALIBRATION:
    RGB.CalibAccLed();
    break;

  case CALL_LED_MAG_CALIBRATION:
    RGB.CalibMagLed();
    break;

  case CALL_LED_CONFIG_FLIGHT:
    RGB.ConfigFlight_Led();
    break;

  case CALL_LED_CALIBRATION_ESC:
    RGB.CalibEsc_Led();
    break;

  case CALL_LED_PRE_ARM_INIT:
    RGB.Pre_Arm_Initializing();
    break;

  case CALL_LED_PRE_ARM_SUCESS:
    RGB.Pre_Arm_Sucess();
    break;

  case CALL_LED_PRE_ARM_FAIL:
    RGB.Pre_Arm_Fail();
    break;

  case CALL_LED_GYRO_CALIBRATION:
    RGB.CalibGyroLed();
    break;
  }
}

void LEDRGB::CalibAccLed(void)
{
  //LED RGB
#ifdef __AVR_ATmega2560__
  RGB.SetColorValue[RED] = 0;         //VERMELHO
  RGB.SetColorValue[GREEN] = MAX_PWM; //VERDE
  RGB.SetColorValue[BLUE] = 220;      //AZUL
#endif

#ifdef ESP32
  RGB.SetColorValue[RED] = 0;         //VERMELHO
  RGB.SetColorValue[GREEN] = MAX_PWM; //VERDE
  RGB.SetColorValue[BLUE] = 4061;     //AZUL
#endif
}

void LEDRGB::CalibGyroLed(void)
{
  static uint8_t NextColor = 0;
  static uint32_t StoreInterval = SCHEDULERTIME.GetMillis();

  if (SCHEDULERTIME.GetMillis() - StoreInterval >= 250)
  {
    NextColor++;
    StoreInterval = SCHEDULERTIME.GetMillis();
  }

  switch (NextColor)
  {

  case 0:
    //LED RGB
#ifdef __AVR_ATmega2560__
    RGB.SetColorValue[RED] = 190;   //VERMELHO
    RGB.SetColorValue[GREEN] = 240; //VERDE
    RGB.SetColorValue[BLUE] = 0;    //AZUL
#endif

#ifdef ESP32
    RGB.SetColorValue[RED] = 4031;   //VERMELHO
    RGB.SetColorValue[GREEN] = 4081; //VERDE
    RGB.SetColorValue[BLUE] = 0;     //AZUL
#endif
    break;

  case 1:
//LED RGB
#ifdef __AVR_ATmega2560__
    RGB.SetColorValue[RED] = 180;  //VERMELHO
    RGB.SetColorValue[GREEN] = 0;  //VERDE
    RGB.SetColorValue[BLUE] = 220; //AZUL
#endif

#ifdef ESP32
    RGB.SetColorValue[RED] = 4021;  //VERMELHO
    RGB.SetColorValue[GREEN] = 0;   //VERDE
    RGB.SetColorValue[BLUE] = 4061; //AZUL
#endif
    break;

  case 2:
    NextColor = 0;
    break;
  }
}

void LEDRGB::CalibMagLed(void)
{
//LED RGB
#ifdef __AVR_ATmega2560__
  RGB.SetColorValue[RED] = 180;  //VERMELHO
  RGB.SetColorValue[GREEN] = 0;  //VERDE
  RGB.SetColorValue[BLUE] = 220; //AZUL
#endif

#ifdef ESP32
  RGB.SetColorValue[RED] = 4021;  //VERMELHO
  RGB.SetColorValue[GREEN] = 0;   //VERDE
  RGB.SetColorValue[BLUE] = 4061; //AZUL
#endif
}

void LEDRGB::ConfigFlight_Led(void)
{
  static bool ToogleBlinkConfig = true;
  static uint32_t StoreBlinkConfig = SCHEDULERTIME.GetMillis();
  if (SCHEDULERTIME.GetMillis() - StoreBlinkConfig >= 500)
  {
    ToogleBlinkConfig = !ToogleBlinkConfig;
    StoreBlinkConfig = SCHEDULERTIME.GetMillis();
  }
  if (ToogleBlinkConfig)
  {
    //LED RGB
#ifdef __AVR_ATmega2560__
    RGB.SetColorValue[RED] = 190;   //VERMELHO
    RGB.SetColorValue[GREEN] = 240; //VERDE
    RGB.SetColorValue[BLUE] = 0;    //AZUL
#endif

#ifdef ESP32
    RGB.SetColorValue[RED] = 4031;   //VERMELHO
    RGB.SetColorValue[GREEN] = 4081; //VERDE
    RGB.SetColorValue[BLUE] = 0;     //AZUL
#endif
  }
  else
  {
    RGB.Off_All_Leds();
  }
}

void LEDRGB::CalibEsc_Led(void)
{
  static uint8_t FlashLedCount = 0;
  static uint32_t FlashTimer = SCHEDULERTIME.GetMillis();
  //TEMPO DE ATUALIZAÇÃO DO LED FLASHER
  if (SCHEDULERTIME.GetMillis() - FlashTimer >= 170)
  {
    FlashLedCount += 1;
    FlashTimer = SCHEDULERTIME.GetMillis();
  }
  //LED FLASHER POR PARTES
  switch (FlashLedCount)
  {

  case 1:
    RGB.SetColorValue[RED] = MAX_PWM; //VERMELHO
    RGB.SetColorValue[GREEN] = 0;     //VERDE
    RGB.SetColorValue[BLUE] = 0;      //AZUL
    break;

  case 2:
    RGB.SetColorValue[RED] = 0;         //VERMELHO
    RGB.SetColorValue[GREEN] = MAX_PWM; //VERDE
    RGB.SetColorValue[BLUE] = 0;        //AZUL
    break;

  case 3:
    RGB.SetColorValue[RED] = 0;        //VERMELHO
    RGB.SetColorValue[GREEN] = 0;      //VERDE
    RGB.SetColorValue[BLUE] = MAX_PWM; //AZUL
    break;

  case 4:
  case 5:
    RGB.Off_All_Leds();
    break;

  case 6:
    FlashLedCount = 1;
    break;
  }
}

void LEDRGB::GPS_Led(void)
{
  //SE O NÚMERO DE SATELITES FOR MENOR OU IGUAL A 4,O LED VERMELHO IRÁ FICAR PISCANDO SEM PARAR
  static bool GPS_Fail_Toggle = false;
  static uint32_t GPS_Fail = SCHEDULERTIME.GetMillis();
  if (Get_GPS_In_Bad_Condition())
  {
    if (SCHEDULERTIME.GetMillis() - GPS_Fail >= 350)
    {
      GPS_Fail_Toggle = !GPS_Fail_Toggle;
      GPS_Fail = SCHEDULERTIME.GetMillis();
    }
    if (GPS_Fail_Toggle)
    {
      //LIGA O LED VERMELHO
      RGB.SetColorValue[RED] = MAX_PWM; //VERMELHO
      RGB.SetColorValue[GREEN] = 0;     //VERDE
      RGB.SetColorValue[BLUE] = 0;      //AZUL
    }
    else
    {
      RGB.Off_All_Leds();
    }
    return; //NÃO FAÇA O QUE ESTÁ ABAIXO
  }
  //SE O NUMERO DE GPS FOR MAIOR OU IGUAL A 5,O LED VERDE IRÁ PISCAR DE ACORDO COM O NÚMERO DE SATELITES
  //5 SATELITES         = 1 PISCADA
  //6 SATELITES         = 2 PISCADAS
  //7 SATELITES         = 3 PISCADAS
  //8 SATELITES OU MAIS = 4 PISCADAS
  static uint8_t BlinkCount;
  static uint32_t BlinkTime = SCHEDULERTIME.GetMillis();
  if (GPSParameters.Mode.Navigation != DO_START_RTH)
  {
    if (SCHEDULERTIME.GetMillis() - BlinkTime >= 150)
    {
      if (Get_GPS_In_Good_Condition())
      {
        if (Get_GPS_In_Eight_Or_Plus_Satellites())
        {
          if (BlinkCount++ > 16)
          {
            BlinkCount = 0;
          }
        }
        else
        {
          if (BlinkCount++ > 2 * GPSParameters.Navigation.Misc.Get.Satellites)
            BlinkCount = 0;
        }
        if (BlinkCount >= 10 && ((BlinkCount % 2) == 0))
        {
          //INDICA O NÚM. DE SAT.(VERDE)
          RGB.SetColorValue[RED] = 0;         //VERMELHO
          RGB.SetColorValue[GREEN] = MAX_PWM; //VERDE
          RGB.SetColorValue[BLUE] = 0;        //AZUL
        }
        if (BlinkCount == 6 && ((BlinkCount % 2) == 0))
        {
          //INDICA O HOME POINT (AZUL)
          RGB.SetColorValue[RED] = 0;        //VERMELHO
          RGB.SetColorValue[GREEN] = 0;      //VERDE
          RGB.SetColorValue[BLUE] = MAX_PWM; //AZUL
        }
      }
      else
      {
        BlinkCount = 0;
      }
      BlinkTime = SCHEDULERTIME.GetMillis();
    }
    else
    {
      RGB.Off_All_Leds();
    }
  }
  else //INDICAÇÃO DO MODO DE VOO RTH
  {
    if (SCHEDULERTIME.GetMicros() % 100000 > 50000)
    {
      RGB.SetColorValue[RED] = 0;        //VERMELHO
      RGB.SetColorValue[GREEN] = 0;      //VERDE
      RGB.SetColorValue[BLUE] = MAX_PWM; //AZUL
    }
    else
    {
      RGB.Off_All_Leds();
    }
  }
}

void LEDRGB::Pre_Arm_Initializing(void)
{
  RGB.SetColorValue[RED] = 0;        //VERMELHO
  RGB.SetColorValue[GREEN] = 0;      //VERDE
  RGB.SetColorValue[BLUE] = MAX_PWM; //AZUL
}

void LEDRGB::Pre_Arm_Sucess(void)
{
  RGB.SetColorValue[RED] = 0;         //VERMELHO
  RGB.SetColorValue[GREEN] = MAX_PWM; //VERDE
  RGB.SetColorValue[BLUE] = 0;        //AZUL
}

void LEDRGB::Pre_Arm_Fail(void)
{
  RGB.SetColorValue[RED] = MAX_PWM; //VERMELHO
  RGB.SetColorValue[GREEN] = 0;     //VERDE
  RGB.SetColorValue[BLUE] = 0;      //AZUL
}

void LEDRGB::Off_All_Leds(void)
{
  RGB.SetColorValue[RED] = 0;   //VERMELHO
  RGB.SetColorValue[GREEN] = 0; //VERDE
  RGB.SetColorValue[BLUE] = 0;  //AZUL
}