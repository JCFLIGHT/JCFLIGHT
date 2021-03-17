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

#include "ARMING.h"
#include "RadioControl/RCSTATES.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "BatteryMonitor/BATTERY.h"
#include "ProgMem/PROGMEM.h"
#include "Glitch/GLITCH.h"
#include "IOMCU/IOMCU.h"
#include "FailSafe/FAILSAFE.h"
#include "GPSNavigation/NAVIGATION.h"
#include "Common/STRUCTS.h"
#include "PerformanceCalibration/PERFORMGYRO.h"

PreArmClass PREARM;

#ifdef __AVR_ATmega2560__

const char Message_0[] __attribute__((__progmem__)) = "Erro:Acelerometro ruim;";
const char Message_1[] __attribute__((__progmem__)) = "Erro:Modo de voo ativo;";
const char Message_2[] __attribute__((__progmem__)) = "Erro:GPS Glitch;";
const char Message_3[] __attribute__((__progmem__)) = "Erro:Fail-Safe ativo;";
const char Message_4[] __attribute__((__progmem__)) = "Erro:Giroscopio ruim;";
const char Message_5[] __attribute__((__progmem__)) = "Erro:Controladora muito inclinada;";
const char Message_6[] __attribute__((__progmem__)) = "Erro:O switch nao foi ativado para o modo safe;";
const char Message_7[] __attribute__((__progmem__)) = "Erro:Bateria ruim;";
const char Message_8[] __attribute__((__progmem__)) = "Nenhum erro,seguro para armar;";
const char Message_9[] __attribute__((__progmem__)) = "Erro:Compass ruim;";
const char Message_10[] __attribute__((__progmem__)) = "Erro:Barometro ruim;";

#elif defined __arm__ || defined ESP32

const char *const Message_0 = "Erro:Acelerometro ruim;";
const char *const Message_1 = "Erro:Modo de voo ativo;";
const char *const Message_2 = "Erro:GPS Glitch;";
const char *const Message_3 = "Erro:Fail-Safe ativo;";
const char *const Message_4 = "Erro:Giroscopio ruim;";
const char *const Message_5 = "Erro:Controladora muito inclinada;";
const char *const Message_6 = "Erro:O switch nao foi ativado para o modo safe;";
const char *const Message_7 = "Erro:Bateria ruim;";
const char *const Message_8 = "Nenhum erro,seguro para armar;";
const char *const Message_9 = "Erro:Compass ruim;";
const char *const Message_10 = "Erro:Barometro ruim;";

#endif

void PreArmClass::UpdateGCSErrorText(uint8_t GCSErrorType)
{
    switch (GCSErrorType)
    {

    case IMU_ERROR:
        GCS.SendStringToGCS(Message_0);
        break;

    case FLIGHT_MODES_ERROR:
        GCS.SendStringToGCS(Message_1);
        break;

    case GPS_ERROR:
        GCS.SendStringToGCS(Message_2);
        break;

    case FAIL_SAFE_ERROR:
        GCS.SendStringToGCS(Message_3);
        break;

    case GYRO_EEROR:
        GCS.SendStringToGCS(Message_4);
        break;

    case INCLINATION_ERROR:
        GCS.SendStringToGCS(Message_5);
        break;

    case BUTTON_ERROR:
        GCS.SendStringToGCS(Message_6);
        break;

    case BATTERY_ERROR:
        GCS.SendStringToGCS(Message_7);
        break;

    case COMPASS_ERROR:
        GCS.SendStringToGCS(Message_9);
        break;

    case BAROMETER_ERROR:
        GCS.SendStringToGCS(Message_10);
        break;

    case NONE_ERROR:
        GCS.SendStringToGCS(Message_8);
        break;
    }
}

uint8_t PreArmClass::Checking(void)
{
    if (CALIBRATION.AccelerometerZero[ROLL] > 0x7D0) //IMU NÃO CALIBRADA
    {
        return IMU_ERROR;
    }

    if (GPS_Flight_Mode != GPS_MODE_NONE) //MODOS DE VOO ATIVOS
    {
        return FLIGHT_MODES_ERROR;
    }

    if (SystemInFailSafe()) //MODO FAIL-SAFE ATIVO
    {
        return FAIL_SAFE_ERROR;
    }

    if (GyroCalibrationRunning()) //GYROSCOPIO EM CALIBRAÇÃO
    {
        return GYRO_EEROR;
    }

    if (CheckInclinationForArm()) //INCLINAÇÃO DE 25 GRAUS DETECTADA
    {
        return INCLINATION_ERROR;
    }

    if (!SAFETYBUTTON.GetSafeStateToOutput()) //SAFETY-BUTTON EMBARCADO,PORÉM NÃO ESTÁ NO MODE "SAFE"
    {
        return BUTTON_ERROR;
    }

    if (BATTERY.LowBattPreventArm) //BATERIA COM BAIXA TENSÃO
    {
        return BATTERY_ERROR;
    }

    if (!GLITCH.CheckCompass()) //CHECA O COMPASS
    {
        return COMPASS_ERROR;
    }

    if (!GLITCH.CheckBarometer()) //CHECA O BAROMETRO
    {
        return BAROMETER_ERROR;
    }

    if (!GLITCH.CheckGPS()) //CHECA O GPS
    {
        return GPS_ERROR;
    }

    //TUDO ESTÁ OK,A CONTROLADORA ESTÁ PRONTA PARA ARMAR
    return NONE_ERROR;
}

bool PreArmClass::CheckSafeState(void)
{
    if (PREARM.Checking() == NONE_ERROR ||       //NENHUM DISPOSITVO ESTÁ RUIM
        PREARM.Checking() == GPS_ERROR ||        //NOTIFIQUE QUE O GPS ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
        PREARM.Checking() == COMPASS_ERROR ||    //NOTIFIQUE QUE O COMPASS ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
        PREARM.Checking() == BAROMETER_ERROR ||  //NOTIFIQUE QUE O BAROMETRO ESTÁ RUIM,MAS NÃO IMPEÇA DE ARMAR
        PREARM.Checking() == FLIGHT_MODES_ERROR) //NOTIFIQUE QUE O MODO DE VOO POR GPS ESTÁ ATIVO,MAS NÃO IMPEÇA DE ARMAR
    {
        return true;
    }
    return false;
}