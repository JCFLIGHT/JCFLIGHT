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
#include "Common/VARIABLES.h"
#include "RadioControl/STATES.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "BatteryMonitor/BATTERY.h"
#include "ProgMem/PROGMEM.h"
#include "Glitch/GLITCH.h"
#include "IOMCU/IOMCU.h"

PreArmClass PREARM;

enum GCS_Message_Type_Enum
{
    IMU_ERROR = 0,
    FLIGHT_MODES_ERROR,
    GPS_ERROR,
    FAIL_SAFE_ERROR,
    GYRO_EEROR,
    INCLINATION_ERROR,
    BUTTON_ERROR,
    BATTERY_ERROR,
    NONE_ERROR = 254
};

const char Message_0[] __attribute__((__progmem__)) = "Erro:Acelerometro ruim;";
const char Message_1[] __attribute__((__progmem__)) = "Erro:Modo de voo ativo;";
const char Message_2[] __attribute__((__progmem__)) = "Erro:GPS Glitch;";
const char Message_3[] __attribute__((__progmem__)) = "Erro:Fail-Safe ativo;";
const char Message_4[] __attribute__((__progmem__)) = "Erro:Gyroscopio ruim;";
const char Message_5[] __attribute__((__progmem__)) = "Erro:Controladora muito inclinada;";
const char Message_6[] __attribute__((__progmem__)) = "Erro:O switch nao foi ativado para o modo safe;";
const char Message_7[] __attribute__((__progmem__)) = "Erro:Bateria ruim;";
const char Message_8[] __attribute__((__progmem__)) = "Nenhum erro,seguro para armar;";

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

    if (SetFlightModes[ALTITUDE_HOLD_MODE] || GPS_Flight_Mode != GPS_MODE_NONE) //MODOS DE VOO ATIVOS
    {
        return FLIGHT_MODES_ERROR;
    }

    if (Fail_Safe_System > 5) //MODO FAIL-SAFE ATIVO
    {
        return FAIL_SAFE_ERROR;
    }

    if (CalibratingGyroscope > 0) //GYROSCOPIO EM CALIBRAÇÃO
    {
        return GYRO_EEROR;
    }

    if (CheckInclinationForCopter()) //INCLINAÇÃO DE 25 GRAUS DETECTADA
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

    if (!GLITCH.CheckGPS()) //CHECA O GPS
    {
        return GPS_ERROR;
    }

    //TUDO ESTÁ OK,A CONTROLADORA ESTÁ PRONTA PARA ARMAR
    return 254;
}

bool PreArmClass::CheckSafeState(void)
{
    if (Checking() == 254)
        return true;
    return false;
}