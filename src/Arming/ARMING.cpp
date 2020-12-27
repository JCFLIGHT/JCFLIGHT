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

PreArmClass PREARM;

enum GCS_Error_Type_Enum
{
    IMU_ERROR = 0,
    FLIGHT_MODES_ERROR,
    GPS_ERROR,
    FAIL_SAFE_ERROR,
    GYRO_EEROR,
    INCLINATION_ERROR,
    BUTTON_ERROR,
    BATTERY_ERROR
};

char SendGCSErrorText[40];

const char Error_0[] __attribute__((__progmem__)) = "ACELEROMETRO RUIM";
const char Error_1[] __attribute__((__progmem__)) = "MODOS DE VOO ATIVADOS";
const char Error_2[] __attribute__((__progmem__)) = "GPS RUIM";
const char Error_3[] __attribute__((__progmem__)) = "FAIL-SAFE ATIVADO";
const char Error_4[] __attribute__((__progmem__)) = "GYROSCOPIO RUIM";
const char Error_5[] __attribute__((__progmem__)) = "CONTROLADORA INCLINADA DEMAIS";
const char Error_6[] __attribute__((__progmem__)) = "MODO SAFE NÃO ATIVADO COM O BOTÃO";
const char Error_7[] __attribute__((__progmem__)) = "BATERIA RUIM";

const char *const Error_Table[] __attribute__((__progmem__)) = {
    Error_0,
    Error_1,
    Error_2,
    Error_3,
    Error_4,
    Error_5,
    Error_6,
    Error_7,
};

void PreArmClass::UpdateGCSErrorText(uint8_t GCSErrorType)
{
    strcpy_P(SendGCSErrorText, (char *)ProgMemReadDWord((uint16_t)(&(Error_Table[GCSErrorType]))));
}

bool PreArmClass::Checking(void)
{
    if (CALIBRATION.AccelerometerZero[ROLL] > 0x7D0) //IMU NÃO CALIBRADA
    {
        UpdateGCSErrorText(IMU_ERROR);
        return true;
    }

    if (SetFlightModes[ALTITUDE_HOLD_MODE] || GPS_Flight_Mode != GPS_MODE_NONE) //MODOS DE VOO ATIVOS
    {
        UpdateGCSErrorText(FLIGHT_MODES_ERROR);
        return true;
    }

    if (Fail_Safe_System > 5) //MODO FAIL-SAFE ATIVO
    {
        UpdateGCSErrorText(FAIL_SAFE_ERROR);
        return true;
    }

    if (CalibratingGyroscope > 0) //GYROSCOPIO EM CALIBRAÇÃO
    {
        UpdateGCSErrorText(GYRO_EEROR);
        return true;
    }

    if (CheckInclinationForCopter()) //INCLINAÇÃO DE 25 GRAUS DETECTADA
    {
        UpdateGCSErrorText(INCLINATION_ERROR);
        return true;
    }

    if (!SAFETYBUTTON.GetSafeStateToOutput()) //SAFETY-BUTTON EMBARCADO,PORÉM NÃO ESTÁ NO MODE "SAFE"
    {
        UpdateGCSErrorText(BUTTON_ERROR);
        return true;
    }

    if (BATTERY.LowBattPreventArm) //BATERIA COM BAIXA TENSÃO
    {
        UpdateGCSErrorText(BATTERY_ERROR);
        return true;
    }

    if (!GLITCH.CheckGPS()) //CHECA O GPS,FAÇA APENAS A NOTIFICAÇÃO,NÃO IMPEÇA DE ARMAR
    {
        UpdateGCSErrorText(GPS_ERROR);
    }

    //TUDO ESTÁ OK,A CONTROLADORA ESTÁ PRONTA PARA ARMAR
    return false;
}