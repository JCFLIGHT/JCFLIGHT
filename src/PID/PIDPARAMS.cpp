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

#include "PIDPARAMS.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "IOMCU/IOMCU.h"
#include "BAR/BAR.h"
#include "FrameStatus/FRAMESTATUS.h"

void LoadPID()
{
    if (GetFrameStateOfMultirotor())
    {
        //MULTIROTORES
        //PID ROLL
        GET_SET[PID_ROLL].ProportionalVector = 40;
        GET_SET[PID_ROLL].IntegralVector = 30;
        GET_SET[PID_ROLL].DerivativeVector = 23;
        GET_SET[PID_ROLL].FeedForwardVector = 60;
        //PID PITCH
        GET_SET[PID_PITCH].ProportionalVector = 40;
        GET_SET[PID_PITCH].IntegralVector = 30;
        GET_SET[PID_PITCH].DerivativeVector = 23;
        GET_SET[PID_PITCH].FeedForwardVector = 60;
        //PID YAW
        GET_SET[PID_YAW].ProportionalVector = 85;
        GET_SET[PID_YAW].IntegralVector = 45;
        GET_SET[PID_YAW].DerivativeVector = 0;
        GET_SET[PID_YAW].FeedForwardVector = 60;
        //PID DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].ProportionalVector = 20;
        GET_SET[PI_AUTO_LEVEL].IntegralVector = 15;
    }
    else if (GetFrameStateOfAirPlane())
    {
        //AEROS
        //PID ROLL
        GET_SET[PID_ROLL].ProportionalVector = 5;
        GET_SET[PID_ROLL].IntegralVector = 7;
        GET_SET[PID_ROLL].DerivativeVector = 0;
        GET_SET[PID_ROLL].FeedForwardVector = 50;
        //PID PITCH
        GET_SET[PID_PITCH].ProportionalVector = 5;
        GET_SET[PID_PITCH].IntegralVector = 7;
        GET_SET[PID_PITCH].DerivativeVector = 0;
        GET_SET[PID_PITCH].FeedForwardVector = 50;
        //PID YAW
        GET_SET[PID_YAW].ProportionalVector = 6;
        GET_SET[PID_YAW].IntegralVector = 10;
        GET_SET[PID_YAW].DerivativeVector = 0;
        GET_SET[PID_YAW].FeedForwardVector = 60;
        //PID DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].ProportionalVector = 20;
        GET_SET[PI_AUTO_LEVEL].IntegralVector = 5;
    }
    //PID PARA A RETENÇÃO DE ALTITUDE (ALTITUDE-HOLD)
    GET_SET[PID_ALTITUDE].ProportionalVector = 50;
    GET_SET[PID_ALTITUDE].IntegralVector = 20;
    GET_SET[PID_ALTITUDE].DerivativeVector = 16;
    //PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPS_POSITION].ProportionalVector = 100;
    GET_SET[PID_GPS_POSITION].IntegralVector = 90;
    GET_SET[PID_GPS_POSITION].DerivativeVector = 0;
    //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPS_POSITION_RATE].ProportionalVector = 70;
    GET_SET[PID_GPS_POSITION_RATE].IntegralVector = 20;
    GET_SET[PID_GPS_POSITION_RATE].DerivativeVector = 20;
    //RATE DE NAVEGAÇÃO (RHT E MISSÃO)
    GET_SET[PID_GPS_NAVIGATION_RATE].ProportionalVector = 25;
    GET_SET[PID_GPS_NAVIGATION_RATE].IntegralVector = 33;
    GET_SET[PID_GPS_NAVIGATION_RATE].DerivativeVector = 83;
    //HEADING-HOLD RATE
    GET_SET[P_YAW_RATE].ProportionalVector = 60;
    GET_SET[P_YAW_RATE_LIMIT].MinMaxValueVector = 90;
}

void UpdateValuesOfPID()
{
    if (GET_SET[PID_UPDATED].State)
    {
        return;
    }

    GET_SET[PID_ROLL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    GET_SET[PID_ROLL].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    GET_SET[PID_ROLL].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    GET_SET[PID_ROLL].FeedForwardVector = STORAGEMANAGER.Read_8Bits(FF_OR_CD_ROLL_ADDR);

    GET_SET[PID_PITCH].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    GET_SET[PID_PITCH].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    GET_SET[PID_PITCH].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    GET_SET[PID_PITCH].FeedForwardVector = STORAGEMANAGER.Read_8Bits(FF_OR_CD_PITCH_ADDR);

    GET_SET[PID_YAW].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    GET_SET[PID_YAW].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    GET_SET[PID_YAW].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    GET_SET[PID_YAW].FeedForwardVector = STORAGEMANAGER.Read_8Bits(FF_OR_CD_YAW_ADDR);

    GET_SET[PI_AUTO_LEVEL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_AUTOLEVEL_ADDR);
    GET_SET[PI_AUTO_LEVEL].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_AUTOLEVEL_ADDR);

    GET_SET[PID_ALTITUDE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);

    GET_SET[PID_GPS_POSITION].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    GET_SET[PID_GPS_POSITION].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);

    GET_SET[P_YAW_RATE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_HEADING_HOLD_ADDR);
    GET_SET[P_YAW_RATE_LIMIT].MinMaxValueVector = STORAGEMANAGER.Read_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR);

    GET_SET[PID_UPDATED].State = true;
}
