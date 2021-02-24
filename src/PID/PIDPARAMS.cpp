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
        GET_SET[PID_ROLL].FeedForward = 60;
        //PID PITCH
        GET_SET[PID_PITCH].ProportionalVector = 40;
        GET_SET[PID_PITCH].IntegralVector = 30;
        GET_SET[PID_PITCH].DerivativeVector = 23;
        GET_SET[PID_PITCH].FeedForward = 60;
        //PID YAW
        GET_SET[PID_YAW].ProportionalVector = 85;
        GET_SET[PID_YAW].IntegralVector = 45;
        GET_SET[PID_YAW].DerivativeVector = 0;
        GET_SET[PID_YAW].FeedForward = 60;
        //PID DO MODO AUTO-NIVEL
        GET_SET[PID_AUTO_LEVEL].ProportionalVector = 20;
        GET_SET[PID_AUTO_LEVEL].IntegralVector = 15;
    }
    else if (GetFrameStateOfAirPlane())
    {
        //AEROS
        //PID ROLL
        GET_SET[PID_ROLL].ProportionalVector = 5;
        GET_SET[PID_ROLL].IntegralVector = 7;
        GET_SET[PID_ROLL].DerivativeVector = 0;
        GET_SET[PID_ROLL].FeedForward = 50;
        //PID PITCH
        GET_SET[PID_PITCH].ProportionalVector = 5;
        GET_SET[PID_PITCH].IntegralVector = 7;
        GET_SET[PID_PITCH].DerivativeVector = 0;
        GET_SET[PID_PITCH].FeedForward = 50;
        //PID YAW
        GET_SET[PID_YAW].ProportionalVector = 6;
        GET_SET[PID_YAW].IntegralVector = 10;
        GET_SET[PID_YAW].DerivativeVector = 0;
        GET_SET[PID_YAW].FeedForward = 60;
        //PID DO MODO AUTO-NIVEL
        GET_SET[PID_AUTO_LEVEL].ProportionalVector = 20;
        GET_SET[PID_AUTO_LEVEL].IntegralVector = 5;
    }
    //PID PARA A RETENÇÃO DE ALTITUDE (ALTITUDE-HOLD)
    GET_SET[PID_ALTITUDE].ProportionalVector = 50;
    GET_SET[PID_ALTITUDE].IntegralVector = 20;
    GET_SET[PID_ALTITUDE].DerivativeVector = 16;
    //PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPSPOSITION].ProportionalVector = 100;
    GET_SET[PID_GPSPOSITION].IntegralVector = 90;
    GET_SET[PID_GPSPOSITION].DerivativeVector = 0;
    //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPS_POSITION_RATE].ProportionalVector = 70;
    GET_SET[PID_GPS_POSITION_RATE].IntegralVector = 20;
    GET_SET[PID_GPS_POSITION_RATE].DerivativeVector = 20;
    //RATE DE NAVEGAÇÃO (RHT E MISSÃO)
    GET_SET[PID_GPS_NAVIGATION_RATE].ProportionalVector = 25;
    GET_SET[PID_GPS_NAVIGATION_RATE].IntegralVector = 33;
    GET_SET[PID_GPS_NAVIGATION_RATE].DerivativeVector = 83;
    //HEADING-HOLD RATE
    GET_SET[PID_YAW_VELOCITY].ProportionalVector = 40;
}

void UpdateValuesOfPID()
{
    static bool ValuesUpdated = false;

    if (ValuesUpdated && GCS.UpdatePID)
    {
        return;
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != GET_SET[PID_ROLL].ProportionalVector)
    {
        GET_SET[PID_ROLL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, GET_SET[PID_ROLL].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != GET_SET[PID_ROLL].IntegralVector)
    {
        GET_SET[PID_ROLL].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, GET_SET[PID_ROLL].IntegralVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != GET_SET[PID_ROLL].DerivativeVector)
    {
        GET_SET[PID_ROLL].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, GET_SET[PID_ROLL].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != GET_SET[PID_PITCH].ProportionalVector)
    {
        GET_SET[PID_PITCH].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, GET_SET[PID_PITCH].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != GET_SET[PID_PITCH].IntegralVector)
    {
        GET_SET[PID_PITCH].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, GET_SET[PID_PITCH].IntegralVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != GET_SET[PID_PITCH].DerivativeVector)
    {
        GET_SET[PID_PITCH].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, GET_SET[PID_PITCH].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != GET_SET[PID_YAW].ProportionalVector)
    {
        GET_SET[PID_YAW].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, GET_SET[PID_YAW].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != GET_SET[PID_YAW].IntegralVector)
    {
        GET_SET[PID_YAW].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, GET_SET[PID_YAW].IntegralVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != GET_SET[PID_YAW].DerivativeVector)
    {
        GET_SET[PID_YAW].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, GET_SET[PID_YAW].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != GET_SET[PID_ALTITUDE].ProportionalVector)
    {
        GET_SET[PID_ALTITUDE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, GET_SET[PID_ALTITUDE].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != GET_SET[PID_GPSPOSITION].ProportionalVector)
    {
        GET_SET[PID_GPSPOSITION].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, GET_SET[PID_GPSPOSITION].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != GET_SET[PID_GPSPOSITION].IntegralVector)
    {
        GET_SET[PID_GPSPOSITION].IntegralVector = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, GET_SET[PID_GPSPOSITION].IntegralVector);
    }

    GCS.UpdatePID = ValuesUpdated = true;
}
