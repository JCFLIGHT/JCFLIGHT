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
        PID[PIDROLL].ProportionalVector = 40;
        PID[PIDROLL].IntegratorVector = 30;
        PID[PIDROLL].DerivativeVector = 23;
        PID[PIDROLL].FeedForward = 60;
        //PID PITCH
        PID[PIDPITCH].ProportionalVector = 40;
        PID[PIDPITCH].IntegratorVector = 30;
        PID[PIDPITCH].DerivativeVector = 23;
        PID[PIDPITCH].FeedForward = 60;
        //PID YAW
        PID[PIDYAW].ProportionalVector = 85;
        PID[PIDYAW].IntegratorVector = 45;
        PID[PIDYAW].DerivativeVector = 0;
        PID[PIDYAW].FeedForward = 60;
        //PID DO MODO AUTO-NIVEL
        PID[PIDAUTOLEVEL].ProportionalVector = 20;
        PID[PIDAUTOLEVEL].IntegratorVector = 15;
    }
    else if (GetFrameStateOfAirPlane())
    {
        //AEROS
        //PID ROLL
        PID[PIDROLL].ProportionalVector = 5;
        PID[PIDROLL].IntegratorVector = 7;
        PID[PIDROLL].DerivativeVector = 0;
        PID[PIDROLL].FeedForward = 50;
        //PID PITCH
        PID[PIDPITCH].ProportionalVector = 5;
        PID[PIDPITCH].IntegratorVector = 7;
        PID[PIDPITCH].DerivativeVector = 0;
        PID[PIDPITCH].FeedForward = 50;
        //PID YAW
        PID[PIDYAW].ProportionalVector = 6;
        PID[PIDYAW].IntegratorVector = 10;
        PID[PIDYAW].DerivativeVector = 0;
        PID[PIDYAW].FeedForward = 60;
        //PID DO MODO AUTO-NIVEL
        PID[PIDAUTOLEVEL].ProportionalVector = 20;
        PID[PIDAUTOLEVEL].IntegratorVector = 5;
    }
    //PID PARA A RETENÇÃO DE ALTITUDE (ALTITUDE-HOLD)
    PID[PIDALTITUDE].ProportionalVector = 50;
    PID[PIDALTITUDE].IntegratorVector = 20;
    PID[PIDALTITUDE].DerivativeVector = 16;
    //PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    PID[PIDGPSPOSITION].ProportionalVector = 100;
    PID[PIDGPSPOSITION].IntegratorVector = 90;
    PID[PIDGPSPOSITION].DerivativeVector = 0;
    //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    PID[PIDGPSPOSITIONRATE].ProportionalVector = 70;
    PID[PIDGPSPOSITIONRATE].IntegratorVector = 20;
    PID[PIDGPSPOSITIONRATE].DerivativeVector = 20;
    //RATE DE NAVEGAÇÃO (RHT E MISSÃO)
    PID[PIDGPSNAVIGATIONRATE].ProportionalVector = 25;
    PID[PIDGPSNAVIGATIONRATE].IntegratorVector = 33;
    PID[PIDGPSNAVIGATIONRATE].DerivativeVector = 83;
    //HEADING-HOLD RATE
    PID[PIDYAWVELOCITY].ProportionalVector = 40;
}

void UpdateValuesOfPID()
{
    static bool ValuesUpdated = false;

    if (ValuesUpdated && GCS.UpdatePID)
    {
        return;
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != PID[PIDROLL].ProportionalVector)
    {
        PID[PIDROLL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, PID[PIDROLL].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != PID[PIDROLL].IntegratorVector)
    {
        PID[PIDROLL].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, PID[PIDROLL].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != PID[PIDROLL].DerivativeVector)
    {
        PID[PIDROLL].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, PID[PIDROLL].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != PID[PIDPITCH].ProportionalVector)
    {
        PID[PIDPITCH].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, PID[PIDPITCH].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != PID[PIDPITCH].IntegratorVector)
    {
        PID[PIDPITCH].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, PID[PIDPITCH].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != PID[PIDPITCH].DerivativeVector)
    {
        PID[PIDPITCH].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, PID[PIDPITCH].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != PID[PIDYAW].ProportionalVector)
    {
        PID[PIDYAW].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, PID[PIDYAW].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != PID[PIDYAW].IntegratorVector)
    {
        PID[PIDYAW].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, PID[PIDYAW].IntegratorVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != PID[PIDYAW].DerivativeVector)
    {
        PID[PIDYAW].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, PID[PIDYAW].DerivativeVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != PID[PIDALTITUDE].ProportionalVector)
    {
        PID[PIDALTITUDE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, PID[PIDALTITUDE].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != PID[PIDGPSPOSITION].ProportionalVector)
    {
        PID[PIDGPSPOSITION].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, PID[PIDGPSPOSITION].ProportionalVector);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != PID[PIDGPSPOSITION].IntegratorVector)
    {
        PID[PIDGPSPOSITION].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, PID[PIDGPSPOSITION].IntegratorVector);
    }

    GCS.UpdatePID = ValuesUpdated = true;
}
