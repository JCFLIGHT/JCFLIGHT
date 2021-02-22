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

void LoadPID()
{
    //PID ROLL
    PID[PIDROLL].ProportionalVector = 35;
    PID[PIDROLL].IntegratorVector = 25;
    PID[PIDROLL].DerivativeVector = 26;
    //PID PITCH
    PID[PIDPITCH].ProportionalVector = 35;
    PID[PIDPITCH].IntegratorVector = 25;
    PID[PIDPITCH].DerivativeVector = 26;
    //PID YAW
    PID[PIDYAW].ProportionalVector = 69;
    PID[PIDYAW].IntegratorVector = 50;
    PID[PIDYAW].DerivativeVector = 0;
    //PID DO MODO AUTO-NIVEL
    PID[PIDAUTOLEVEL].ProportionalVector = 80;
    PID[PIDAUTOLEVEL].IntegratorVector = 3;
    PID[PIDAUTOLEVEL].DerivativeVector = 100;
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

    if ((STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != PID[PIDROLL].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != 0))
    {
        PID[PIDROLL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, PID[PIDROLL].ProportionalVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != PID[PIDROLL].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != 0))
    {
        PID[PIDROLL].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, PID[PIDROLL].IntegratorVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != PID[PIDROLL].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != 0))
    {
        PID[PIDROLL].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, PID[PIDROLL].DerivativeVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != PID[PIDPITCH].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != 0))
    {
        PID[PIDPITCH].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, PID[PIDPITCH].ProportionalVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != PID[PIDPITCH].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != 0))
    {
        PID[PIDPITCH].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, PID[PIDPITCH].IntegratorVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != PID[PIDPITCH].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != 0))
    {
        PID[PIDPITCH].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, PID[PIDPITCH].DerivativeVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != PID[PIDYAW].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != 0))
    {
        PID[PIDYAW].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, PID[PIDYAW].ProportionalVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != PID[PIDYAW].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != 0))
    {
        PID[PIDYAW].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, PID[PIDYAW].IntegratorVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != PID[PIDYAW].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != 0))
    {
        PID[PIDYAW].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, PID[PIDYAW].DerivativeVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != PID[PIDALTITUDE].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != 0))
    {
        PID[PIDALTITUDE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, PID[PIDALTITUDE].ProportionalVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != PID[PIDGPSPOSITION].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != 0))
    {
        PID[PIDGPSPOSITION].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, PID[PIDGPSPOSITION].ProportionalVector);
    }

    if ((STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != PID[PIDGPSPOSITION].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != 0))
    {
        PID[PIDGPSPOSITION].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);
    }

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) == 0)
    {
        STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, PID[PIDGPSPOSITION].IntegratorVector);
    }

    GCS.UpdatePID = ValuesUpdated = true;
}
