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

#include "ESP32PPM.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "RadioControl/DECODE.h"
#include "BAR/BAR.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "FailSafe/FAILSAFE.h"
#include "Common/ENUM.h"

#if ESP32

void IRAM_ATTR ESP32InterruptRoutine(void)
{
    static uint8_t Channels = 0;
    static uint8_t CheckFailSafe;
    uint16_t PPMTimer;
    uint16_t PPMTimerDifference;
    static uint16_t PPMStoredTimer = 0;
    PPMTimer = SCHEDULERTIME.GetMicros();
    PPMTimerDifference = PPMTimer - PPMStoredTimer;
    PPMStoredTimer = PPMTimer;
    if (PPMTimerDifference > 2700)
    {
        Channels = RESET_PPM;
    }
    else
    {
        if (PPMTimerDifference > 750 && PPMTimerDifference < 2250)
        {
            DECODE.PPMReadChannels[Channels] = PPMTimerDifference;
            if (Channels < 4 && PPMTimerDifference > CHECKSUM.GetFailSafeValue)
            {
                CheckFailSafe |= (1 << Channels);
            }
            if (CheckFailSafe == 0x0F)
            {
                CheckFailSafe = 0;
                if (Fail_Safe_System_Count > 20)
                {
                    Fail_Safe_System_Count -= 20;
                }
                else
                {
                    Fail_Safe_System_Count = 0;
                }
            }
        }
        Channels++;
    }
}

void _PPM_Initialization()
{
    pinMode(GPIO_NUM_36, INPUT);
    attachInterrupt(GPIO_NUM_36, ESP32InterruptRoutine, FALLING);
}

#endif