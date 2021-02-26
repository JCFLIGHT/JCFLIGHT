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

#include "AVRPPM.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "RadioControl/DECODE.h"
#include "BAR/BAR.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "FailSafe/FAILSAFE.h"
#include "Common/ENUM.h"

#ifdef __AVR_ATmega2560__

void _PPM_Initialization()
{
    DDRK &= ~(1 << 7);  //DECLARA COMO ENTRADA
    PORTK |= (1 << 7);  //ATIVA O PULL-UP
    PCICR |= (1 << 2);  //CONFIGURA COMO FALLING
    PCMSK2 |= (1 << 7); //ATIVA A INTERRUPÇÃO
}

void AVRInterruptRoutine(void)
{
    static uint8_t Channels = 0;
    static uint8_t CheckFailSafe;
    uint16_t PPMTimer;
    uint16_t PPMTimerDifference;
    static uint16_t PPMStoredTimer = 0;
    PPMTimer = SCHEDULERTIME.GetMicros();
    __asm__ __volatile__("sei" ::
                             : "memory");
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

extern "C" void __vector_11(void) __attribute__((signal, used, externally_visible));
void __vector_11(void)
{
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER) ||
        (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == IBUS_RECEIVER))
    {
        return;
    }
    if ((*(volatile uint8_t *)(0x106)) & 128)
    {
        AVRInterruptRoutine();
    }
}

#endif