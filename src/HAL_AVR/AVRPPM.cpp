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
#include "Common/VARIABLES.h"
#include "PPM/PPM.h"
#include "BAR/BAR.h"

#ifdef __AVR_ATmega2560__

#define FAILSAFE_DETECT_TRESHOLD 975 //US

void AVR_PPM_Initialization()
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
    PPMTimer = SCHEDULER.GetMicros();
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
            PPMReadChannels[Channels] = PPMTimerDifference;
            if (Channels < 4 && PPMTimerDifference > FAILSAFE_DETECT_TRESHOLD)
            {
                CheckFailSafe |= (1 << Channels);
            }
            if (CheckFailSafe == 0x0F)
            {
                CheckFailSafe = 0;
                if (Fail_Safe_System > 20)
                {
                    Fail_Safe_System -= 20;
                }
                else
                {
                    Fail_Safe_System = 0;
                }
            }
        }
        Channels++;
    }
}

extern "C" void __vector_11(void) __attribute__((signal, __INTR_ATTRS));
void __vector_11(void)
{
    if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 1) ||
        (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == 2))
    {
        return;
    }
    if ((*(volatile uint8_t *)(0x106)) & 128)
    {
        AVRInterruptRoutine();
    }
}

#endif