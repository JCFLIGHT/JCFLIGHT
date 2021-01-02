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

#include "SAFETYBUTTON.h"
#include "Common/VARIABLES.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"
#include "MotorsControl/MOTORS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Build/BOARDDEFS.h"
#include "AnalogDigitalConverter/ADC.h"

SAFETYBUTTONCLASS SAFETYBUTTON;

#define FIRST_CYCLE_COUNT 30  //1 SEGUNDO
#define SECOND_CYCLE_COUNT 60 //1 SEGUNDO
#define RESET_CYCLE_COUNT 150 //3 SEGUNDOS
#define MAX_BUTTON_COUNT 255  //ITERAÇÕES
#define BUTTON_SCHEDULE 33    //33MS - 30HZ

void SAFETYBUTTONCLASS::Initialization()
{
    SAFETY_BUTTON_PININPUT;
    SAFETY_BUTTON_LED_PINOUT;
}

bool SAFETYBUTTONCLASS::GetButtonInterval()
{
    if (AVRTIME.SchedulerMillis() - LastDebounceTime < BUTTON_SCHEDULE)
    {
        return false;
    }
    LastDebounceTime = AVRTIME.SchedulerMillis();
    return true;
}

bool SAFETYBUTTONCLASS::GetButtonState()
{
#ifdef __AVR_ATmega2560__
    return (SAFETY_BUTTON_PIN_READ_STATE ? false : true);
#elif defined __arm__ || defined ESP32
    return (ADCPIN.Read(SAFETY_BUTTON_PIN_READ_STATE) ? false : true);
#endif
}

void SAFETYBUTTONCLASS::FlashButton()
{
    static Led_Pattern Pattern;

    if (DetectRise < FIRST_CYCLE_COUNT)
    {
        Pattern = Led_Pattern::FMU_REFUSE_TO_ARM;
    }
    else if (DetectRise > FIRST_CYCLE_COUNT && DetectRise < SECOND_CYCLE_COUNT) //VERIFICAÇÃO 1
    {
        Pattern = Led_Pattern::FMU_INIT_ARM;
        if (DetectRise == FIRST_CYCLE_COUNT + 1)
            BEEPER.Play(BEEPER_FMU_INIT);
        if (GetButtonState())
        {
            WaitToNextProcess = true;
            PulseInAllMotors(1000);
        }
        else
        {
            if (!WaitToNextProcess)
                DetectRise = FIRST_CYCLE_COUNT + 1;
        }
    }
    else if (DetectRise > SECOND_CYCLE_COUNT && DetectRise < RESET_CYCLE_COUNT) //VERIFICAÇÃO 2
    {
        Pattern = Led_Pattern::FMU_SAFE_TO_ARM;
        if (DetectRise == SECOND_CYCLE_COUNT + 1)
        {
            BEEPER.Play(BEEPER_FMU_SAFE_TO_ARM);
            WaitToNextProcess = false;
        }
        if (GetButtonState())
        {
            WaitToNextProcess = true;
            SafeStateToApplyPulse = true;
        }
        else
        {
            if (!WaitToNextProcess)
                DetectRise = SECOND_CYCLE_COUNT + 1;
        }
    }
    else if (DetectRise > RESET_CYCLE_COUNT) //RESET
    {
        if (DetectRise == RESET_CYCLE_COUNT + 1)
        {
            BEEPER.Play(BEEPER_ACTION_FAIL);
            WaitToNextProcess = false;
        }
        else
        {
            if (GetButtonState())
            {
                DetectRise = 0;
                PulseInAllMotors(0);
                SafeStateToApplyPulse = false;
            }
            else
            {
                if (DetectRise > RESET_CYCLE_COUNT + 1)
                    DetectRise = MAX_BUTTON_COUNT - 2;
            }
        }
    }
    UpdateLedStatus(Pattern);
}

void SAFETYBUTTONCLASS::UpdateLedStatus(enum Led_Pattern Instance)
{
    SetStateToLed(((uint16_t)Instance & (1 << (Blink_Counter++ / 3))));
    if (Blink_Counter > 45)
        Blink_Counter = 0;
}

void SAFETYBUTTONCLASS::SetStateToLed(bool State)
{
    if (!State)
    {
        SAFETY_BUTTON_LED_OFF;
    }
    else
    {
        SAFETY_BUTTON_LED_ON;
    }
}

bool SAFETYBUTTONCLASS::GetSafeStateToOutput()
{
    if (!SafeButtonEnabled())
        return true;
    return SafeStateToApplyPulse;
}

bool SAFETYBUTTONCLASS::SafeButtonEnabled()
{
    if (STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == OFF_ALL_DISP ||
        STORAGEMANAGER.Read_8Bits(DISP_PASSIVES_ADDR) == ONLY_BUZZER)
    {
        return false;
    }
    return true;
}

void SAFETYBUTTONCLASS::UpdateRoutine(void)
{
    if (!SafeButtonEnabled())
        return;
    if (!GetButtonInterval())
        return;
    if (COMMAND_ARM_DISARM)
        return;
    FlashButton();
    if (!GetButtonState() && DetectRise < MAX_BUTTON_COUNT)
        DetectRise++;
}
