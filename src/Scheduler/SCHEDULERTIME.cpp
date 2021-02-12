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

#include "SCHEDULERTIME.h"
#ifdef __AVR_ATmega2560__
#include <avr/interrupt.h>
#endif

SchedulerTimeClass SCHEDULERTIME;

#ifdef __AVR_ATmega2560__

volatile uint32_t Timer0_OverFlow = 0;
volatile uint32_t Timer0_Scheduler_Millis = 0;
static uint8_t Timer0_Fraction = 0;

void SchedulerTimeClass::Initialization()
{
  __asm__ __volatile__("sei" ::
                           : "memory");
  _SFR_BYTE(TCCR0A) |= _BV(WGM01);
  _SFR_BYTE(TCCR0A) |= _BV(WGM00);
  _SFR_BYTE(TCCR0B) |= _BV(CS01);
  _SFR_BYTE(TCCR0B) |= _BV(CS00);
  _SFR_BYTE(TIMSK0) |= _BV(TOIE0);
  TCCR1B = 0;
  _SFR_BYTE(TCCR1B) |= _BV(CS11);
  _SFR_BYTE(TCCR1B) |= _BV(CS10);
  _SFR_BYTE(TCCR1A) |= _BV(WGM10);
  _SFR_BYTE(TCCR2B) |= _BV(CS22);
  _SFR_BYTE(TCCR2A) |= _BV(WGM20);
  _SFR_BYTE(TCCR3B) |= _BV(CS31);
  _SFR_BYTE(TCCR3B) |= _BV(CS30);
  _SFR_BYTE(TCCR3A) |= _BV(WGM30);
  _SFR_BYTE(TCCR4B) |= _BV(CS41);
  _SFR_BYTE(TCCR4B) |= _BV(CS40);
  _SFR_BYTE(TCCR4A) |= _BV(WGM40);
  _SFR_BYTE(TCCR5B) |= _BV(CS51);
  _SFR_BYTE(TCCR5B) |= _BV(CS50);
  _SFR_BYTE(TCCR5A) |= _BV(WGM50);
  _SFR_BYTE(ADCSRA) |= _BV(ADPS2);
  _SFR_BYTE(ADCSRA) |= _BV(ADPS1);
  _SFR_BYTE(ADCSRA) |= _BV(ADPS0);
  _SFR_BYTE(ADCSRA) |= _BV(ADEN);
}

uint32_t SchedulerTimeClass::GetMillis()
{
  uint32_t MillisCount;
  uint8_t oldSREG = SREG;
  __asm__ __volatile__("cli" ::
                           : "memory");
  MillisCount = Timer0_Scheduler_Millis;
  SREG = oldSREG;
  return MillisCount;
}

uint32_t SchedulerTimeClass::GetMicros()
{
  uint32_t MillisCount;
  uint8_t oldSREG = SREG, TCNTCount;
  __asm__ __volatile__("cli" ::
                           : "memory");
  MillisCount = Timer0_OverFlow;
  TCNTCount = TCNT0;
  if ((TIFR0 & _BV(TOV0)) && (TCNTCount < 255))
  {
    MillisCount++;
  }
  SREG = oldSREG;
  return ((MillisCount << 8) + TCNTCount) * 4;
}

void SchedulerTimeClass::Sleep(uint16_t MillisSeconds)
{
  uint32_t Start = SCHEDULERTIME.GetMicros();
  while (MillisSeconds > 0)
  {
    SCHEDULERTIME.GetMicros();
    SCHEDULERTIME.GetMillis();
    while ((SCHEDULERTIME.GetMicros() - Start) >= 1000)
    {
      MillisSeconds--;
      if (MillisSeconds == 0)
      {
        break;
      }
      Start += 1000;
    }
  }
}

void SchedulerTimeClass::MicroSecondsSleep(uint16_t MicroSeconds)
{
  if (--MicroSeconds == 0)
  {
    return;
  }
  MicroSeconds <<= 2;
  MicroSeconds -= 2;
  __asm__ __volatile__("1: sbiw %0,1"
                       "\n\t"
                       "brne 1b"
                       : "=w"(MicroSeconds)
                       : "0"(MicroSeconds));
}

extern "C" void __vector_23(void) __attribute__((signal, __INTR_ATTRS));
void __vector_23(void)
{
  uint32_t MillisCount = Timer0_Scheduler_Millis;
  uint8_t FractionCount = Timer0_Fraction;
  MillisCount += 1;
  FractionCount += (1024 % 1000) >> 3;
  if (FractionCount >= 1000 >> 3)
  {
    FractionCount -= 1000 >> 3;
    MillisCount += 1;
  }
  Timer0_Fraction = FractionCount;
  Timer0_Scheduler_Millis = MillisCount;
  Timer0_OverFlow++;
}

#elif defined ESP32

void SchedulerTimeClass::Initialization()
{
}

uint32_t SchedulerTimeClass::GetMillis()
{
  return millis();
}

uint32_t SchedulerTimeClass::GetMicros()
{
  return micros();
}

void SchedulerTimeClass::Sleep(uint16_t MillisSeconds)
{
  delay(MillisSeconds);
}

void SchedulerTimeClass::MicroSecondsSleep(uint16_t MicroSeconds)
{
  delayMicroseconds(MicroSeconds);
}

#elif defined __arm__

void SchedulerTimeClass::Initialization()
{
}

uint32_t SchedulerTimeClass::GetMillis()
{
  return millis();
}

uint32_t SchedulerTimeClass::GetMicros()
{
  return micros();
}

void SchedulerTimeClass::Sleep(uint16_t MillisSeconds)
{
  delay(MillisSeconds);
}

void SchedulerTimeClass::MicroSecondsSleep(uint16_t MicroSeconds)
{
  delayMicroseconds(MicroSeconds);
}
#endif