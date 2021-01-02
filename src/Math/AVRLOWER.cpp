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

#include "AVRLOWER.h"

#ifdef __AVR_ATmega2560__

#define MultiS16X16to32(OperationResult, ValueIntA, ValueIntB) \
  asm volatile(                                                \
      "clr r26 \n\t"                                           \
      "mul %A1, %A2 \n\t"                                      \
      "movw %A0, r0 \n\t"                                      \
      "muls %B1, %B2 \n\t"                                     \
      "movw %C0, r0 \n\t"                                      \
      "mulsu %B2, %A1 \n\t"                                    \
      "sbc %D0, r26 \n\t"                                      \
      "add %B0, r0 \n\t"                                       \
      "adc %C0, r1 \n\t"                                       \
      "adc %D0, r26 \n\t"                                      \
      "mulsu %B1, %A2 \n\t"                                    \
      "sbc %D0, r26 \n\t"                                      \
      "add %B0, r0 \n\t"                                       \
      "adc %C0, r1 \n\t"                                       \
      "adc %D0, r26 \n\t"                                      \
      "clr r1 \n\t"                                            \
      : "=&r"(OperationResult)                                 \
      : "a"(ValueIntA),                                        \
        "a"(ValueIntB)                                         \
      : "r26")

int32_t __attribute__((noinline)) Multiplication32Bits(int16_t ValueA, int16_t ValueB)
{
  int32_t Result;
  MultiS16X16to32(Result, ValueA, ValueB);
  return Result;
}

#elif defined __arm__ || defined ESP32

int32_t __attribute__((noinline)) Multiplication32Bits(int16_t ValueA, int16_t ValueB)
{
  int32_t Result;
  Result = ValueA * ValueB;
  return Result;
}

#endif