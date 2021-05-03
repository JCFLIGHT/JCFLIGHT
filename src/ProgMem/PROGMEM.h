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

#ifndef PROGMEM_H_
#define PROGMEM_H_

#ifdef __AVR_ATmega2560__

#define FLASH_MEMORY_ATTRIBUTE __attribute__((__progmem__))

#define ProgMemReadByte(Address) (__extension__({   \
    uint16_t __Address16Bits = (uint16_t)(Address); \
    uint8_t __Result;                               \
    __asm__ __volatile__("lpm %0, Z"                \
                         "\n\t"                     \
                         : "=r"(__Result)           \
                         : "z"(__Address16Bits));   \
    __Result;                                       \
}))

#define ProgMemReadWord(Address) (__extension__({                \
    uint16_t __Address16Bits = (uint16_t)(Address);              \
    int16_t __Result;                                            \
    __asm__ __volatile__("lpm %A0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %B0, Z"                            \
                         "\n\t"                                  \
                         : "=r"(__Result), "=z"(__Address16Bits) \
                         : "1"(__Address16Bits));                \
    __Result;                                                    \
}))

#define ProgMemReadDWord(Address) (__extension__({               \
    uint16_t __Address16Bits = (uint16_t)(Address);              \
    int32_t __Result;                                            \
    __asm__ __volatile__("lpm %A0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %B0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %C0,Z+"                            \
                         "\n\t"                                  \
                         "lpm %D0, Z"                            \
                         "\n\t"                                  \
                         : "=r"(__Result), "=z"(__Address16Bits) \
                         : "1"(__Address16Bits));                \
    __Result;                                                    \
}))

#define ProgMemReadFloat(Address) (__extension__({               \
    uint16_t __Address16Bits = (uint16_t)(Address);              \
    float __Result;                                              \
    __asm__ __volatile__("lpm %A0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %B0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %C0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %D0, Z"                            \
                         "\n\t"                                  \
                         : "=r"(__Result), "=z"(__Address16Bits) \
                         : "1"(__Address16Bits));                \
    __Result;                                                    \
}))

#define ProgMemReadPTR(Address) (void *)(__extension__({         \
    uint16_t __Address16Bits = (uint16_t)(Address);              \
    uint16_t __Result;                                           \
    __asm__ __volatile__("lpm %A0, Z+"                           \
                         "\n\t"                                  \
                         "lpm %B0, Z"                            \
                         "\n\t"                                  \
                         : "=r"(__Result), "=z"(__Address16Bits) \
                         : "1"(__Address16Bits));                \
    __Result;                                                    \
}))

#elif defined __arm__ || defined ESP32

#define FLASH_MEMORY_ATTRIBUTE

#define ProgMemReadByte(Address) (*(const unsigned char *)(Address))

#define ProgMemReadWord(Address) ({ typeof(Address) _Address = (Address); *(const unsigned short *)(_Address); })

#define ProgMemReadDWord(Address) ({ typeof(Address) _Address = (Address); *(const unsigned long *)(_Address); })

#define ProgMemReadFloat(Address) ({ typeof(Address) _Address = (Address); *(const float *)(_Address); })

#define ProgMemReadPTR(Address) ({ typeof(Address) _Address = (Address); *(void * const *)(_Address); })

#endif

#endif