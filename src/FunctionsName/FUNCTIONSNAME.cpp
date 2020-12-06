#include "FUNCTIONSNAME.h"

#define __ProgramMemoryDWord__(addr) (__extension__({ uint16_t __addr16 = (uint16_t)(addr); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))

const char Function_0[] __attribute__((__progmem__)) = "Slow_Loop()";
const char Function_1[] __attribute__((__progmem__)) = "Medium_Loop()";
const char Function_2[] __attribute__((__progmem__)) = "Fast_Medium_Loop()";
const char Function_3[] __attribute__((__progmem__)) = "Fast_Loop()";
const char Function_4[] __attribute__((__progmem__)) = "Total_Loop()";

const char *const Function_Table[] __attribute__((__progmem__)) =
    {
        Function_0,
        Function_1,
        Function_2,
        Function_3,
        Function_4};

char GetFunctionName[20];

void UpdateFunctionName(uint8_t FunctionNumber)
{
    strcpy_P(GetFunctionName, (char *)__ProgramMemoryDWord__((uint16_t)(&(Function_Table[FunctionNumber]))));
}