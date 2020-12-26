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

#include "FUNCTIONSNAME.h"
#include "ProgMem/PROGMEM.h"

#ifdef __AVR_ATmega2560__

char GetFunctionName[20];

const char Function_0[] __attribute__((__progmem__)) = "Slow_Loop()";
const char Function_1[] __attribute__((__progmem__)) = "Medium_Loop()";
const char Function_2[] __attribute__((__progmem__)) = "Fast_Medium_Loop()";
const char Function_3[] __attribute__((__progmem__)) = "Fast_Loop()";
const char Function_4[] __attribute__((__progmem__)) = "Integral_Loop()";

const char *const Function_Table[] __attribute__((__progmem__)) = {
    Function_0,
    Function_1,
    Function_2,
    Function_3,
    Function_4,
};

void UpdateFunctionName(uint8_t FunctionNumber)
{
    strcpy_P(GetFunctionName, (char *)ProgMemReadDWord((uint16_t)(&(Function_Table[FunctionNumber]))));
}

#elif defined __arm__

char *GetFunctionName;

void UpdateFunctionName(uint8_t FunctionNumber)
{
    if (FunctionNumber == 0)
    {
        GetFunctionName = (char *)"Slow_Loop()";
    }
    else if (FunctionNumber == 1)
    {
        GetFunctionName = (char *)"Medium_Loop()";
    }
    else if (FunctionNumber == 2)
    {
        GetFunctionName = (char *)"Fast_Medium_Loop()";
    }
    else if (FunctionNumber == 3)
    {
        GetFunctionName = (char *)"Fast_Loop()";
    }
    else if (FunctionNumber == 4)
    {
        GetFunctionName = (char *)"Integral_Loop()";
    }
}
#endif