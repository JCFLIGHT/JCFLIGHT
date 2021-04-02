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

#pragma once

#include "stdint.h"

//CONVERTE STRING EM NÚMEROS INTEIROS OU FLOAT

int16_t ATO_Int(const char *Pointer)
{
    int16_t Signal = 1;
    int16_t Base = 0;
    int16_t IndexCounter = 0;

    while (Pointer[IndexCounter] == ' ')
    {
        IndexCounter++;
    }

    if (Pointer[IndexCounter] == '-' || Pointer[IndexCounter] == '+')
    {
        Signal = 1 - 2 * (Pointer[IndexCounter++] == '-');
    }

    while (Pointer[IndexCounter] >= '0' && Pointer[IndexCounter] <= '9')
    {
        if ((Base > (0x7fff / 10)) || (Base == (0x7fff / 10) && Pointer[IndexCounter] - '0' > 7))
        {
            if (Signal == 1)
            {
                return 0x7fff;
            }
            else
            {
                return (-0x7fff - 1);
            }
        }
        Base = 10 * Base + (Pointer[IndexCounter++] - '0');
    }
    return Base * Signal;
}

static float ATO_Float(const char *Pointer)
{
    int16_t Fraction = 0;
    float Signal;
    float Value;
    float Scale;

    while (((*Pointer) == ' ' || (*Pointer) == '\t'))
    {
        Pointer += 1;
    }

    Signal = 1.0f;
    if (*Pointer == '-')
    {
        Signal = -1.0f;
        Pointer += 1;
    }
    else if (*Pointer == '+')
    {
        Pointer += 1;
    }

    Value = 0.0f;
    while (((*Pointer) >= '0' && (*Pointer) <= '9'))
    {
        Value = Value * 10.0f + (*Pointer - '0');
        Pointer += 1;
    }

    if (*Pointer == '.')
    {
        float Power10 = 10.0f;
        Pointer += 1;

        while (((*Pointer) >= '0' && (*Pointer) <= '9'))
        {
            Value += (*Pointer - '0') / Power10;
            Power10 *= 10.0f;
            Pointer += 1;
        }
    }

    Scale = 1.0f;
    if ((*Pointer == 'e') || (*Pointer == 'E'))
    {
        uint16_t Exponential;
        Pointer += 1;

        Fraction = 0;
        if (*Pointer == '-')
        {
            Fraction = 1;
            Pointer += 1;
        }
        else if (*Pointer == '+')
        {
            Pointer += 1;
        }

        Exponential = 0;
        while (((*Pointer) >= '0' && (*Pointer) <= '9'))
        {
            Exponential = Exponential * 10 + (*Pointer - '0');
            Pointer += 1;
        }
        if (Exponential > 308)
        {
            Exponential = 308;
        }

        while (Exponential >= 8)
        {
            Scale *= 1E8f;
            Exponential -= 8;
        }
        while (Exponential > 0)
        {
            Scale *= 10.0f;
            Exponential -= 1;
        }
    }
    return Signal * (Fraction ? (Value / Scale) : (Value * Scale));
}