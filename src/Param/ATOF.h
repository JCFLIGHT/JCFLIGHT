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

#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')

static float _atof(const char *p)
{
    int frac = 0;
    float sign, value, scale;

    while (white_space(*p))
    {
        p += 1;
    }

    sign = 1.0f;
    if (*p == '-')
    {
        sign = -1.0f;
        p += 1;
    }
    else if (*p == '+')
    {
        p += 1;
    }

    value = 0.0f;
    while (valid_digit(*p))
    {
        value = value * 10.0f + (*p - '0');
        p += 1;
    }

    if (*p == '.')
    {
        float pow10 = 10.0f;
        p += 1;

        while (valid_digit(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p += 1;
        }
    }

    scale = 1.0f;
    if ((*p == 'e') || (*p == 'E'))
    {
        unsigned int expon;
        p += 1;

        frac = 0;
        if (*p == '-')
        {
            frac = 1;
            p += 1;
        }
        else if (*p == '+')
        {
            p += 1;
        }

        expon = 0;
        while (valid_digit(*p))
        {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308)
        {
            expon = 308;
        }

        while (expon >= 8)
        {
            scale *= 1E8f;
            expon -= 8;
        }
        while (expon > 0)
        {
            scale *= 10.0f;
            expon -= 1;
        }
    }
    return sign * (frac ? (value / scale) : (value * scale));
}