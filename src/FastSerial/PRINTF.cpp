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

#include "PRINTF.h"
#include "FASTSERIAL.h"
#include "Declination/AUTODECLINATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "InertialNavigation/INS.h"
#include "MemoryCheck/FREERAM.h"
#include "RadioControl/RCCONFIG.h"
#include "Barometer/BAROREAD.h"
#include "BatteryMonitor/BATTERY.h"
#include "RadioControl/STICKS.h"
#include "BAR/BAR.h"
#include "ProgMem/PROGMEM.h"
#include "Common/ENUM.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Common/STRUCTS.h"

SerialPrint PRINTF;

#ifdef __AVR_ATmega2560__

#define FTOA_MINUS 1
#define FTOA_ZERO 2
#define FTOA_INF 4
#define FTOA_NAN 8
#define FTOA_CARRY 16
#define XTOA_PREFIX 0x0100
#define XTOA_UPPER 0x0200
#define FL_ZFILL 0x01
#define FL_PLUS 0x02
#define FL_SPACE 0x04
#define FL_LPAD 0x08
#define FL_ALT 0x10
#define FL_WIDTH 0x20
#define FL_PREC 0x40
#define FL_LONG 0x80
#define FL_PGMSTRING FL_LONG
#define FL_NEGATIVE FL_LONG
#define FL_ALTUPP FL_PLUS
#define FL_ALTHEX FL_SPACE
#define FL_FLTUPP FL_ALT
#define FL_FLTEXP FL_PREC
#define FL_FLTFIX FL_LONG

#define ntz(x)                 \
  ((1 & (((x)&1) == 0)) +      \
   (1 & (((x)&3) == 0)) +      \
   (1 & (((x)&7) == 0)) +      \
   (1 & (((x)&017) == 0)) +    \
   (1 & (((x)&037) == 0)) +    \
   (1 & (((x)&077) == 0)) +    \
   (1 & (((x)&0177) == 0)) +   \
   (1 & (((x)&0377) == 0)) +   \
   (1 & (((x)&0777) == 0)) +   \
   (1 & (((x)&01777) == 0)) +  \
   (1 & (((x)&03777) == 0)) +  \
   (1 & (((x)&07777) == 0)) +  \
   (1 & (((x)&017777) == 0)) + \
   (1 & (((x)&037777) == 0)) + \
   (1 & (((x)&077777) == 0)) + \
   (1 & (((x)&0177777) == 0)))

#define GETBYTE(flag, mask, pnt) ({ \
  unsigned char __c;                \
  asm(                              \
      "sbrc      %2,%3   \n\t"      \
      "lpm       %0,Z+   \n\t"      \
      "sbrs      %2,%3   \n\t"      \
      "ld        %0,Z+   "          \
      : "=r"(__c),                  \
        "+z"(pnt)                   \
      : "r"(flag),                  \
        "I"(ntz(mask)));            \
  __c;                              \
})

void SerialPrint::SendToConsole(const char *fmt, ...)
{
  __gnuc_va_list ap;
  __builtin_va_start(ap, fmt);
  PRINTF.SerialPrintF(1, fmt, ap);
  __builtin_va_end(ap);
  while (!FASTSERIAL.TXFree(UART_NUMB_0))
  {
  }
}

/*
   COMANDOS PARA TAMANHOS DE VARIAVEIS:
   %0.f >> float (0.000000000000...)
   %.2f >> float (MOSTRA APENAS DUAS CASAS DECIMAIS,O NÚMERO "2" DEVE SER SUBSTITUIDO DE ACORDO COM A NECESSIDADE)
   %u   >> uint16_t
   %d   >> int16_t
   %ld  >> int32_t
*/

void SerialPrint::SerialPrintF(unsigned char in_progmem, const char *fmt, __gnuc_va_list ap)
{
  unsigned char c;
  unsigned char flags;
  unsigned char width;
  unsigned char prec;
  unsigned char buf[11];
  for (;;)
  {
    for (;;)
    {
      c = GETBYTE(in_progmem, 1, fmt);
      if (!c)
      {
        return;
      }
      if (c == '%')
      {
        c = GETBYTE(in_progmem, 1, fmt);
        if (c != '%')
        {
          break;
        }
      }
      if (c == '\n')
      {
        FASTSERIAL.Write(UART_NUMB_0, '\r');
      }
      FASTSERIAL.Write(UART_NUMB_0, c);
    }
    flags = 0;
    width = 0;
    prec = 0;
    do
    {
      if (flags < FL_WIDTH)
      {
        switch (c)
        {
        case '0':
          flags |= FL_ZFILL;
          continue;
        case '+':
          flags |= FL_PLUS;
        case ' ':
          flags |= FL_SPACE;
          continue;
        case '-':
          flags |= FL_LPAD;
          continue;
        case '#':
          flags |= FL_ALT;
          continue;
        }
      }
      if (flags < FL_LONG)
      {
        if (c >= '0' && c <= '9')
        {
          c -= '0';
          if (flags & FL_PREC)
          {
            prec = 10 * prec + c;
            continue;
          }
          width = 10 * width + c;
          flags |= FL_WIDTH;
          continue;
        }
        if (c == '.')
        {
          if (flags & FL_PREC)
            return;
          flags |= FL_PREC;
          continue;
        }
        if (c == 'l')
        {
          flags |= FL_LONG;
          continue;
        }
        if (c == 'h')
          continue;
      }
      break;
    } while ((c = GETBYTE(in_progmem, 1, fmt)) != 0);
    if (c >= 'E' && c <= 'G')
    {
      flags |= FL_FLTUPP;
      c += 'e' - 'E';
      goto flt_oper;
    }
    else if (c >= 'e' && c <= 'g')
    {
      int exp;
      int n;
      unsigned char vtype;
      unsigned char sign;
      unsigned char ndigs;
      flags &= ~FL_FLTUPP;
    flt_oper:
      if (!(flags & FL_PREC))
      {
        prec = 6;
      }
      flags &= ~(FL_FLTEXP | FL_FLTFIX);
      if (c == 'e')
      {
        flags |= FL_FLTEXP;
      }
      else if (c == 'f')
      {
        flags |= FL_FLTFIX;
      }
      else if (prec > 0)
      {
        prec -= 1;
      }
      if (flags & FL_FLTFIX)
      {
        vtype = 7;
        ndigs = prec < 60 ? prec + 1 : 60;
      }
      else
      {
        if (prec > 7)
          prec = 7;
        vtype = prec;
        ndigs = 0;
      }
      exp = __ftoa_engine(va_arg(ap, double), (char *)buf, vtype, ndigs);
      vtype = buf[0];
      sign = 0;
      if ((vtype & FTOA_MINUS) && !(vtype & FTOA_NAN))
      {
        sign = '-';
      }
      else if (flags & FL_PLUS)
      {
        sign = '+';
      }
      else if (flags & FL_SPACE)
      {
        sign = ' ';
      }
      if (vtype & (FTOA_NAN | FTOA_INF))
      {
        const char *p;
        ndigs = sign ? 4 : 3;
        if (width > ndigs)
        {
          width -= ndigs;
          if (!(flags & FL_LPAD))
          {
            do
            {
              FASTSERIAL.Write(UART_NUMB_0, ' ');
            } while (--width);
          }
        }
        else
        {
          width = 0;
        }
        if (sign)
        {
          FASTSERIAL.Write(UART_NUMB_0, sign);
        }
        p = ProgramMemoryString("inf");
        if (vtype & FTOA_NAN)
        {
          p = ProgramMemoryString("nan");
        }
        while ((ndigs = ProgMemReadByte(p)) != 0)
        {
          if (flags & FL_FLTUPP)
            ndigs += 'I' - 'i';
          FASTSERIAL.Write(UART_NUMB_0, ndigs);
          p++;
        }
        goto tail;
      }
      if (flags & FL_FLTFIX)
      {
        ndigs += exp;
        if ((vtype & FTOA_CARRY) && buf[1] == '1')
        {
          ndigs -= 1;
        }
        if ((signed char)ndigs < 1)
        {
          ndigs = 1;
        }
        else if (ndigs > 8)
        {
          ndigs = 8;
        }
      }
      else if (!(flags & FL_FLTEXP))
      {
        if (exp <= prec && exp >= -4)
        {
          flags |= FL_FLTFIX;
        }
        while (prec && buf[1 + prec] == '0')
        {
          prec--;
        }
        if (flags & FL_FLTFIX)
        {
          ndigs = prec + 1;
          prec = prec > exp
                     ? prec - exp
                     : 0;
        }
      }
      if (flags & FL_FLTFIX)
      {
        n = (exp > 0 ? exp + 1 : 1);
      }
      else
        n = 5;
      if (sign)
      {
        n += 1;
      }
      if (prec)
      {
        n += prec + 1;
      }
      width = width > n ? width - n : 0;
      if (!(flags & (FL_LPAD | FL_ZFILL)))
      {
        while (width)
        {
          FASTSERIAL.Write(UART_NUMB_0, ' ');
          width--;
        }
      }
      if (sign)
        FASTSERIAL.Write(UART_NUMB_0, sign);
      if (!(flags & FL_LPAD))
      {
        while (width)
        {
          FASTSERIAL.Write(UART_NUMB_0, '0');
          width--;
        }
      }
      if (flags & FL_FLTFIX)
      {
        n = exp > 0 ? exp : 0;
        do
        {
          if (n == -1)
          {
            FASTSERIAL.Write(UART_NUMB_0, '.');
          }
          flags = (n <= exp && n > exp - ndigs)
                      ? buf[exp - n + 1]
                      : '0';
          if (--n < -prec)
          {
            break;
          }
          FASTSERIAL.Write(UART_NUMB_0, flags);
        } while (1);
        if (n == exp && (buf[1] > '5' || (buf[1] == '5' && !(vtype & FTOA_CARRY))))
        {
          flags = '1';
        }
        FASTSERIAL.Write(UART_NUMB_0, flags);
      }
      else
      {
        if (buf[1] != '1')
          vtype &= ~FTOA_CARRY;
        FASTSERIAL.Write(UART_NUMB_0, buf[1]);
        if (prec)
        {
          FASTSERIAL.Write(UART_NUMB_0, '.');
          sign = 2;
          do
          {
            FASTSERIAL.Write(UART_NUMB_0, buf[sign++]);
          } while (--prec);
        }
        FASTSERIAL.Write(UART_NUMB_0, flags & FL_FLTUPP ? 'E' : 'e');
        ndigs = '+';
        if (exp < 0 || (exp == 0 && (vtype & FTOA_CARRY) != 0))
        {
          exp = -exp;
          ndigs = '-';
        }
        FASTSERIAL.Write(UART_NUMB_0, ndigs);
        for (ndigs = '0'; exp >= 10; exp -= 10)
        {
          ndigs += 1;
        }
        FASTSERIAL.Write(UART_NUMB_0, ndigs);
        FASTSERIAL.Write(UART_NUMB_0, '0' + exp);
      }
      goto tail;
    }

    const char *pnt;
    size_t size;
    switch (c)
    {

    case 'c':
      buf[0] = va_arg(ap, int);
      pnt = (char *)buf;
      size = 1;
      goto no_pgmstring;

    case 's':
      pnt = va_arg(ap, char *);
      size = strnlen(pnt, (flags & FL_PREC) ? prec : ~0);
    no_pgmstring:
      flags &= ~FL_PGMSTRING;
      goto str_lpad;

    case 'S':
    str_lpad:
      if (!(flags & FL_LPAD))
      {
        while (size < width)
        {
          FASTSERIAL.Write(UART_NUMB_0, ' ');
          width--;
        }
      }
      while (size)
      {
        FASTSERIAL.Write(UART_NUMB_0, GETBYTE(flags, FL_PGMSTRING, pnt));
        if (width)
          width -= 1;
        size -= 1;
      }
      goto tail;
    }

    if (c == 'd' || c == 'i')
    {
      long x = (flags & FL_LONG) ? va_arg(ap, long) : va_arg(ap, int);
      flags &= ~(FL_NEGATIVE | FL_ALT);
      if (x < 0)
      {
        x = -x;
        flags |= FL_NEGATIVE;
      }
      c = __ultoa_invert(x, (char *)buf, 10) - (char *)buf;
    }
    else
    {
      int base;

      if (c == 'u')
      {
        flags &= ~FL_ALT;
        base = 10;
        goto ultoa;
      }

      flags &= ~(FL_PLUS | FL_SPACE);

      switch (c)
      {

      case 'o':
        base = 8;
        goto ultoa;

      case 'p':
        flags |= FL_ALT;

      case 'x':
        if (flags & FL_ALT)
          flags |= FL_ALTHEX;
        base = 16;
        goto ultoa;

      case 'X':
        if (flags & FL_ALT)
          flags |= (FL_ALTHEX | FL_ALTUPP);
        base = 16 | XTOA_UPPER;
      ultoa:
        c = __ultoa_invert((flags & FL_LONG)
                               ? va_arg(ap, unsigned long)
                               : va_arg(ap, unsigned int),
                           (char *)buf, base) -
            (char *)buf;
        flags &= ~FL_NEGATIVE;
        break;

      default:
        return;
      }
    }
    unsigned char len;
    len = c;
    if (flags & FL_PREC)
    {
      flags &= ~FL_ZFILL;
      if (len < prec)
      {
        len = prec;
        if ((flags & FL_ALT) && !(flags & FL_ALTHEX))
        {
          flags &= ~FL_ALT;
        }
      }
    }
    if (flags & FL_ALT)
    {
      if (buf[c - 1] == '0')
      {
        flags &= ~(FL_ALT | FL_ALTHEX | FL_ALTUPP);
      }
      else
      {
        len += 1;
        if (flags & FL_ALTHEX)
          len += 1;
      }
    }
    else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE))
    {
      len += 1;
    }
    if (!(flags & FL_LPAD))
    {
      if (flags & FL_ZFILL)
      {
        prec = c;
        if (len < width)
        {
          prec += width - len;
          len = width;
        }
      }
      while (len < width)
      {
        FASTSERIAL.Write(UART_NUMB_0, ' ');
        len++;
      }
    }
    width = (len < width) ? width - len : 0;
    if (flags & FL_ALT)
    {
      FASTSERIAL.Write(UART_NUMB_0, '0');
      if (flags & FL_ALTHEX)
      {
        FASTSERIAL.Write(UART_NUMB_0, flags & FL_ALTUPP ? 'X' : 'x');
      }
    }
    else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE))
    {
      unsigned char z = ' ';
      if (flags & FL_PLUS)
      {
        z = '+';
      }
      if (flags & FL_NEGATIVE)
      {
        z = '-';
      }
      FASTSERIAL.Write(UART_NUMB_0, z);
    }
    while (prec > c)
    {
      FASTSERIAL.Write(UART_NUMB_0, '0');
      prec--;
    }
    do
    {
      FASTSERIAL.Write(UART_NUMB_0, buf[--c]);
    } while (c);
  tail:
    while (width)
    {
      FASTSERIAL.Write(UART_NUMB_0, ' ');
      width--;
    }
  }
}

#endif

typedef void (*putcf)(void *, char);
static putcf stdout_putf;
static void *stdout_putp;

static void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
  int n = 0;
  unsigned int d = 1;
  while (num / d >= base)
    d *= base;
  while (d != 0)
  {
    int dgt = num / d;
    num %= d;
    d /= base;
    if (n || dgt > 0 || d == 0)
    {
      *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
      ++n;
    }
  }
  *bf = 0;
}

static void li2a(long num, char *bf)
{
  if (num < 0)
  {
    num = -num;
    *bf++ = '-';
  }
  uli2a(num, 10, 0, bf);
}

static void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
  int n = 0;
  unsigned int d = 1;
  while (num / d >= base)
    d *= base;
  while (d != 0)
  {
    int dgt = num / d;
    num %= d;
    d /= base;
    if (n || dgt > 0 || d == 0)
    {
      *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
      ++n;
    }
  }
  *bf = 0;
}

static void i2a(int num, char *bf)
{
  if (num < 0)
  {
    num = -num;
    *bf++ = '-';
  }
  ui2a(num, 10, 0, bf);
}

static int a2d(char ch)
{
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  else if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  else if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  else
    return -1;
}

static char a2i(char ch, char **src, int base, int *nump)
{
  char *p = *src;
  int num = 0;
  int digit;
  while ((digit = a2d(ch)) >= 0)
  {
    if (digit > base)
      break;
    num = num * base + digit;
    ch = *p++;
  }
  *src = p;
  *nump = num;
  return ch;
}

static void putchw(void *putp, putcf putf, int n, char z, char *bf)
{
  char fc = z ? '0' : ' ';
  char ch;
  char *p = bf;
  while (*p++ && n > 0)
    n--;
  while (n-- > 0)
    putf(putp, fc);
  while ((ch = *bf++))
    putf(putp, ch);
}

void tfp_format(void *putp, putcf putf, char *fmt, va_list va)
{
  char bf[12];

  char ch;

  while ((ch = *(fmt++)))
  {
    if (ch != '%')
      putf(putp, ch);
    else
    {
      char lz = 0;
      char lng = 0;
      int w = 0;
      ch = *(fmt++);
      if (ch == '0')
      {
        ch = *(fmt++);
        lz = 1;
      }

      if (ch >= '0' && ch <= '9')
      {
        ch = a2i(ch, &fmt, 10, &w);
      }

      if (ch == 'l')
      {
        ch = *(fmt++);
        lng = 1;
      }

      switch (ch)
      {

      case 0:
        goto abort;

      case 'u':
      {
        if (lng)
          uli2a(va_arg(va, unsigned long int), 10, 0, bf);
        else
          ui2a(va_arg(va, unsigned int), 10, 0, bf);
        putchw(putp, putf, w, lz, bf);
        break;
      }

      case 'd':
      {
        if (lng)
          li2a(va_arg(va, unsigned long int), bf);
        else
          i2a(va_arg(va, int), bf);
        putchw(putp, putf, w, lz, bf);
        break;
      }

      case 'x':
      case 'X':
        if (lng)
          uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
        else
          ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
        putchw(putp, putf, w, lz, bf);
        break;

      case 'c':
        putf(putp, (char)(va_arg(va, int)));
        break;

      case 's':
        putchw(putp, putf, w, 0, va_arg(va, char *));
        break;

      case '%':
        putf(putp, ch);
      default:
        break;
      }
    }
  }
abort:;
}

void _putc(void *p, char c)
{
  (void)p;
  FASTSERIAL.Write(UART_NUMB_0, c);
}

void init_printf(void *putp, void (*putf)(void *, char))
{
  stdout_putf = putf;
  stdout_putp = putp;
}

void SerialPrint::Initialization(void)
{
#ifndef __AVR_ATmega2560__
  init_printf(NULL, _putc);
#endif
}

void SerialPrint::tfp_printf(char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  tfp_format(stdout_putp, stdout_putf, fmt, va);
  va_end(va);
  while (!FASTSERIAL.TXFree(UART_NUMB_0))
  {
  }
}