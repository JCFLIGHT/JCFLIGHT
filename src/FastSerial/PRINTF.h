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

#ifndef PRINTF_H_
#define PRINTF_H_
#include "Arduino.h"
#ifdef __AVR_ATmega2560__
extern "C"
{
  int __ftoa_engine(double val, char *buf, unsigned char prec, unsigned char maxdgs);
  char *__ultoa_invert(unsigned long val, char *s, int base);
}
#endif
class SerialPrint
{
public:
  void ParamsToConsole();
#ifndef __AVR_ATmega2560__
  void SendToConsole();
#else
  void SendToConsole(const char *fmt, ...);
#endif
  void CallSchedulerSleep();

private:
#ifndef __AVR_ATmega2560__
  void SerialPrintF();
#else
  void SerialPrintF(unsigned char in_progmem, const char *fmt, __gnuc_va_list ap);
#endif
};
extern SerialPrint PRINTF;
#ifdef __AVR_ATmega2560__
#define DEBUG(fmt, args...)                       \
  do                                              \
  {                                               \
    PRINTF.SendToConsole(PSTR(fmt "\n"), ##args); \
    PRINTF.CallSchedulerSleep();                  \
  } while (0)
#define LOG(fmt)                                                                        \
  do                                                                                    \
  {                                                                                     \
    PRINTF.SendToConsole(PSTR("Funcao:%s Linha:%d " fmt "\n"), __FUNCTION__, __LINE__); \
    PRINTF.CallSchedulerSleep();                                                        \
  } while (0)
#else
#define DEBUG(x, y) (void)(x), (void)(y)
#define LOG(x) (void)(x)
#endif
#endif
