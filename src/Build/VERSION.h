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

#ifndef VERSION_H_
#define VERSION_H_
#ifdef __AVR_ATmega2560__
extern const char PlatformName[];
extern const char FirwareName[];
extern const char FirmwareVersion[];
extern const char CompilerVersion[];
extern const char BuildDate[];
extern const char BuildTime[];
#elif defined __arm__
extern const char *const PlatformName;
extern const char *const FirwareName;
extern const char *const FirmwareVersion;
extern const char *const CompilerVersion;
extern const char *const BuildDate;
extern const char *const BuildTime;
#endif
#endif