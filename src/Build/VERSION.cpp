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

#include "VERSION.h"
#include "ProgMem/PROGMEM.h"

#ifdef __AVR_ATmega2560__
#define FC_FIRMWARE_NAME "JCFLIGHT-CLASSIC"
#define FC_VERSION_MAJOR 1
#define FC_VERSION_MINOR 0
#define FC_VERSION_PATCH_LEVEL 0
#elif defined STM32F103xB
#define FC_FIRMWARE_NAME "JCFLIGHT-ZION"
#define FC_VERSION_MAJOR 1
#define FC_VERSION_MINOR 0
#define FC_VERSION_PATCH_LEVEL 0
#elif defined STM32F407xx
#define FC_FIRMWARE_NAME "JCFLIGHT-PASCAL"
#define FC_VERSION_MAJOR 1
#define FC_VERSION_MINOR 0
#define FC_VERSION_PATCH_LEVEL 0
#elif defined ESP32
#define FC_FIRMWARE_NAME "JCFLIGHT-EXTREME"
#define FC_VERSION_MAJOR 1
#define FC_VERSION_MINOR 0
#define FC_VERSION_PATCH_LEVEL 0
#endif
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING \
  STR(FC_VERSION_MAJOR)   \
  "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)
#define SEPARATOR ";"

#ifdef __AVR_ATmega2560__
const char PlatformName[] FLASH_MEMORY_ATTRIBUTE = "AVR" SEPARATOR;
const char FirwareName[] FLASH_MEMORY_ATTRIBUTE = FC_FIRMWARE_NAME SEPARATOR;
const char FirmwareVersion[] FLASH_MEMORY_ATTRIBUTE = FC_VERSION_STRING SEPARATOR;
const char CompilerVersion[] FLASH_MEMORY_ATTRIBUTE = __VERSION__ SEPARATOR;
const char BuildDate[] FLASH_MEMORY_ATTRIBUTE = __DATE__ SEPARATOR;
const char BuildTime[] FLASH_MEMORY_ATTRIBUTE = __TIME__ SEPARATOR;
#elif defined __arm__ || defined ESP32
#ifdef __arm__
const char *const PlatformName = "STM32" SEPARATOR;
#elif defined ESP32
const char *const PlatformName = "ESP32" SEPARATOR;
#endif
const char *const FirwareName = FC_FIRMWARE_NAME SEPARATOR;
const char *const FirmwareVersion = FC_VERSION_STRING SEPARATOR;
const char *const CompilerVersion = __VERSION__ SEPARATOR;
const char *const BuildDate = __DATE__ SEPARATOR;
const char *const BuildTime = __TIME__ SEPARATOR;
#endif