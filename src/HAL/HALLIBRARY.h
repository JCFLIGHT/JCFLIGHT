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

#ifndef HALLIBRARY_H_
#define HALLIBRARY_H_

#ifdef __AVR_ATmega2560__
#include "HAL_AVR/AVRADC.h"
#include "HAL_AVR/AVREEPROM.h"
#include "HAL_AVR/AVRFREERAM.h"
#include "HAL_AVR/AVRPPM.h"
#include "HAL_AVR/AVRSERIAL.h"
#endif

#ifdef ESP32
#include "HAL_ESP32/ESP32ADC.h"
#include "HAL_ESP32/ESP32EEPROM.h"
#include "HAL_ESP32/ESP32FREERAM.h"
#include "HAL_ESP32/ESP32PPM.h"
#include "HAL_ESP32/ESP32SERIAL.h"
#endif

#ifdef __arm__
#include "HAL_STM32/STM32ADC.h"
#include "HAL_STM32/STM32EEPROM.h"
#include "HAL_STM32/STM32FREERAM.h"
#include "HAL_STM32/STM32PPM.h"
#include "HAL_STM32/STM32SERIAL.h"
#endif

#endif