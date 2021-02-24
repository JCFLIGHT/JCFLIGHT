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

#ifndef RCDEFINES_H_
#define RCDEFINES_H_
#include "BitArray/BITARRAY.h"
#include "RadioControl/DECODE.h"
#define RANGE_MIN 900
#define MIN_PULSE 1100
#define MAX_PULSE 1900
#define MIN_STICKS_PULSE 1000
#define MAX_STICKS_PULSE 2000
#define MIDDLE_STICKS_PULSE (MIN_STICKS_PULSE + MAX_STICKS_PULSE) / 2
#define DISABLE_IO_PIN 0
//AUX1
#define AUX1_LOW DECODE.GetRxChannelOutput(AUX1) < 1100
#define AUX1_MID DECODE.GetRxChannelOutput(AUX1) > 1400 && DECODE.GetRxChannelOutput(AUX1) < 1600
#define AUX1_HIGH DECODE.GetRxChannelOutput(AUX1) > 1900
//AUX2
#define AUX2_LOW DECODE.GetRxChannelOutput(AUX2) < 1100
#define AUX2_MID DECODE.GetRxChannelOutput(AUX2) > 1400 && DECODE.GetRxChannelOutput(AUX2) < 1600
#define AUX2_HIGH DECODE.GetRxChannelOutput(AUX2) > 1900
//AUX3
#define AUX3_LOW DECODE.GetRxChannelOutput(AUX3) < 1100
#define AUX3_MID DECODE.GetRxChannelOutput(AUX3) > 1400 && DECODE.GetRxChannelOutput(AUX3) < 1600
#define AUX3_HIGH DECODE.GetRxChannelOutput(AUX3) > 1900
//AUX4
#define AUX4_LOW DECODE.GetRxChannelOutput(AUX4) < 1100
#define AUX4_MID DECODE.GetRxChannelOutput(AUX4) > 1400 && DECODE.GetRxChannelOutput(AUX4) < 1600
#define AUX4_HIGH DECODE.GetRxChannelOutput(AUX4) > 1900
//AUX5
#define AUX5_LOW DECODE.GetRxChannelOutput(AUX5) < 1100
#define AUX5_MID DECODE.GetRxChannelOutput(AUX5) > 1400 && DECODE.GetRxChannelOutput(AUX5) < 1600
#define AUX5_HIGH DECODE.GetRxChannelOutput(AUX5) > 1900
//AUX6
#define AUX6_LOW DECODE.GetRxChannelOutput(AUX6) < 1100
#define AUX6_MID DECODE.GetRxChannelOutput(AUX6) > 1400 && DECODE.GetRxChannelOutput(AUX6) < 1600
#define AUX6_HIGH DECODE.GetRxChannelOutput(AUX6) > 1900
//AUX7
#define AUX7_LOW DECODE.GetRxChannelOutput(AUX7) < 1100
#define AUX7_MID DECODE.GetRxChannelOutput(AUX7) > 1400 && DECODE.GetRxChannelOutput(AUX7) < 1600
#define AUX7_HIGH DECODE.GetRxChannelOutput(AUX7) > 1900
//AUX8
#define AUX8_LOW DECODE.GetRxChannelOutput(AUX8) < 1100
#define AUX8_MID DECODE.GetRxChannelOutput(AUX8) > 1400 && DECODE.GetRxChannelOutput(AUX8) < 1600
#define AUX8_HIGH DECODE.GetRxChannelOutput(AUX8) > 1900
#endif