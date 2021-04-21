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

#include "Common/ENUM.h"
#include "Targets/PASCAL.h"
#include "Targets/EXTREME.h"
#include "Targets/CLASSIC.h"

#ifdef __AVR_ATmega2560__

//NÃO USA ALGUNS RECURSOS NA JCFLIGHT-CLASSIC POR MOTIVO DE FALTA DE MEMORIA RAM

#undef USE_NAZA_GPS
#undef USE_DERIVATIVE_BOOST_PID
#undef USE_AIRSPEED_AUTO_SCALE_CALIBRATION
#undef USE_WIND_ESTIMATOR

#else

#define USE_NAZA_GPS
#define USE_DERIVATIVE_BOOST_PID
#define USE_AIRSPEED_AUTO_SCALE_CALIBRATION
#define USE_WIND_ESTIMATOR

#endif