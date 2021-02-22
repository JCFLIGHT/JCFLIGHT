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

#ifndef MIXTABLE_H_
#define MIXTABLE_H_
#include "Arduino.h"
#include "Common/STRUCTS.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

#ifdef __AVR_ATmega2560__

static const PID_Mixer_Struct Pid_Mixer_Quad_X[] __attribute__((__progmem__)) = {
    {+1.0f, -1.0f, -1.0f}, //MOTOR 1
    {+1.0f, +1.0f, +1.0f}, //MOTOR 2
    {-1.0f, -1.0f, +1.0f}, //MOTOR 3
    {-1.0f, +1.0f, -1.0f}, //MOTOR 4
};

static const PID_Mixer_Struct Pid_Mixer_Hexa_X[] __attribute__((__progmem__)) = {
    {+0.8f, -0.9f, -1.0f}, //MOTOR 1
    {+0.8f, +0.9f, -1.0f}, //MOTOR 2
    {-0.8f, -0.9f, +1.0f}, //MOTOR 3
    {-0.8f, +0.9f, +1.0f}, //MOTOR 4
    {+0.8f, +0.0f, +1.0f}, //MOTOR 5
    {-0.8f, +0.0f, -1.0f}, //MOTOR 6
};

static const PID_Mixer_Struct Pid_Mixer_Hexa_I[] __attribute__((__progmem__)) = {
    {+0.9f, -0.8f, -1.0f}, //MOTOR 1
    {+0.9f, +0.8f, +1.0f}, //MOTOR 2
    {-0.9f, -0.8f, -1.0f}, //MOTOR 3
    {-0.9f, +0.8f, +1.0f}, //MOTOR 4
    {+0.0f, +0.8f, -1.0f}, //MOTOR 5
    {+0.0f, -0.8f, +1.0f}, //MOTOR 6
};

static const PID_Mixer_Struct Pid_Mixer_ZMR250[] __attribute__((__progmem__)) = {
    {+1.0f, -0.772f, -1.0f}, //MOTOR 1
    {+1.0f, +0.772f, +1.0f}, //MOTOR 2
    {-1.0f, -0.772f, +1.0f}, //MOTOR 3
    {-1.0f, +0.772f, -1.0f}, //MOTOR 4
};

static const PID_Mixer_Struct Pid_Mixer_TBS[] __attribute__((__progmem__)) = {
    {+1.0f, -0.647f, -1.0f},   //MOTOR 1
    {+0.848f, +0.647f, +1.0f}, //MOTOR 2
    {-1.0f, -0.647f, +1.0f},   //MOTOR 3
    {-0.848f, +0.647f, -1.0f}, //MOTOR 4
};

static const Motors_Count_Struct Motors_Count[] __attribute__((__progmem__)) = {
    {4}, //QUAD X
    {6}, //HEXA X
    {6}, //HEXA I
    {1}, //AEROMODELO
    {1}, //ASA-FIXA
    {1}, //AEROMODELO DO TIPO V-TAIL
    {4}, //ZMR250
    {4}, //TBS
};

#elif defined __arm__ || defined ESP32

static const PID_Mixer_Struct Pid_Mixer_Quad_X[] = {
    {+1.0f, -1.0f, -1.0f}, //MOTOR 1
    {+1.0f, +1.0f, +1.0f}, //MOTOR 2
    {-1.0f, -1.0f, +1.0f}, //MOTOR 3
    {-1.0f, +1.0f, -1.0f}, //MOTOR 4
};

static const PID_Mixer_Struct Pid_Mixer_Hexa_X[] = {
    {+0.8f, -0.9f, -1.0f}, //MOTOR 1
    {+0.8f, +0.9f, -1.0f}, //MOTOR 2
    {-0.8f, -0.9f, +1.0f}, //MOTOR 3
    {-0.8f, +0.9f, +1.0f}, //MOTOR 4
    {+0.8f, +0.0f, +1.0f}, //MOTOR 5
    {-0.8f, +0.0f, -1.0f}, //MOTOR 6
};

static const PID_Mixer_Struct Pid_Mixer_Hexa_I[] = {
    {+0.9f, -0.8f, -1.0f}, //MOTOR 1
    {+0.9f, +0.8f, +1.0f}, //MOTOR 2
    {-0.9f, -0.8f, -1.0f}, //MOTOR 3
    {-0.9f, +0.8f, +1.0f}, //MOTOR 4
    {+0.0f, +0.8f, -1.0f}, //MOTOR 5
    {+0.0f, -0.8f, +1.0f}, //MOTOR 6
};

static const PID_Mixer_Struct Pid_Mixer_ZMR250[] = {
    {+1.0f, -0.772f, -1.0f}, //MOTOR 1
    {+1.0f, +0.772f, +1.0f}, //MOTOR 2
    {-1.0f, -0.772f, +1.0f}, //MOTOR 3
    {-1.0f, +0.772f, -1.0f}, //MOTOR 4
};

static const PID_Mixer_Struct Pid_Mixer_TBS[] = {
    {+1.0f, -0.647f, -1.0f},   //MOTOR 1
    {+0.848f, +0.647f, +1.0f}, //MOTOR 2
    {-1.0f, -0.647f, +1.0f},   //MOTOR 3
    {-0.848f, +0.647f, -1.0f}, //MOTOR 4
};

static const Motors_Count_Struct Motors_Count[] = {
    {4}, //QUAD X
    {6}, //HEXA X
    {6}, //HEXA I
    {1}, //AEROMODELO
    {1}, //ASA-FIXA
    {1}, //AEROMODELO DO TIPO V-TAIL
    {4}, //ZMR250
    {4}, //TBS
};

#endif

#endif