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
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
#include "ProgMem/PROGMEM.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

static const PID_Mixer_Struct PID_Mixer_Quad_X[] FLASH_MEMORY_ATTRIBUTE = {
    {+1.0f, -1.0f, -1.0f}, //MOTOR 1
    {+1.0f, +1.0f, +1.0f}, //MOTOR 2
    {-1.0f, -1.0f, +1.0f}, //MOTOR 3
    {-1.0f, +1.0f, -1.0f}, //MOTOR 4
    //ROLL   PITCH  YAW
};

static const PID_Mixer_Struct PID_Mixer_Hexa_X[] FLASH_MEMORY_ATTRIBUTE = {
    {+0.8f, -0.9f, -1.0f}, //MOTOR 1
    {+0.8f, +0.9f, -1.0f}, //MOTOR 2
    {-0.8f, -0.9f, +1.0f}, //MOTOR 3
    {-0.8f, +0.9f, +1.0f}, //MOTOR 4
    {+0.8f, +0.0f, +1.0f}, //MOTOR 5
    {-0.8f, +0.0f, -1.0f}, //MOTOR 6
    //ROLL   PITCH  YAW
};

static const PID_Mixer_Struct PID_Mixer_Hexa_I[] FLASH_MEMORY_ATTRIBUTE = {
    {+0.9f, -0.8f, -1.0f}, //MOTOR 1
    {+0.9f, +0.8f, +1.0f}, //MOTOR 2
    {-0.9f, -0.8f, -1.0f}, //MOTOR 3
    {-0.9f, +0.8f, +1.0f}, //MOTOR 4
    {+0.0f, +0.8f, -1.0f}, //MOTOR 5
    {+0.0f, -0.8f, +1.0f}, //MOTOR 6
    //ROLL   PITCH  YAW
};

static const PID_Mixer_Struct PID_Mixer_ZMR250[] FLASH_MEMORY_ATTRIBUTE = {
    {+1.0f, -0.772f, -1.0f}, //MOTOR 1
    {+1.0f, +0.772f, +1.0f}, //MOTOR 2
    {-1.0f, -0.772f, +1.0f}, //MOTOR 3
    {-1.0f, +0.772f, -1.0f}, //MOTOR 4
    //ROLL   PITCH    YAW
};

static const PID_Mixer_Struct PID_Mixer_TBS[] FLASH_MEMORY_ATTRIBUTE = {
    {+1.0f,   -0.647f, -1.0f}, //MOTOR 1
    {+0.848f, +0.647f, +1.0f}, //MOTOR 2
    {-1.0f,   -0.647f, +1.0f}, //MOTOR 3
    {-0.848f, +0.647f, -1.0f}, //MOTOR 4
    //ROLL     PITCH    YAW
};

static const Motors_Count_Struct Motors_Count[] FLASH_MEMORY_ATTRIBUTE = {
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