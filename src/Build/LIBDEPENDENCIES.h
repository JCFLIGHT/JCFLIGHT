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

#ifndef LIBDEPENDENCIES_H_
#define LIBDEPENDENCIES_H_

#ifdef __AVR_ATmega2560__

#include <avr/io.h>       //PARA MANIPULAÇÃO DE REGISTRADORES
#include <avr/pgmspace.h> //strlen_P (STRING LENGTH COM PONTEIRO)

//PARA O AVR:ALOQUE NA MEMORIA FLASH AFIM DE OCUPAR MENOS MEMORIA RAM
#define ProgramMemoryString(StringToArray) (__extension__({static const char __CharArray[] __attribute__((__progmem__)) = (StringToArray); &__CharArray[0]; }))

#else

#include "Arduino.h" //MANTÉM POR ENQUATO

#define ProgramMemoryString(StringToArray) (StringToArray)

#ifndef strlcat_P
#define strlcat_P(dest, src, len) strlcat((dest), (src), (len))
#endif

#endif

#include <inttypes.h> //int8_t,uint8_t,int16_t,uint16_t,int32_t & uint32_t
#include <math.h>     //isnan & isinf
#include <stdarg.h>   //__gnuc_va_list
#include <stdio.h>    //size_t
#include <stdlib.h>   //malloc & free
#include <string.h>   //strnlen (STRING LENGTH COM DEFINIÇÃO DE TAMANHO)

#undef NULL
#ifndef NULL
#define NULL 0
#endif

#endif