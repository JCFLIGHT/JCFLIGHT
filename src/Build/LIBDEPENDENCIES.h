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
#define NULL 0
#define PSTR(String) (__extension__({static const char __CharArray[] __attribute__((__progmem__)) = (String); &__CharArray[0]; }))
#include <inttypes.h>     //int8_t,uint8_t,int16_t,uint16_t,int32_t & uint32_t
#include <math.h>         //isnan & isinf
#include <stdarg.h>       //va_list
#include <stdio.h>        //size_t
#include <stdlib.h>       //malloc & free
#include <string.h>       //strnlen (STRING LENGTH COM DEFINIÇÃO DE TAMANHO)
#include <avr/io.h>       //PARA MANIPULAÇÃO DE REGISTRADORES
#include <avr/pgmspace.h> //strlen_P (STRING LENGTH COM PONTEIRO)
/*
//ERA PRA FUNCIONAR NÉ?!MAS ENFIM...
static inline size_t strlen_P(const char *pstr) //STRING LENGTH COM PONTEIRO
{
  return strlen((const char *)pstr);
}
*/
#endif