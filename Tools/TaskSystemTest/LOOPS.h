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

typedef enum
{
  TASK_SLOW_LOOP = 0,
  TASK_MEDIUM_LOOP,
  TASK_FAST_LOOP,
  TASK_SYSTEM_LOAD,
  //TASK COUNT SEMPRE EM ÚLTIMO LUGAR
  TASK_COUNT
} Tasks_ID_Enum;

void Slow_Loop();
void Medium_Loop();
void Fast_Loop();
