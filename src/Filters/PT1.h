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

#ifndef PT1_H_
#define PT1_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
void PT1FilterInit(PT1_Filter_Struct *Filter, float CutOffFrequency, float DeltaTime);
float PT1FilterApply(PT1_Filter_Struct *Filter, float Input, float CutOffFrequency, float DeltaTime);
float PT1FilterApply2(PT1_Filter_Struct *Filter, float Input, float DeltaTime);
float PT1FilterApply3(PT1_Filter_Struct *Filter, float Input);
#endif