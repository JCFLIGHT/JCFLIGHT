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

#include "PT1.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//PT1:
//P = PONTOS
//T = DURAÇÃO
//1 = PRIMEIRA ORDEM

void PT1FilterInit(PT1_Filter_Struct *Filter, float CutOffFrequency, float DeltaTime)
{
  //CALCULA O VALOR RC DO FILTRO
  //ESTAMOS CALCULANDO UM RESISTOR E UM CAPACITOR DIGITALMENTE
  Filter->RC = 1.0f / (6.283185307179586476925286766559f * CutOffFrequency);
  //OBTÉM O DELTA TIME
  Filter->DeltaTime = DeltaTime;
}

float PT1FilterApply(PT1_Filter_Struct *Filter, float Input, float CutOffFrequency, float DeltaTime)
{
  //CALCULA O VALOR RC DO FILTRO
  //ESTAMOS CALCULANDO UM RESISTOR E UM CAPACITOR DIGITALMENTE
  Filter->RC = 1.0f / (6.283185307179586476925286766559f * CutOffFrequency);
  //OBTÉM O DELTA TIME
  Filter->DeltaTime = DeltaTime;
  //CALCULA O VALOR DO FILTRO
  Filter->State = Filter->State + DeltaTime / (Filter->RC + DeltaTime) * (Input - Filter->State);
  //RETORNA O VALOR FILTRADO
  return Filter->State;
}

float PT1FilterApply2(PT1_Filter_Struct *Filter, float Input, float DeltaTime)
{
  //OBTÉM O DELTA TIME
  Filter->DeltaTime = DeltaTime;
  //CALCULA O VALOR DO FITLRO
  Filter->State = Filter->State + DeltaTime / (Filter->RC + DeltaTime) * (Input - Filter->State);
  //RETORNA O VALOR FILTRADO
  return Filter->State;
}

float PT1FilterApply3(PT1_Filter_Struct *Filter, float Input)
{
  //CALCULA O VALOR DO FILTRO
  Filter->State = Filter->State + Filter->DeltaTime / (Filter->RC + Filter->DeltaTime) * (Input - Filter->State);
  //RETORNA O VALOR FILTRADO
  return Filter->State;
}