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

//PT1:
//P = PONTOS
//T = DURAÇÃO
//1 = PRIMEIRA ORDEM

void PT1FilterInitRC(PT1_Filter_Struct *Filter, float CutOff, float DeltaTime)
{
    Filter->State = 0.0f;
    Filter->RC = CutOff;
    Filter->DeltaTime = DeltaTime;
}

void PT1FilterInit(PT1_Filter_Struct *Filter, float CutOff, float DeltaTime)
{
    PT1FilterInitRC(Filter, 1.0f / (2.0f * 3.14159265358979323846f * CutOff), DeltaTime);
}

float PT1FilterApply(PT1_Filter_Struct *Filter, float Input, float CutOffFrequency, float DeltaTime)
{
    //CALCULA O VALOR RC DO FILTRO
    //ESTAMOS CALCULANDO UM RESISTOR E UM CAPACITOR DIGITALMENTE
    if (!Filter->RC)
    {
        Filter->RC = 1.0f / (2.0f * 3.14159265358979323846f * CutOffFrequency);
    }
    //GUARDA O ÚLTIMO VALOR DO DELTA TIME
    Filter->DeltaTime = DeltaTime;
    //CALCULA O VALOR DO FILTRO
    Filter->State = Filter->State + DeltaTime / (Filter->RC + DeltaTime) * (Input - Filter->State);
    //RETORNA O VALOR FILTRADO
    return Filter->State;
}

float PT1FilterApply2(PT1_Filter_Struct *Filter, float Input)
{
    Filter->State = Filter->State + Filter->DeltaTime / (Filter->RC + Filter->DeltaTime) * (Input - Filter->State);
    return Filter->State;
}