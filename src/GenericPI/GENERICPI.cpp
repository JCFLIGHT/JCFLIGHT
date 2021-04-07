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

#include "GENERICPI.h"
#include "Math/MATHSUPPORT.h"

float GenericPIClass::Get_PI_Calced(int16_t Error, float DeltaTime)
{
  Integral_Sum += ((float)Error * kI / kI_Scale) * DeltaTime;
  Integral_Sum = Constrain_Float(Integral_Sum, -Integral_Max, Integral_Max);
  return (float)(Error * kP / kP_Scale) + (Integral_Sum / Integral_Scale);
}

int16_t GenericPIClass::GetPICalcedWithDataConstrained(float Input_PI_Calced, float Input_To_Sum)
{
  return Constrain_16Bits(Input_PI_Calced + Input_To_Sum, OutputMin, OutputMax);
}

void GenericPIClass::Reset_Integral()
{
  Integral_Sum = 0;
}