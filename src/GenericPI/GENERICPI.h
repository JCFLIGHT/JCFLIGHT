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

#ifndef GENERICPI_H_
#define GENERICPI_H_

#include "Build/LIBDEPENDENCIES.h"

class GenericPIClass
{
public:
	float Get_PI_Calced(int16_t Error, float DeltaTime);
	int16_t GetPICalcedWithDataConstrained(float Input_PI_Calced, float Input_To_Sum);
	void Reset_Integral();

	void Set_kP(const float Value)
	{
		kP = Value;
	}

	void Set_kI(const float Value)
	{
		kI = Value;
	}

	void Set_kP_Scale(const float Value)
	{
		kP_Scale = Value;
	}

	void Set_kI_Scale(const float Value)
	{
		kI_Scale = Value;
	}

	void Set_Integral_Max(const int16_t Value)
	{
		Integral_Max = Value;
	}

	void Set_Integral_Scale(const int16_t Value = 1)
	{
		Integral_Scale = Value;
	}

	void Set_Output_Min(const int16_t Value)
	{
		OutputMin = Value;
	}

	void Set_Output_Max(const int16_t Value)
	{
		OutputMax = Value;
	}

private:
	float kP;
	float kI;
	float kP_Scale;
	float kI_Scale;
	float Integral_Sum;
	float Integral_Scale;
	int16_t Integral_Max;
	int32_t OutputMin;
	int32_t OutputMax;
};

#endif
