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
	int32_t Get_PI_Calced(int32_t Error, float DeltaTime);

	void Reset_Integrator();

	void SetkP(const float Value)
	{
		kP = Value;
	}

	void SetkI(const float Value)
	{
		kI = Value;
	}

	void SetIntegratorMax(const int16_t Value)
	{
		Integrator_Max = Value;
	}

	void SetOutputMin(const int16_t Value)
	{
		OutputMin = Value;
	}

	void SetOutputMax(const int16_t Value)
	{
		OutputMax = Value;
	}

private:
	float kP;
	float kI;
	float Integrator_Sum;
	int16_t Integrator_Max;
	int32_t OutputMin;
	int32_t OutputMax;
};

#endif
