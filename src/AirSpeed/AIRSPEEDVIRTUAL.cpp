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

#include "AIRSPEEDVIRTUAL.h"
#include "WindEstimator/WINDESTIMATOR.h"
#include "Math/MATHSUPPORT.h"
#include "InertialNavigation/INS.h"
#include "AHRS/AHRS.h"
#include "GPSNavigation/NAVIGATION.h"
#include "Build/BOARDDEFS.h"

#define AIR_DENSITY_SEA_LEVEL_15C 1.225f //DENSIDADE DO AR ACIMA DO MAR COM A TEMPERATURA EM 15 GRAUS
#define P0 101325.0f                     //1 ATMOSFERAS EM PASCALS
#define USE_INS_VEL_XY                   //USE A VELOCIDADE XY DO INS AO INVÉS DO GROUND-SPEED DADO PELO GPS

float AirSpeed_Virtual_Get_Actual_Value(void)
{

#ifndef USE_WIND_ESTIMATOR

  return 0;

#else

  float AirSpeed = 0.0f;
  float PressureRet = 0.0f;

  if (WINDESTIMATOR.EstimatedValid())
  {
    uint16_t WindHeading;
    float WindSpeed = WINDESTIMATOR.GetEstimatedValueHorizontal(&WindHeading);
    float HorizontalWindSpeed = WindSpeed * Fast_Cosine(ConvertCentiDegreesToRadians(WindHeading - ConvertDecidegreesToCentiDegrees(Attitude.EulerAngles.YawDecidegrees)));

#ifdef USE_INS_VEL_XY

    float VelocityXY = sqrtf(SquareFloat(INS.AccelerationEarthFrame_LPF[ROLL]) + SquareFloat(INS.AccelerationEarthFrame_LPF[PITCH]));

#else

    float VelocityXY = GPSParameters.Navigation.Misc.Get.GroundSpeed

#endif

    AirSpeed = VelocityXY - HorizontalWindSpeed;
  }
  else
  {
    //AirSpeed = ReferenceAirSpeed; //ISSO DEVE SER ATUALIZADO QUANDO O "ReferenceAirSpeed" FOR PASSADO PARA A LISTA DE PARAMETROS
  }

  PressureRet = SquareFloat(AirSpeed) * AIR_DENSITY_SEA_LEVEL_15C / 20000.0f + P0;

  return PressureRet;

#endif
}