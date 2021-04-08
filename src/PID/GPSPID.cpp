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

#include "GPSPID.h"
#include "Math/MATHSUPPORT.h"

PID_PARAM PositionHoldPID;
PID_PARAM PositionHoldRatePID;
PID_PARAM NavigationPID;

GPS_PID PositionHoldPIDArray[2];
GPS_PID PositionHoldRatePIDArray[2];
GPS_PID NavigationPIDArray[2];

#define GPS_DERIVATIVE_CUTOFF 20 //HZ

int32_t GPSGetProportional(int32_t Error, struct _PID_PARAM *PID)
{
    return (float)Error * PID->kP;
}

int32_t GPSGetIntegral(int32_t Error, float DeltaTime, struct _GPS_PID *PID, struct _PID_PARAM *GPS_PID_Param)
{
    PID->Integral += ((float)Error * GPS_PID_Param->kI) * DeltaTime;
    PID->Integral = Constrain_Float(PID->Integral, -GPS_PID_Param->IntegralMax, GPS_PID_Param->IntegralMax);
    return PID->Integral;
}

float Get_Derivative_LPF_Coefficient(float CutOff)
{
    return (1.0f / (6.283185482f * (float)CutOff));
}

int32_t GPSGetDerivative(int32_t Input, float DeltaTime, struct _GPS_PID *PID, struct _PID_PARAM *GPS_PID_Param)
{
    PID->Derivative = (Input - PID->Last_Input) / DeltaTime;
    PID->Derivative = PID->Last_Derivative + (DeltaTime / (Get_Derivative_LPF_Coefficient(GPS_DERIVATIVE_CUTOFF) + DeltaTime)) * (PID->Derivative - PID->Last_Derivative);
    PID->Last_Input = Input;
    PID->Last_Derivative = PID->Derivative;
    return GPS_PID_Param->kD * PID->Derivative;
}

void GPSResetPID(struct _GPS_PID *PID)
{
    PID->Integral = 0;
    PID->Last_Input = 0;
    PID->Last_Derivative = 0;
}

void ResetAllGPSPID(void)
{
    GPSResetPID(&PositionHoldPIDArray[COORD_LATITUDE]);
    GPSResetPID(&PositionHoldPIDArray[COORD_LONGITUDE]);
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LATITUDE]);
    GPSResetPID(&PositionHoldRatePIDArray[COORD_LONGITUDE]);
    GPSResetPID(&NavigationPIDArray[COORD_LATITUDE]);
    GPSResetPID(&NavigationPIDArray[COORD_LONGITUDE]);
}