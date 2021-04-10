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

PID_Terms_Float_Struct PositionHoldPID;
PID_Terms_Float_Struct PositionHoldRatePID;
PID_Terms_Float_Struct NavigationPID;

PID_Terms_Float_Struct PositionHoldPIDArray[2];
PID_Terms_Float_Struct PositionHoldRatePIDArray[2];
PID_Terms_Float_Struct NavigationPIDArray[2];

#define GPS_DERIVATIVE_CUTOFF 20 //HZ

int32_t GPSGetProportional(int32_t Error, PID_Terms_Float_Struct *PID)
{
    return (float)Error * PID->kP;
}

int32_t GPSGetIntegral(int32_t Error, float DeltaTime, PID_Terms_Float_Struct *PID, PID_Terms_Float_Struct *GPS_PID_Param)
{
    PID->GPS.IntegralSum += ((float)Error * GPS_PID_Param->kI) * DeltaTime;
    PID->GPS.IntegralSum = Constrain_Float(PID->GPS.IntegralSum, -GPS_PID_Param->GPS.IntegralMax, GPS_PID_Param->GPS.IntegralMax);
    return PID->GPS.IntegralSum;
}

float Get_Derivative_LPF_Coefficient(float CutOff)
{
    return (1.0f / (6.283185482f * (float)CutOff));
}

int32_t GPSGetDerivative(int32_t Input, float DeltaTime, PID_Terms_Float_Struct *PID, PID_Terms_Float_Struct *GPS_PID_Param)
{
    PID->GPS.DerivativeCalced = (Input - PID->GPS.LastInput) / DeltaTime;
    PID->GPS.DerivativeCalced = PID->GPS.LastDerivative + (DeltaTime / (Get_Derivative_LPF_Coefficient(GPS_DERIVATIVE_CUTOFF) + DeltaTime)) * (PID->GPS.DerivativeCalced - PID->GPS.LastDerivative);
    PID->GPS.LastInput = Input;
    PID->GPS.LastDerivative = PID->GPS.DerivativeCalced;
    return GPS_PID_Param->kD * PID->GPS.DerivativeCalced;
}

void GPSResetPID(PID_Terms_Float_Struct *PID)
{
    PID->GPS.IntegralSum = 0;
    PID->GPS.LastInput = 0;
    PID->GPS.LastDerivative = 0;
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