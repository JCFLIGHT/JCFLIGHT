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

#ifndef PIDXYZ_H_
#define PIDXYZ_H_
#include "Arduino.h"
class PIDXYZClass
{
public:
  int16_t IntegralAccError[2];
  int16_t IntegralGyroError[2];
  int32_t IntegralGyroError_Yaw;
  int16_t CalcedRateTargetRoll;
  int16_t CalcedRateTargetPitch;
  int16_t CalcedRateTargetYaw;
  void DerivativeLPF_Update();
  void Update(int32_t DeltaTimeUs);
  int16_t AngleTarget(int16_t RcControllerInput, uint8_t AttitudeAngle, int16_t MaxInclination);

private:
  int16_t Get_LPF_Derivative_Value = 0;
  void Controll_Pitch(int16_t RateTargetInput, int32_t DeltaTimeUs);
  void Controll_Roll(int16_t RateTargetInput, int32_t DeltaTimeUs);
  void Controll_Yaw(int16_t RateTargetInput, int32_t DeltaTimeUs);
  int32_t ProportionalTermProcess(int16_t RcRateError, uint8_t kP);
  int32_t DerivativeTermProcessRoll(int16_t ActualGyro, int16_t LastGyro, int16_t DynamicDerivative, int32_t DeltaTimeUs);
  int32_t DerivativeTermProcessPitch(int16_t ActualGyro, int16_t LastGyro, int16_t DynamicDerivative, int32_t DeltaTimeUs);
  int16_t ApplyIntegralTermLimiting(int16_t IntegratorTerminate);
  bool GetGyroSaturation(uint8_t GyroAxis, int16_t SaturationValue);
  int16_t TurnControllerForAirPlane(int16_t RadioControlToTurn);
  void Reset_Integral_Accumulators();
};
extern PIDXYZClass PIDXYZ;
#endif
