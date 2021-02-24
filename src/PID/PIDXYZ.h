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
  int16_t CalcedRateTargetRollToGCS;
  int16_t CalcedRateTargetPitchToGCS;
  int16_t CalcedRateTargetYawToGCS;
  void DerivativeLPF_Update();
  void Update(float DeltaTime);
  void Reset_Integral_Accumulators();

private:
  int16_t Get_LPF_Derivative_Value = 0;
  int16_t CalcedRateTargetRoll;
  int16_t CalcedRateTargetPitch;
  int16_t CalcedRateTargetYaw;
  float ProportionalTermProcess(uint8_t kP, float RateError);
  float DerivativeTermProcessRoll(float GyroDiffInpu);
  float DerivativeTermProcessPitch(float GyroDiffInput);
  float ApplyIntegralTermRelax(float CurrentPIDSetpoint, float IntegralTermErrorRate);
  float ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral);
  float PIDLevelRoll(float DeltaTime);
  float PIDLevelPitch(float DeltaTime);
  void PIDApplyMulticopterRateControllerRoll(float DeltaTime);
  void PIDApplyMulticopterRateControllerPitch(float DeltaTime);
  void PIDApplyMulticopterRateControllerYaw(float DeltaTime);
  void PIDApplyFixedWingRateControllerRoll(float DeltaTime);
  void PIDApplyFixedWingRateControllerPitch(float DeltaTime);
  void PIDApplyFixedWingRateControllerYaw(float DeltaTime);
  bool FixedWingIntegralTermLimitActive(uint8_t Axis);
  int16_t GetNewYawControllerForPlane(int16_t RateTargetInput);
  int16_t TurnControllerForAirPlane(int16_t RadioControlToTurn);
};
extern PIDXYZClass PIDXYZ;
#endif
