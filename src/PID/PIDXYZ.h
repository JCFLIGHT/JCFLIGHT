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
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern PID_Resources_Struct PIDResources;
class PIDXYZClass
{
public:
  void Initialization(void);
  void Update(float DeltaTime);
  void Reset_Integral_Accumulators(void);

private:
  float ProportionalTermProcess(uint8_t kP, float RateError);
  float DerivativeTermProcessRoll(int16_t ActualGyro, float PrevGyro, float PrevRateTarget, float DeltaTime);
  float DerivativeTermProcessPitch(int16_t ActualGyro, float PrevGyro, float PrevRateTarget, float DeltaTime);
  float DerivativeTermProcessYaw(int16_t ActualGyro, float PreviousRateGyro, float PreviousRateTarget, float DeltaTime);
  float ApplyIntegralTermRelaxRoll(float CurrentPIDSetpoint, float IntegralTermErrorRate);
  float ApplyIntegralTermRelaxPitch(float CurrentPIDSetpoint, float IntegralTermErrorRate);
  float ApplyIntegralTermLimiting(uint8_t Axis, float ErrorGyroIntegral);
  float ApplyDerivativeBoostRoll(int16_t ActualGyro, int16_t PrevGyro, int16_t ActualRateTagert, int16_t PrevRateTagert, float DeltaTime);
  float ApplyDerivativeBoostPitch(int16_t ActualGyro, int16_t PrevGyro, int16_t ActualRateTagert, int16_t PrevRateTagert, float DeltaTime);
  float LevelRoll(float DeltaTime);
  float LevelPitch(float DeltaTime);
  void ApplyMulticopterRateControllerRoll(float DeltaTime);
  void ApplyMulticopterRateControllerPitch(float DeltaTime);
  void ApplyMulticopterRateControllerYaw(float DeltaTime);
  void ApplyFixedWingRateControllerRoll(float DeltaTime);
  void ApplyFixedWingRateControllerPitch(float DeltaTime);
  void ApplyFixedWingRateControllerYaw(float DeltaTime);
  bool FixedWingIntegralTermLimitActive(uint8_t Axis);
  void GetNewControllerForPlaneWithTurn(void);
};
extern PIDXYZClass PIDXYZ;
#endif
