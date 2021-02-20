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

#include "DYNAMICPID.h"
#include "Common/VARIABLES.h"
#include "IntelligentOrientationControl/IOCMODE.h"
#include "TPA.h"
#include "Math/MATHSUPPORT.h"
#include "RadioControl/RCSMOOTH.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "RadioControl/CURVESRC.h"
#include "MotorsControl/THRBOOST.h"
#include "FastSerial/PRINTF.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//DEBUG
//#define PRINTLN_TPA

uint8_t DynamicPIDCalced;
uint8_t DynamicProportionalVector[2];
uint8_t DynamicDerivativeVector[2];

void GetRCDataConvertedAndApplyFilter()
{
  int32_t CalcedThrottle;
  CalcedThrottle = Constrain_16Bits(RadioControllOutput[THROTTLE], AttitudeThrottleMin, MAX_STICKS_PULSE);
  CalcedThrottle = (uint32_t)(CalcedThrottle - AttitudeThrottleMin) * MIN_STICKS_PULSE / (MAX_STICKS_PULSE - AttitudeThrottleMin);
  RCController[THROTTLE] = CalcedLookupThrottle(CalcedThrottle);
  RCController[YAW] = -CalcedAttitudeRC(YAW, RCExpo);
  RCController[PITCH] = CalcedAttitudeRC(PITCH, RCExpo);
  RCController[ROLL] = -CalcedAttitudeRC(ROLL, RCExpo);

  //APLICA O FILTRO LPF NO RC DA ATTITUDE
  RCInterpolationApply();

  //FAZ UMA PEQUENA ZONA MORTA NOS CANAIS DA ATTITUDE
  if (ABS(RCController[YAW]) < 5)
  {
    RCController[YAW] = 0;
  }
  if (ABS(RCController[PITCH]) < 5)
  {
    RCController[PITCH] = 0;
  }
  if (ABS(RCController[ROLL]) < 5)
  {
    RCController[ROLL] = 0;
  }

  //REMOVE OS VALORES MAIORES QUE -500 E 500 CAUSADOS PELO FILTRO
  if (RCController[YAW] > 500)
  {
    RCController[YAW] = 500;
  }
  else if (RCController[YAW] < -500)
  {
    RCController[YAW] = -500;
  }
  if (RCController[ROLL] > 500)
  {
    RCController[ROLL] = 500;
  }
  else if (RCController[ROLL] < -500)
  {
    RCController[ROLL] = -500;
  }
  if (RCController[PITCH] > 500)
  {
    RCController[PITCH] = 500;
  }
  else if (RCController[PITCH] < -500)
  {
    RCController[PITCH] = -500;
  }
}

void PID_Dynamic()
{
  //CONVERTE AS DATAS DOS RADIO E APLICA O FILTRO LPF
  GetRCDataConvertedAndApplyFilter();

  //APLICA O BOOST NO THROTTLE PARA O MODO STABILIZE
  ApplyThrottleBoost();

  //THROTTLE PID ATTENUATION
  //A ATENUAÇÃO OCORRE APENAS NO PROPORCIONAL E NO DERIVATIVO
  //AJUSTE DINAMICO DE ACORDO COM O VALOR DO THROTTLE
  if (GetFrameStateOfMultirotor()) //CONFIG PARA DRONES
  {
    TPA_Parameters.CalcedValue = CalculateMultirotorTPAFactor(RCController[THROTTLE]);
#if defined(PRINTLN_TPA)
    PRINTF.SendToConsole(PSTR("TPACopter:%d\n"), TPA_Parameters.CalcedValue);
#endif
  }
  else //CONFIG PARA AEROS E ASA-FIXA
  {
    TPA_Parameters.CalcedValue = CalculateFixedWingTPAFactor(RCController[THROTTLE]);
#if defined(PRINTLN_TPA)
    PRINTF.SendToConsole(PSTR("TPAPlane:%d\n"), TPA_Parameters.CalcedValue);
#endif
  }
  for (uint8_t RCIndexCount = 0; RCIndexCount < 2; RCIndexCount++)
  {
    uint16_t DynamicStored = MIN(ABS(RadioControllOutput[RCIndexCount] - MIDDLE_STICKS_PULSE), 500);
    DynamicPIDCalced = 100 - (uint16_t)DynamicRollAndPitchRate[RCIndexCount] * DynamicStored / 500;
    DynamicPIDCalced = (uint16_t)DynamicPIDCalced * TPA_Parameters.CalcedValue / 100;
    DynamicProportionalVector[RCIndexCount] = (uint16_t)PID[RCIndexCount].ProportionalVector * DynamicPIDCalced / 100;
    DynamicDerivativeVector[RCIndexCount] = (uint16_t)PID[RCIndexCount].DerivativeVector * DynamicPIDCalced / 100;
  }
  IOC_Mode_Update();
}