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
#include "FlightModes/IOCMODE.h"
#include "TPA.h"
#include "Math/AVRMATH.h"
#include "RadioControl/RCSMOOTH.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "FastSerial/PRINTF.h"

//lrint RETORNA UM VALOR ARREDONDADO DE UM NÚMERO
#define LRintFloat lrint

//DEBUG
//#define PRINTLN_TPA

uint8_t DynamicProportionalVector[2];
uint8_t DynamicDerivativeVector[2];

int16_t CalcedAttitudeRC(int16_t Data, int16_t RCExpo)
{
  int16_t RCValueDeflection;
  RCValueDeflection = Constrain_16Bits(RadioControllOutput[Data] - 1500, -500, 500);
  float ConvertValueToFloat = RCValueDeflection / 100.0f;
  return LRintFloat((2500.0f + (float)RCExpo * (ConvertValueToFloat * ConvertValueToFloat - 25.0f)) * ConvertValueToFloat / 25.0f);
}

uint16_t CalcedLookupThrottle(uint16_t CalcedDeflection)
{
  if (CalcedDeflection > 999)
    return 1900;

  const uint8_t CalcedLookUpStep = CalcedDeflection / 100;
  return CalculeLookUpThrottle[CalcedLookUpStep] +
         (CalcedDeflection - CalcedLookUpStep * 100) *
             (CalculeLookUpThrottle[CalcedLookUpStep + 1] -
              CalculeLookUpThrottle[CalcedLookUpStep]) /
             100;
}

void DynamicPID()
{
  uint8_t DynamicProportional;
  uint8_t DynamicProportionalTwo;
  //THROTTLE PID ATTENUATION
  //A ATENUAÇÃO OCORRE APENAS NO PROPORCIONAL E NO DERIVATIVO
  //AJUSTE DINAMICO DE ACORDO COM O VALOR DO THROTTLE
  if (GetFrameStateOfMultirotor()) //CONFIG PARA DRONES
  {
    DynamicProportionalTwo = CalculateMultirotorTPAFactor(RCController[THROTTLE]);
#if defined(PRINTLN_TPA)
    FastSerialPrintln(PSTR("TPACopter:%d\n"), DynamicProportionalTwo);
#endif
  }
  else //CONFIG PARA AEROS E ASA-FIXA
  {
    DynamicProportionalTwo = CalculateFixedWingTPAFactor(RCController[THROTTLE]);
#if defined(PRINTLN_TPA)
    FastSerialPrintln(PSTR("TPAPlane:%d\n"), DynamicProportionalTwo);
#endif
  }

  for (uint8_t RCIndexCount = 0; RCIndexCount < 2; RCIndexCount++)
  {
    uint16_t DynamicStored = MIN_U16BITS(ABS_16BITS(RadioControllOutput[RCIndexCount] - 1500), 500);
    DynamicProportional = 100 - (uint16_t)RollAndPitchRate * DynamicStored / 500;
    DynamicProportional = (uint16_t)DynamicProportional * DynamicProportionalTwo / 100;
    DynamicProportionalVector[RCIndexCount] = (uint16_t)PID[RCIndexCount].ProportionalVector * DynamicProportional / 100;
    DynamicDerivativeVector[RCIndexCount] = (uint16_t)PID[RCIndexCount].DerivativeVector * DynamicProportional / 100;
  }

  int32_t CalcedThrottle;
  CalcedThrottle = Constrain_16Bits(RadioControllOutput[THROTTLE], 1000, 2000);
  CalcedThrottle = (uint32_t)(CalcedThrottle - 1000) * 1000 / 900;
  RCController[THROTTLE] = CalcedLookupThrottle(CalcedThrottle);
  RCController[YAW] = CalcedAttitudeRC(YAW, RCRate);
  RCController[PITCH] = CalcedAttitudeRC(PITCH, RCRate);
  RCController[ROLL] = CalcedAttitudeRC(ROLL, RCRate);
  RCInterpolationApply();
  //REMOVE O -1 CAUSADO PELO FILTRO
  if (ABS_16BITS(RCController[YAW]) < 5)
    RCController[YAW] = 0;
  if (ABS_16BITS(RCController[PITCH]) < 5)
    RCController[PITCH] = 0;
  if (ABS_16BITS(RCController[ROLL]) < 5)
    RCController[ROLL] = 0;
  //REMOVE OS VALORES MAIORES QUE -500 E 500 CAUSADOS PELO FILTRO
  if (RCController[YAW] > 500)
    RCController[YAW] = 500;
  else if (RCController[YAW] < -500)
    RCController[YAW] = -500;
  if (RCController[ROLL] > 500)
    RCController[ROLL] = 500;
  else if (RCController[ROLL] < -500)
    RCController[ROLL] = -500;
  if (RCController[PITCH] > 500)
    RCController[PITCH] = 500;
  else if (RCController[PITCH] < -500)
    RCController[PITCH] = -500;
  IOC_Mode_Update();
}