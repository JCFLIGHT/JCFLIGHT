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

#ifndef MATHSUPPORT_H_
#define MATHSUPPORT_H_
#include "Build/LIBDEPENDENCIES.h"
#define _CHOOSE2(binoper, lexpr, lvar, rexpr, rvar) \
  (__extension__({                                  \
    __typeof__(lexpr) lvar = (lexpr);               \
    __typeof__(rexpr) rvar = (rexpr);               \
    lvar binoper rvar ? lvar : rvar;                \
  }))
#define _CHOOSE_VAR2(prefix, unique) prefix##unique
#define _CHOOSE_VAR(prefix, unique) _CHOOSE_VAR2(prefix, unique)
#define _CHOOSE(binoper, lexpr, rexpr)        \
  _CHOOSE2(                                   \
      binoper,                                \
      lexpr, _CHOOSE_VAR(_left, __COUNTER__), \
      rexpr, _CHOOSE_VAR(_right, __COUNTER__))
#define MIN(a, b) _CHOOSE(<, a, b)
#define MAX(a, b) _CHOOSE(>, a, b)
#define _ABS_II(x, var)      \
  (__extension__({           \
    __typeof__(x) var = (x); \
    var < 0 ? -var : var;    \
  }))
#define _ABS_I(x, var) _ABS_II(x, var)
#define ABS(x) _ABS_I(x, _CHOOSE_VAR(_abs, __COUNTER__))
#define GRAVITY_MSS 9.80665f  //VALOR DA GRAVIDADE EM M/S^2
#define GRAVITY_CMSS 980.665f //VALOR DA GRAVIDADE EM CM/S^2
float Constrain_Float(float ValueInput, float ValueInputMin, float ValueInputMax);
int8_t Constrain_8Bits(int8_t ValueInput, int8_t ValueInputMin, int8_t ValueInputMax);
uint8_t Constrain_U8Bits(uint8_t ValueInput, uint8_t ValueInputMin, uint8_t ValueInputMax);
int16_t Constrain_16Bits(int16_t ValueInput, int16_t ValueInputMin, int16_t ValueInputMax);
uint16_t Constrain_U16Bits(uint16_t ValueInput, uint16_t ValueInputMin, uint16_t ValueInputMax);
int32_t Constrain_32Bits(int32_t ValueInput, int32_t ValueInputMin, int32_t ValueInputMax);
float ScaleRangeFloat(float Value, float MinInputValue, float MaxInputValue, float MinOutputValue, float MaxOutputValue);
int16_t ScaleRange16Bits(int16_t Value, int16_t MinInputValue, int16_t MaxInputValue, int16_t MinOutputValue, int16_t MaxOutputValue);
int32_t ScaleRange32Bits(int32_t Value, int32_t MinInputValue, int32_t MaxInputValue, int32_t MinOutputValue, int32_t MaxOutputValue);
float SquareFloat(float InputValue);
int32_t Square32Bits(int32_t InputValue);
float ConvertToDegrees(float InputValue);
float ConvertToRadians(float InputValue);
float ConvertRadiansToDeciDegrees(float Inputvalue);
float ConvertDeciDegreesToRadians(float Inputvalue);
float ConvertDeciDegreesToDegrees(float Inputvalue);
float ConvertCentiDegreesToDeciDegrees(float Inputvalue);
float ConvertDegreesToDecidegrees(float Inputvalue);
float ConvertDecidegreesToCentiDegrees(float Inputvalue);
float ConvertDegreesToCentiDegrees(float Inputvalue);
float ConvertRadiansToCentiDegrees(float Inputvalue);
float ConvertCentiDegreesToRadians(float Inputvalue);
float ConvertCentiDegreesToDegrees(float Inputvalue);
float ConvertCoordinateToFloatingPoint(int32_t CoordinateInput);
float ConvertCMToMeters(int32_t CM_Input);
int32_t ConverMetersToCM(float Meters_Input);
float Fast_SquareRoot(float ValueInput);
uint16_t SquareRootU16Bits(uint16_t ValueInput);
uint32_t SquareRootU32Bits(uint32_t ValueInput);
float Fast_Sine(float X);
float Fast_Cosine(float X);
float Fast_Atan2(float Y, float X);
float Fast_AtanCosine(float X);
float Sine_Curve(const float InputValue, const float CurveWidth);
float Fast_Tangent(float InputValue);
int32_t WRap_18000(int32_t AngleInput);
int32_t WRap_36000(int32_t AngleInput);
float Fast_Pow(float ValueA, float ValueB);
float Power_Float(float Base, int16_t Exponential);
float ConvertCentimeterPerSecondsToKmPerHour(float Value);
float Power3_Float(float InputValue);
#endif