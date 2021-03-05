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
#define MIN(a, b) \
  __extension__({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a, b) \
  __extension__({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
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
float ConvertDegreesToDecidegrees(float Inputvalue);
float ConvertCoordinateToFloatingPoint(int32_t CoordinateInput);
float ConvertAccelerationEarthFrameToCMSS(float InputAccEF);
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
float Fast_Pow(float ValueA, float ValueB);
#endif