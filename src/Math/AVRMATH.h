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

#ifndef AVRMATH_H_
#define AVRMATH_H_
#include "Arduino.h"
float ABS_FLOAT(float X);
int16_t ABS_16BITS(int16_t X);
int32_t ABS_32BITS(int32_t X);
float MIN_FLOAT(float X, float Y);
float MAX_FLOAT(float X, float Y);
int8_t MIN_8BITS(int8_t X, int8_t Y);
uint16_t MIN_U16BITS(uint16_t X, uint16_t Y);
uint16_t MAX_U16BITS(uint16_t X, uint16_t Y);
float Constrain_Float(float ValueInput, float ValueInputMin, float ValueInputMax);
int8_t Constrain_8Bits(int8_t ValueInput, int8_t ValueInputMin, int8_t ValueInputMax);
uint8_t Constrain_U8Bits(uint8_t ValueInput, uint8_t ValueInputMin, uint8_t ValueInputMax);
int16_t Constrain_16Bits(int16_t ValueInput, int16_t ValueInputMin, int16_t ValueInputMax);
uint16_t Constrain_U16Bits(uint16_t ValueInput, uint16_t ValueInputMin, uint16_t ValueInputMax);
int32_t Constrain_32Bits(int32_t ValueInput, int32_t ValueInputMin, int32_t ValueInputMax);
float Map_Float(float Value, float MinInputValue, float MaxInputValue, float MinOutputValue, float MaxOutputValue);
int16_t Map_16Bits(int16_t Value, int16_t MinInputValue, int16_t MaxInputValue, int16_t MinOutputValue, int16_t MaxOutputValue);
int32_t Map_32Bits(int32_t Value, int32_t MinInputValue, int32_t MaxInputValue, int32_t MinOutputValue, int32_t MaxOutputValue);
float SquareFloat(float InputValue);
int32_t Square32Bits(int32_t InputValue);
float ConvetToDegrees(float InputValue);
float ConvertToRadians(float InputValue);
float ConvertRadiansToDeciDegrees(float Inputvalue);
float ConvertDeciDegreesToRadians(float Inputvalue);
float ConvertDeciDegreesToDegrees(float Inputvalue);
int16_t ApproximationOfAtan2(int16_t AccRoll, int16_t AccYaw);
float Calculate_Sine_Approx(int16_t InputAngle);
float Calculate_Cosine_Approx(int16_t InputAngle);
uint16_t SquareRootU16Bits(uint16_t ValueInput);
uint32_t SquareRootU32Bits(uint32_t ValueInput);
float InvertSquareRootFloat(float InputValue);
float Fast_Sine(float X);
float Fast_Cosine(float X);
float Fast_Atan2(float Y, float X);
float Fast_AtanCosine(float X);
#endif