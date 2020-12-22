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


#ifndef BIQUADFILTER_h
#define BIQUADFILTER_h
#include "Arduino.h"
#include "Common/ENUM.h"
class BiQuadFilter
{
public:
  void Settings(int16_t CutOffFreq, int16_t SampleFreq, uint8_t FilterType);
  int16_t FilterOutput(int16_t DeviveToFilter);

private:
  //COEFICIENTES PARA O FILTRO
  int16_t Coeff1;
  int16_t Coeff2;
  int16_t Coeff3;
  int16_t Coeff4;
  int16_t Coeff5;

  //VARIAVEIS PARA GUARDAR VALORES LIDOS DO DISPOSITIVO A CADA CICLO DE MAQUINA
  int16_t GuardInput1 = 0;
  int16_t GuardInput2 = 0;
  int16_t GuardOutput1 = 0;
  int16_t GuardOutput2 = 0;

  //CONVERSÃO DE VALORES
  union TypeConverter
  {
    int32_t LongValue;
    int16_t ShortValue[2];
  } Result;
};
#endif
