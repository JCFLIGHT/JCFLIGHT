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

#ifndef COMPASSREAD_H_
#define COMPASSREAD_H_
#include "Arduino.h"
class CompassReadClass
{
public:
  bool Calibrating;
  uint8_t Type;
  uint8_t Address;
  uint8_t Register;
  uint8_t FakeHMC5883Address = 0;
  float MagnetometerRead[3];
  float MagnetometerGain[3] = {1.0f, 1.0f, 1.0f};
  int16_t MagCalibrationMinVector[3];
  int16_t MagCalibrationMaxVector[3];
  int16_t CalibrationCount = 0;
  int16_t IOC_Initial;
  void Initialization();
  void Constant_Read();
  void UpdateCompassCalibration();

private:
  bool PushBias(uint8_t InputBias);
  void InitialReadBufferData();
  void ReadBufferData();
};
extern CompassReadClass COMPASS;
#endif
