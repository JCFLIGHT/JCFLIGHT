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

#ifndef LEDRGB_H_
#define LEDRGB_H_
#include "Build/LIBDEPENDENCIES.h"
class LEDRGB
{
public:
  void Initialization(void);
  void Update(void);
  void Function(uint8_t Mode);
  void Off_All_Leds(void);

private:
  uint16_t SetColorValue[3];
  void CalibAccLed(void);
  void CalibMagLed(void);
  void ConfigFlight_Led(void);
  void CalibEsc_Led(void);
  void GPS_Led(void);
  void Pre_Arm_Initializing(void);
  void Pre_Arm_Sucess(void);
  void Pre_Arm_Fail(void);
  void CalibGyroLed(void);
};
extern LEDRGB RGB;
#endif
