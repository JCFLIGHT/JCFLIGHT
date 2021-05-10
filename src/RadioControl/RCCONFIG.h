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

#ifndef RCCONFIG_H_
#define RCCONFIG_H_
#include "Build/LIBDEPENDENCIES.h"
class RC_Config
{
public:
  int16_t Input;
  int16_t Output;
  int16_t Min_Pulse;
  int16_t Max_Pulse;
  int16_t Get_Channel_Range(void);
  void Set_Range(int16_t Min, int16_t Max);
  void Set_Pulse(int16_t ChannelInputValue);
  void Set_Reverse(bool Reverse);
  void Set_Dead_Zone(uint8_t DeadZone);
  void Set_Fail_Safe(bool FailSafe);

private:
  bool _FailSafe;
  bool _Fail_Safe;
  int8_t _Reverse;
  uint8_t _DeadZone;
  int16_t _Max_Pulse;
  int16_t _Min_Pulse;
  int16_t RcConstrain;
};

class RCConfigClass
{
public:
  void Initialization(void);
  void Set_Pulse(void);
  void Update_Channels(void);
};
extern RCConfigClass RCCONFIG;
extern RC_Config Throttle;
extern RC_Config Yaw;
extern RC_Config Pitch;
extern RC_Config Roll;
extern RC_Config AuxiliarOne;
extern RC_Config AuxiliarTwo;
extern RC_Config AuxiliarThree;
extern RC_Config AuxiliarFour;
extern RC_Config AuxiliarFive;
extern RC_Config AuxiliarSix;
extern RC_Config AuxiliarSeven;
extern RC_Config AuxiliarEight;
#endif
