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

#include "RCCONFIG.h"
#include "Common/VARIABLES.h"
#include "RadioControl/DECODE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "SBUS/SBUSREAD.h"
#include "IBUS/IBUSREAD.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "Common/RCDEFINES.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//INSTANCIAS
RC_Config Throttle;
RC_Config Yaw;
RC_Config Pitch;
RC_Config Roll;
RC_Config AuxiliarOne;
RC_Config AuxiliarTwo;
RC_Config AuxiliarThree;
RC_Config AuxiliarFour;
RC_Config AuxiliarFive;
RC_Config AuxiliarSix;
RC_Config AuxiliarSeven;
RC_Config AuxiliarEight;

RCConfigClass RCCONFIG;

void RC_Config::Set_Range(int16_t Min, int16_t Max)
{
  _Max_Pulse = Max;
  _Min_Pulse = Min;
}

void RC_Config::Set_Filter(bool Filter)
{
  _Filter = Filter;
}

void RC_Config::Set_Pulse(int16_t ChannelInputValue)
{
  if (_Filter)
  {
    //SMALL FILTERING NOS CANAIS (APENAS UMA MÉDIA ENTRE O VALOR ATUAL E O ANTERIOR)
    Input = (ChannelInputValue + Input) >> 1;
  }
  else //SEM FILTRO
  {
    Input = ChannelInputValue;
  }
  if (!_FailSafe)
  {
    _Fail_Safe = false;
  }
  else
  {
    if (Input > (int16_t)CHECKSUM.GetFailSafeValue)
    {
      _Fail_Safe = false;
    }
    else if (Input < (int16_t)CHECKSUM.GetFailSafeValue)
    {
      _Fail_Safe = true;
    }
  }
  Output = Get_Channel_Range();
}

void RC_Config::Set_Reverse(bool Reverse)
{
  _Reverse = Reverse;
}

void RC_Config::Set_Dead_Zone(uint8_t DeadZone)
{
  //NÃO APLICA A ZONA MORTA NO SBUS E IBUS
  if ((STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == SBUS_RECEIVER) ||
      (STORAGEMANAGER.Read_8Bits(UART_NUMB_2_ADDR) == IBUS_RECEIVER))
  {
    _DeadZone = 0;
  }
  else
  {
    _DeadZone = Constrain_8Bits(DeadZone, 0, 50);
  }
}

void RC_Config::Set_Fail_Safe(bool FailSafe)
{
  _FailSafe = FailSafe;
}

int16_t RC_Config::Get_Channel_Range()
{
  if (!_Fail_Safe)
  {
    RcConstrain = Constrain_16Bits(Input, Min_Pulse, Max_Pulse);
  }
  else
  {
    RcConstrain = Constrain_16Bits(Input, RANGE_MIN, Max_Pulse);
  }
  if (_Reverse)
  {
    RcConstrain = Max_Pulse - (RcConstrain - Min_Pulse);
  }
  if (!RCCONFIG.CancelDeadZone)
  {
    if ((Input > 1450 + _DeadZone) && (Input < 1550 - _DeadZone) && (_DeadZone > 0))
    {
      return MIDDLE_STICKS_PULSE;
    }
  }
  if (RcConstrain > Min_Pulse)
  {
    return (_Min_Pulse + ((int32_t)(_Max_Pulse - _Min_Pulse) * (int32_t)(RcConstrain - Min_Pulse)) / (int32_t)(Max_Pulse - Min_Pulse));
  }
  if (!_Fail_Safe)
  {
    return _Min_Pulse;
  }
  else
  {
    return RcConstrain;
  }
}

void RCConfigClass::Init()
{
  //PULSO MINIMO E MAXIMO PARA TODOS OS CANAIS RÁDIO
  Throttle.Min_Pulse = STORAGEMANAGER.Read_16Bits(THROTTLE_MIN_ADDR);
  Throttle.Max_Pulse = STORAGEMANAGER.Read_16Bits(THROTTLE_MAX_ADDR);
  Yaw.Min_Pulse = STORAGEMANAGER.Read_16Bits(YAW_MIN_ADDR);
  Yaw.Max_Pulse = STORAGEMANAGER.Read_16Bits(YAW_MAX_ADDR);
  Pitch.Min_Pulse = STORAGEMANAGER.Read_16Bits(PITCH_MIN_ADDR);
  Pitch.Max_Pulse = STORAGEMANAGER.Read_16Bits(PITCH_MAX_ADDR);
  Roll.Min_Pulse = STORAGEMANAGER.Read_16Bits(ROLL_MIN_ADDR);
  Roll.Max_Pulse = STORAGEMANAGER.Read_16Bits(ROLL_MAX_ADDR);
  AuxiliarOne.Min_Pulse = MIN_PULSE;
  AuxiliarOne.Max_Pulse = MAX_PULSE;
  AuxiliarTwo.Min_Pulse = MIN_PULSE;
  AuxiliarTwo.Max_Pulse = MAX_PULSE;
  AuxiliarThree.Min_Pulse = MIN_PULSE;
  AuxiliarThree.Max_Pulse = MAX_PULSE;
  AuxiliarFour.Min_Pulse = MIN_PULSE;
  AuxiliarFour.Max_Pulse = MAX_PULSE;
  AuxiliarFive.Min_Pulse = MIN_PULSE;
  AuxiliarFive.Max_Pulse = MAX_PULSE;
  AuxiliarSix.Min_Pulse = MIN_PULSE;
  AuxiliarSix.Max_Pulse = MAX_PULSE;
  AuxiliarSeven.Min_Pulse = MIN_PULSE;
  AuxiliarSeven.Max_Pulse = MAX_PULSE;
  AuxiliarEight.Min_Pulse = MIN_PULSE;
  AuxiliarEight.Max_Pulse = MAX_PULSE;
  //CONFIGURAÇÃO DE TODOS OS CANAIS DO RÁDIO
  //THROTTLE
  Throttle.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Throttle.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(THROTTLE_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ALTITUDE-HOLD)
  Throttle.Set_Reverse(false);
  Throttle.Set_Filter(true);
  Throttle.Set_Fail_Safe(true);
  //YAW
  Yaw.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Yaw.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(YAW_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Yaw.Set_Reverse(false);
  Yaw.Set_Filter(true);
  Yaw.Set_Fail_Safe(false);
  //PITCH
  Pitch.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Pitch.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(PITCH_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Pitch.Set_Reverse(false);
  Pitch.Set_Filter(true);
  Pitch.Set_Fail_Safe(false);
  //ROLL
  Roll.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  Roll.Set_Dead_Zone(STORAGEMANAGER.Read_8Bits(ROLL_DZ_ADDR)); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (ATTITUDE)
  Roll.Set_Reverse(false);
  Roll.Set_Filter(true);
  Roll.Set_Fail_Safe(false);
  //AUX1
  AuxiliarOne.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarOne.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarOne.Set_Reverse(false);
  AuxiliarOne.Set_Filter(true);
  AuxiliarOne.Set_Fail_Safe(false);
  //AUX2
  AuxiliarTwo.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarTwo.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarTwo.Set_Reverse(false);
  AuxiliarTwo.Set_Filter(true);
  AuxiliarTwo.Set_Fail_Safe(false);
  //AUX3
  AuxiliarThree.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarThree.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarThree.Set_Reverse(false);
  AuxiliarThree.Set_Filter(true);
  AuxiliarThree.Set_Fail_Safe(false);
  //AUX4
  AuxiliarFour.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarFour.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarFour.Set_Reverse(false);
  AuxiliarFour.Set_Filter(true);
  AuxiliarFour.Set_Fail_Safe(false);
  //AUX5
  AuxiliarFive.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarFive.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarFive.Set_Reverse(false);
  AuxiliarFive.Set_Filter(true);
  AuxiliarFive.Set_Fail_Safe(false);
  //AUX6
  AuxiliarSix.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarSix.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarSix.Set_Reverse(false);
  AuxiliarSix.Set_Filter(true);
  AuxiliarSix.Set_Fail_Safe(false);
  //AUX7
  AuxiliarSeven.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarSeven.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarSeven.Set_Reverse(false);
  AuxiliarSeven.Set_Filter(true);
  AuxiliarSeven.Set_Fail_Safe(false);
  //AUX8
  AuxiliarEight.Set_Range(MIN_STICKS_PULSE, MAX_STICKS_PULSE);
  AuxiliarEight.Set_Dead_Zone(45); //0...50 - ZONA MORTA NO PONTO MEDIO >> 45 = VALORES ENTRE 1495 E 1505 SÃO CONSIDERADOS 1500 (PERFORMANCE)
  AuxiliarEight.Set_Reverse(false);
  AuxiliarEight.Set_Filter(true);
  AuxiliarEight.Set_Fail_Safe(false);
  //FAZ AS PRIMEIRAS LEITURAS DOS CANAIS PARA A CALIBRAÇÃO DOS ESC'S
  //CORRE AS FUNÇÕES 100 VEZES PARA OBTER OS VALORES ATUAIS DOS CANAIS DO RADIO
  for (uint8_t i = 0; i < 100; i++)
  {
    DECODE.Update();            //FAZ AS PRIMEIRA LEITURAS
    RCCONFIG.Set_Pulse();       //APLICA OS VALORES LIDOS
    RCCONFIG.Update_Channels(); //FAZ A LEITURA DOS CANAIS APÓS A CONFIGURAÇÃO
    SBUS_Update();              //FAZ A LEITURA DOS CANAIS DADO PELA COMUNICAÇÃO SBUS
    IBUS_Update();              //FAZ A LEITURA DOS CANAIS DADO PELA COMUNICAÇÃO IBUS
  }
}

void RCConfigClass::Set_Pulse()
{
  Throttle.Set_Pulse(DirectRadioControllRead[THROTTLE]);
  Yaw.Set_Pulse(DirectRadioControllRead[YAW]);
  Pitch.Set_Pulse(DirectRadioControllRead[PITCH]);
  Roll.Set_Pulse(DirectRadioControllRead[ROLL]);
  AuxiliarOne.Set_Pulse(DirectRadioControllRead[AUX1]);
  AuxiliarTwo.Set_Pulse(DirectRadioControllRead[AUX2]);
  AuxiliarThree.Set_Pulse(DirectRadioControllRead[AUX3]);
  AuxiliarFour.Set_Pulse(DirectRadioControllRead[AUX4]);
  AuxiliarFive.Set_Pulse(DirectRadioControllRead[AUX5]);
  AuxiliarSix.Set_Pulse(DirectRadioControllRead[AUX6]);
  AuxiliarSeven.Set_Pulse(DirectRadioControllRead[AUX7]);
  AuxiliarEight.Set_Pulse(DirectRadioControllRead[AUX8]);
}

void RCConfigClass::Update_Channels()
{
  if (!Lock_UP)
  {
    RadioControllOutput[THROTTLE] = StoredValueOfThrottle = Throttle.Output;
  }
  else
  {
    RadioControllOutput[THROTTLE] = StoredValueOfThrottle;
  }
  RadioControllOutput[YAW] = Yaw.Output;
  RadioControllOutput[PITCH] = Pitch.Output;
  RadioControllOutput[ROLL] = Roll.Output;
  RadioControllOutput[AUX1] = AuxiliarOne.Output;
  RadioControllOutput[AUX2] = AuxiliarTwo.Output;
  RadioControllOutput[AUX3] = AuxiliarThree.Output;
  RadioControllOutput[AUX4] = AuxiliarFour.Output;
  RadioControllOutput[AUX5] = AuxiliarFive.Output;
  RadioControllOutput[AUX6] = AuxiliarSix.Output;
  RadioControllOutput[AUX7] = AuxiliarSeven.Output;
  RadioControllOutput[AUX8] = AuxiliarEight.Output;
}
