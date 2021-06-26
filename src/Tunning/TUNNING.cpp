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

#include "TUNNING.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

TunningClass TUNNING;

enum Tunning_Enum
{
  NONE_TUNNING_MODE = 0,
  TUNNING_NONE_CHANNEL = 0,
  TUNNING_KP_ROLL,
  TUNNING_KI_ROLL,
  TUNNING_KD_ROLL,
  TUNNING_KCD_OR_KFF_ROLL,
  TUNNING_KP_PITCH,
  TUNNING_KI_PITCH,
  TUNNING_KD_PITCH,
  TUNNING_KCD_OR_KFF_PITCH,
  TUNNING_KP_YAW,
  TUNNING_KI_YAW,
  TUNNING_KD_YAW,
  TUNNING_KCD_OR_KFF_YAW,
  TUNNING_PITOT_FACTOR,
};

void TunningClass::Initialization(void)
{
  TUNNING.ChannelControll = STORAGEMANAGER.Read_8Bits(CH_TUNNING_ADDR);
  TUNNING.Mode = STORAGEMANAGER.Read_8Bits(TUNNING_ADDR);
}

void TunningClass::Update(void)
{
  if (TUNNING.Mode == NONE_TUNNING_MODE || TUNNING.ChannelControll == TUNNING_NONE_CHANNEL)
  {
    return;
  }

  switch (TUNNING.Mode)
  {

  case TUNNING_KP_ROLL:
    break;

  case TUNNING_KI_ROLL:
    break;

  case TUNNING_KD_ROLL:
    break;

  case TUNNING_KCD_OR_KFF_ROLL:
    break;

  case TUNNING_KP_PITCH:
    break;

  case TUNNING_KI_PITCH:
    break;

  case TUNNING_KD_PITCH:
    break;

  case TUNNING_KCD_OR_KFF_PITCH:
    break;

  case TUNNING_KP_YAW:
    break;

  case TUNNING_KI_YAW:
    break;

  case TUNNING_KD_YAW:
    break;

  case TUNNING_KCD_OR_KFF_YAW:
    break;

  case TUNNING_PITOT_FACTOR:
    break;
  }
}