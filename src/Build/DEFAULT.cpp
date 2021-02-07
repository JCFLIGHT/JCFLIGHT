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

#include "DEFAULT.h"
#include "IOMCU/IOMCU.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "BOARDDEFS.h"
#ifdef ESP32
#include "EEPROM.h"
#endif

#define HIGH_BYTE_CHECK 0x80

bool StorageCheckPassed()
{
    if (STORAGEMANAGER.Read_8Bits(FIRST_LINK_ADDR) < HIGH_BYTE_CHECK)
    {
        return false;
    }
    return true;
}

void SetHighByteInAddress()
{
    STORAGEMANAGER.Write_8Bits(FIRST_LINK_ADDR, HIGH_BYTE_CHECK);
}

void RecallDefaultConfiguration()
{
    GCS.Default_All_Configs();
}

void EEPROMClearSensorsCalibration()
{
    //LIMPA TODOS OS ENDEREÇOS DA EEPROM QUE SÃO UTILIZADOS PARA ARMAZENAR A CALIBRAÇÃO DOS SENSORES
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_ROLL_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_PITCH_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_YAW_ADDR, 0);
}

void EEPROMClearWayPointStorage()
{
    //4 BYTES DA PRIMEIRA LATITUDE
    STORAGEMANAGER.Write_8Bits(704, 0);
    STORAGEMANAGER.Write_8Bits(705, 0);
    STORAGEMANAGER.Write_8Bits(706, 0);
    STORAGEMANAGER.Write_8Bits(707, 0);
    //4 BYTES DA SEGUNDA LATITUDE
    STORAGEMANAGER.Write_8Bits(708, 0);
    STORAGEMANAGER.Write_8Bits(709, 0);
    STORAGEMANAGER.Write_8Bits(710, 0);
    STORAGEMANAGER.Write_8Bits(711, 0);
    //4 BYTES DA TERCEIRA LATITUDE
    STORAGEMANAGER.Write_8Bits(712, 0);
    STORAGEMANAGER.Write_8Bits(713, 0);
    STORAGEMANAGER.Write_8Bits(714, 0);
    STORAGEMANAGER.Write_8Bits(715, 0);
    //4 BYTES DA QUARTA LATITUDE
    STORAGEMANAGER.Write_8Bits(716, 0);
    STORAGEMANAGER.Write_8Bits(717, 0);
    STORAGEMANAGER.Write_8Bits(718, 0);
    STORAGEMANAGER.Write_8Bits(719, 0);
    //4 BYTES DA QUINTA LATITUDE
    STORAGEMANAGER.Write_8Bits(720, 0);
    STORAGEMANAGER.Write_8Bits(721, 0);
    STORAGEMANAGER.Write_8Bits(722, 0);
    STORAGEMANAGER.Write_8Bits(723, 0);
    //4 BYTES DA SEXTA LATITUDE
    STORAGEMANAGER.Write_8Bits(724, 0);
    STORAGEMANAGER.Write_8Bits(725, 0);
    STORAGEMANAGER.Write_8Bits(726, 0);
    STORAGEMANAGER.Write_8Bits(727, 0);
    //4 BYTES DA SETIMA LATITUDE
    STORAGEMANAGER.Write_8Bits(728, 0);
    STORAGEMANAGER.Write_8Bits(729, 0);
    STORAGEMANAGER.Write_8Bits(730, 0);
    STORAGEMANAGER.Write_8Bits(731, 0);
    //4 BYTES DA OITAVA LATITUDE
    STORAGEMANAGER.Write_8Bits(732, 0);
    STORAGEMANAGER.Write_8Bits(733, 0);
    STORAGEMANAGER.Write_8Bits(734, 0);
    STORAGEMANAGER.Write_8Bits(735, 0);
    //4 BYTES DA NONA LATITUDE
    STORAGEMANAGER.Write_8Bits(736, 0);
    STORAGEMANAGER.Write_8Bits(737, 0);
    STORAGEMANAGER.Write_8Bits(738, 0);
    STORAGEMANAGER.Write_8Bits(739, 0);
    //4 BYTES DA DECIMA LATITUDE
    STORAGEMANAGER.Write_8Bits(740, 0);
    STORAGEMANAGER.Write_8Bits(741, 0);
    STORAGEMANAGER.Write_8Bits(742, 0);
    STORAGEMANAGER.Write_8Bits(743, 0);
    //4 BYTES DA PRIMEIRA LONGITUDE
    STORAGEMANAGER.Write_8Bits(744, 0);
    STORAGEMANAGER.Write_8Bits(745, 0);
    STORAGEMANAGER.Write_8Bits(746, 0);
    STORAGEMANAGER.Write_8Bits(747, 0);
    //4 BYTES DA SEGUNDA LONGITUDE
    STORAGEMANAGER.Write_8Bits(748, 0);
    STORAGEMANAGER.Write_8Bits(749, 0);
    STORAGEMANAGER.Write_8Bits(750, 0);
    STORAGEMANAGER.Write_8Bits(751, 0);
    //4 BYTES DA TERCEIRA LONGITUDE
    STORAGEMANAGER.Write_8Bits(752, 0);
    STORAGEMANAGER.Write_8Bits(753, 0);
    STORAGEMANAGER.Write_8Bits(754, 0);
    STORAGEMANAGER.Write_8Bits(755, 0);
    //4 BYTES DA QUARTA LONGITUDE
    STORAGEMANAGER.Write_8Bits(756, 0);
    STORAGEMANAGER.Write_8Bits(757, 0);
    STORAGEMANAGER.Write_8Bits(758, 0);
    STORAGEMANAGER.Write_8Bits(759, 0);
    //4 BYTES DA QUINTA LONGITUDE
    STORAGEMANAGER.Write_8Bits(760, 0);
    STORAGEMANAGER.Write_8Bits(761, 0);
    STORAGEMANAGER.Write_8Bits(762, 0);
    STORAGEMANAGER.Write_8Bits(763, 0);
    //4 BYTES DA SEXTA LONGITUDE
    STORAGEMANAGER.Write_8Bits(764, 0);
    STORAGEMANAGER.Write_8Bits(765, 0);
    STORAGEMANAGER.Write_8Bits(766, 0);
    STORAGEMANAGER.Write_8Bits(767, 0);
    //4 BYTES DA SETIMA LONGITUDE
    STORAGEMANAGER.Write_8Bits(768, 0);
    STORAGEMANAGER.Write_8Bits(769, 0);
    STORAGEMANAGER.Write_8Bits(770, 0);
    STORAGEMANAGER.Write_8Bits(771, 0);
    //4 BYTES DA OITAVA LONGITUDE
    STORAGEMANAGER.Write_8Bits(772, 0);
    STORAGEMANAGER.Write_8Bits(773, 0);
    STORAGEMANAGER.Write_8Bits(774, 0);
    STORAGEMANAGER.Write_8Bits(775, 0);
    //4 BYTES DA NONA LONGITUDE
    STORAGEMANAGER.Write_8Bits(776, 0);
    STORAGEMANAGER.Write_8Bits(777, 0);
    STORAGEMANAGER.Write_8Bits(778, 0);
    STORAGEMANAGER.Write_8Bits(779, 0);
    //4 BYTES DA DECIMA LONGITUDE
    STORAGEMANAGER.Write_8Bits(780, 0);
    STORAGEMANAGER.Write_8Bits(781, 0);
    STORAGEMANAGER.Write_8Bits(782, 0);
    STORAGEMANAGER.Write_8Bits(783, 0);
    //LIMPA O TIMER DAS MISSÕES COM GPS-HOLD
    STORAGEMANAGER.Write_8Bits(784, 0);
    STORAGEMANAGER.Write_8Bits(785, 0);
    STORAGEMANAGER.Write_8Bits(786, 0);
    STORAGEMANAGER.Write_8Bits(787, 0);
    STORAGEMANAGER.Write_8Bits(788, 0);
    STORAGEMANAGER.Write_8Bits(789, 0);
    STORAGEMANAGER.Write_8Bits(790, 0);
    STORAGEMANAGER.Write_8Bits(791, 0);
    STORAGEMANAGER.Write_8Bits(792, 0);
    STORAGEMANAGER.Write_8Bits(793, 0);
    //LIMPA O MODO DE VOO DAS MISSÕES
    STORAGEMANAGER.Write_8Bits(794, 0);
    STORAGEMANAGER.Write_8Bits(795, 0);
    STORAGEMANAGER.Write_8Bits(796, 0);
    STORAGEMANAGER.Write_8Bits(797, 0);
    STORAGEMANAGER.Write_8Bits(798, 0);
    STORAGEMANAGER.Write_8Bits(799, 0);
    STORAGEMANAGER.Write_8Bits(800, 0);
    STORAGEMANAGER.Write_8Bits(801, 0);
    STORAGEMANAGER.Write_8Bits(802, 0);
    STORAGEMANAGER.Write_8Bits(803, 0);
    //LIMPA A ALTITUDE DAS MISSÕES
    STORAGEMANAGER.Write_8Bits(804, 0);
    STORAGEMANAGER.Write_8Bits(805, 0);
    STORAGEMANAGER.Write_8Bits(806, 0);
    STORAGEMANAGER.Write_8Bits(807, 0);
    STORAGEMANAGER.Write_8Bits(808, 0);
    STORAGEMANAGER.Write_8Bits(809, 0);
    STORAGEMANAGER.Write_8Bits(810, 0);
    STORAGEMANAGER.Write_8Bits(811, 0);
    STORAGEMANAGER.Write_8Bits(812, 0);
    STORAGEMANAGER.Write_8Bits(813, 0);
}

void CheckFirstLinkOrganizeEEPROM()
{
#ifdef ESP32
    EEPROM.begin(SIZE_OF_EEPROM);
#endif
    if (!StorageCheckPassed())
    {
        for (uint8_t i = 0; i < 5; i++) //FORÇA UMA REPETIÇÃO DE 5 VEZES
        {
            RecallDefaultConfiguration();
            EEPROMClearSensorsCalibration();
            EEPROMClearWayPointStorage();
            SetHighByteInAddress();
        }
    }
}