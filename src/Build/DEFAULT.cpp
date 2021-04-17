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
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_SCALE_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_ROLL_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_PITCH_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_YAW_OFFSET_ADDR, 0);
}

void EEPROMClearWayPointStorage()
{
    STORAGEMANAGER.Erase(704, 813);
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