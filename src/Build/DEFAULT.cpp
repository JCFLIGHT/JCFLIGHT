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
#include "Param/PARAM.h"
#include "FastSerial/PRINTF.h"
#include "WayPointNavigation/WAYPOINT.h"
#ifdef ESP32
#include "EEPROM.h"
#endif

#define MAX_RETRY_COUNT 5 //FORÇA UMA REPETIÇÃO DE 5 VEZES

uint8_t Actual_Format_Version = 10; //1.0

bool CheckActualFormatVersion(void)
{
    static uint8_t System_Version = STORAGEMANAGER.Read_8Bits(FIRMWARE_FIRST_USAGE_ADDR);
    if (System_Version != Actual_Format_Version)
    {
        return false;
    }
    return true;
}

void SetNewActualFormatVersion(void)
{
    STORAGEMANAGER.Write_8Bits(FIRMWARE_FIRST_USAGE_ADDR, Actual_Format_Version);
}

void ClearSensorsCalibration(void)
{
    //LIMPA TODOS OS ENDEREÇOS DA EEPROM QUE SÃO UTILIZADOS PARA ARMAZENAR A CALIBRAÇÃO DOS SENSORES
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_ROLL_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_PITCH_OFFSET_ADDR, 0);
    STORAGEMANAGER.Write_16Bits(MAG_YAW_OFFSET_ADDR, 0);
    //COLOCANDO O GANHO DOS SENSORES NO MAXIMO EVITA DE OCORRER DIVISÕES POR ZERO NO CICLO DE MAQUINA
    STORAGEMANAGER.Write_16Bits(ACC_ROLL_SCALE_ADDR, 4096);
    STORAGEMANAGER.Write_16Bits(ACC_PITCH_SCALE_ADDR, 4096);
    STORAGEMANAGER.Write_16Bits(ACC_YAW_SCALE_ADDR, 4096);
    STORAGEMANAGER.Write_16Bits(MAG_ROLL_GAIN_ADDR, 4096);
    STORAGEMANAGER.Write_16Bits(MAG_PITCH_GAIN_ADDR, 4096);
    STORAGEMANAGER.Write_16Bits(MAG_YAW_GAIN_ADDR, 4096);
}

void RecallAllParams(void)
{
    GCS.Default_All_Configs();
    ClearSensorsCalibration();
    WAYPOINT.Erase();
    PARAM.Default_List();
    SetNewActualFormatVersion();
}

void FirmwareOrganizeAllParams(void)
{
#ifdef ESP32
    EEPROM.begin(SIZE_OF_EEPROM);
#endif
    if (!CheckActualFormatVersion())
    {
        LOG("Restaurando os valores de fabrica dos parametros...");
        for (uint8_t IndexCount = 0; IndexCount < MAX_RETRY_COUNT; IndexCount++)
        {
            RecallAllParams();
        }
        LOG("Ok...Parametros reconfigurados!");
        LINE_SPACE;
    }
}