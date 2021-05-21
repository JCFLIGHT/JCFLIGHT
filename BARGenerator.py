import codecs
import enum
import pathlib
import datetime
Import("env")


FIRMWARE_STORAGE_REVISION = 10  # 1.0 - INCREMENTE SEMPRE QUE HOUVER UM LANÇAMENTO

WAYPOINTS_MAXIMUM = 10
OTHERS_PARAMS_MAXIMUM = 3  # ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO


class AddrSizeOf(enum.Enum):
    TYPE_NONE = 0x00
    TYPE_8_BITS = 0x01
    TYPE_16_BITS = 0x02
    TYPE_32_BITS = 0x04
    TYPE_FLOAT = 0x04


class StorageSizeOf(enum.Enum):
    # 474 BYTES RESERVADOS DO ARMAZENAMENTO PARA O CLI
    CLI_SIZE_INITIAL_RESERVED = 0x01
    CLI_SIZE_FINAL_RESERVED = 0x1DB
    # 515 BYTES RESERVADOS DO ARMAZENAMENTO PARA AS CONFIGURAÇÕES NORMAIS
    NORMAL_CONFIG_SIZE_INITIAL_RESERVED = 0x1E0
    NORMAL_CONFIG_SIZE_FINAL_RESERVED = 0x3E3
    # 495 BYTES RESERVADOS DO ARMAZENAMENTO PARA AS CONFIGURAÇÕES DO MODO WAYPOINT
    WAYPOINT_SIZE_INITIAL_RESERVED = 0x3E8
    WAYPOINT_SIZE_FINAL_RESERVED = 0x5D7
    # ENDEREÇO PARA A VERIFICAÇÃO DE UPLOAD DO FIRMWARE
    FIRMWARE_RESERVED_MAGIC_ADDR = 0x5DC
    # 2000 BYTES RESERVADOS PARA USO
    TOTAL_SIZE_OF_STORAGE_RESERVED_TO_USE = 0x7D0


StorageLayout = [
    ['Grupo', 'Endereço Inicial', 'Endereço Final'],
    ['CLI', StorageSizeOf.CLI_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CLI_SIZE_FINAL_RESERVED.value],
    ['NORMAL_CONFIG', StorageSizeOf.NORMAL_CONFIG_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.NORMAL_CONFIG_SIZE_FINAL_RESERVED.value],
    ['WAYPOINT', StorageSizeOf.WAYPOINT_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.WAYPOINT_SIZE_FINAL_RESERVED.value],
    ['FIRMWARE_MAGIC_ADDRESS', AddrSizeOf.TYPE_NONE.value,
        StorageSizeOf.FIRMWARE_RESERVED_MAGIC_ADDR.value],
    ['TOTAL_SIZE_OF_STORAGE', AddrSizeOf.TYPE_NONE.value,
        StorageSizeOf.TOTAL_SIZE_OF_STORAGE_RESERVED_TO_USE.value],
]

DefsCLITable = [
    ['Nome da Definição', 'OffSet'],
    ['KP_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_AHRS_BA_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_IMU_BA_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_IMU_GPS_VEL_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_TRIGGER_MOTOR_DELAY_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_ELEVATOR_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AL_SPINUP_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_SPINUP_TIME_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_MAX_THROTTLE_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_EXIT_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AL_ALTITUDE_ADDR', AddrSizeOf.TYPE_32_BITS.value],
    ['BATT_VOLTAGE_FACTOR_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['BATT_AMPS_VOLT_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['BATT_AMPS_OFFSET_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['CC_BANKANGLE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['CC_TIME_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GIMBAL_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['GIMBAL_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['LAND_CHECKACC_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['THROTTLE_FACTOR_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['AUTODISARM_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_THR_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_YPR_MIN_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['AUTODISARM_YPR_MAX_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['WHEELS_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['GPS_BAUDRATE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['NAV_VEL_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['WP_RADIUS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['RTH_LAND_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['GPS_TILT_COMP_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AIRSPEED_SAMPLES_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AIRSPEED_FACTOR_ADDR', AddrSizeOf.TYPE_FLOAT.value],
    ['ARM_TIME_SAFETY_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['DISARM_TIME_SAFETY_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['COMPASS_CAL_TIME_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AUTO_PILOT_MODE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['AS_AUTO_CAL_SCALE_ADDR', AddrSizeOf.TYPE_8_BITS.value],
]


DefsNormalConfigTable = [
    ['Nome da Definição', 'OffSet'],
    ['ACC_ROLL_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_PITCH_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_YAW_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
]

FinalAddrOfWayPointCoordinates = (
    StorageLayout[3][1] + (WAYPOINTS_MAXIMUM * (AddrSizeOf.TYPE_32_BITS.value * 2)))
FinalAddrOfWayPointCoordinatesWithOffSet = (
    FinalAddrOfWayPointCoordinates + AddrSizeOf.TYPE_32_BITS.value)

DefsWayPointTable = [
    ['Nome da Definição', 'OffSet'],
    ['WAYPOINTS_MAXIMUM', WAYPOINTS_MAXIMUM],
    ['OTHERS_PARAMS_MAXIMUM', OTHERS_PARAMS_MAXIMUM],
    ['INITIAL_ADDR_OF_COORDINATES', StorageLayout[3][1]],
    ['FINAL_ADDR_OF_COORDINATES', FinalAddrOfWayPointCoordinates],
    ['INITIAL_ADDR_OF_OTHERS_PARAMS', FinalAddrOfWayPointCoordinatesWithOffSet],
    ['FINAL_ADDR_OF_OTHERS_PARAMS', StorageLayout[3][2]],
]


def Format_Entry(StrIn):

    return '%d' % round(StrIn)


def Generate_Defines(File, DefineName, StorageAddressOffSet):

    File.write('#define %s ' % DefineName)
    File.write(Format_Entry(StorageAddressOffSet))
    File.write("\n")


def Generate_Address_Type_To_Str(AddressSizeOf):

    StringRet = 'BYTES'

    if (AddressSizeOf == AddrSizeOf.TYPE_8_BITS.value):
        StringRet = 'BYTE'
    else:
        StringRet = 'BYTES'

    return StringRet


def Generate_WayPoint_Defs(File, InputTable):

    ColumnsCount = (len(InputTable) - 1)
    StringPrint = ''

    for TableSizeCount in range(ColumnsCount):
        File.write('#define %s ' % InputTable[TableSizeCount + 1][0])
        File.write(Format_Entry(InputTable[TableSizeCount + 1][1]))
        if(TableSizeCount == 0):
            File.write(' //NÚMERO MAXIMO DE WAYPOINTS SUPORTADO')
        if(TableSizeCount == 1):
            File.write(' //ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO')
        if(TableSizeCount > 1):
            StringPrint = 'ENDEREÇO DE ARMAZENAMENTO:'
        else:
            StringPrint = 'VALOR:'
        print('DEF: %s' % InputTable[TableSizeCount + 1][0] + '  %s' % StringPrint + '%d' %
              InputTable[TableSizeCount + 1][1])
        File.write("\n")


def Generate_Info_And_Defines(InputTable, InputStorageLayoutMin, InputStorageLayoutMax, InputErrorMessage, InputSuccessMessage):

    ColumnsCount = (len(InputTable) - 1)
    CheckSum = 0
    NextStorageAddress = 0
    PrevStorageAddress = 0
    SendMessageSuccess = False

    for TableSizeCount in range(ColumnsCount):
        CheckSum = CheckSum + InputTable[TableSizeCount + 1][1]

    for TableSizeCount in range(ColumnsCount):
        if(CheckSum >= InputStorageLayoutMax):
            print(InputErrorMessage)
            break
        SendMessageSuccess = True
        NextStorageAddress = NextStorageAddress + \
            InputTable[TableSizeCount + 1][1]
        Generate_Defines(
            File, InputTable[TableSizeCount + 1][0], PrevStorageAddress + 1 + InputStorageLayoutMin)
        print('DEF: %s' % InputTable[TableSizeCount + 1][0] + '  ENDEREÇO DE ARMAZENAMENTO:%d' %
              (PrevStorageAddress + 1 + InputStorageLayoutMin) + '  TAMANHO:%d' % InputTable[TableSizeCount + 1][1] + ' %s' % Generate_Address_Type_To_Str(InputTable[TableSizeCount + 1][1]))
        PrevStorageAddress = NextStorageAddress

    if (SendMessageSuccess):
        print(InputSuccessMessage)


def Generate_Code(File, Date):
    # GERA O TOPO DA EXTENSÃO
    File.write("\
/* \n\
   Este arquivo faz parte da JCFLIGHT.\
   \n\
   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar \n\
   sob os termos da GNU General Public License conforme publicada por \n\
   a Free Software Foundation, seja a versão 3 da Licença, ou \n\
   (à sua escolha) qualquer versão posterior. \n\
   \n\
   JCFLIGHT é distribuído na esperança de ser útil, \n\
   mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de \n\
   COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o \n\
   GNU General Public License para mais detalhes. \n\
   \n\
   Você deve ter recebido uma cópia da Licença Pública Geral GNU \n\
   junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>. \n\
*/\
\n\n")

    File.write('#pragma once\n\n')
    File.write('/*\n')
    File.write('BAR - BASE ADDRESS REGISTER\n\n')
    File.write(
        'ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NÃO O EDITE MANUALMENTE!\n\n')
    File.write('ATUALIZADO EM %s\n' % Date)
    File.write('*/\n\n')

    File.write('//INCREMENTE SEMPRE QUE HOUVER UM NOVO LANÇAMENTO\n')
    File.write('#define FIRMWARE_STORAGE_REVISION' + ' %d' %
               FIRMWARE_STORAGE_REVISION + ' //%.1f' %
               (FIRMWARE_STORAGE_REVISION / 10) + '\n\n')

    print('\n')
    print('FIRMWARE_STORAGE_REVISION' + ' %.1f' %
          (FIRMWARE_STORAGE_REVISION / 10))

    File.write('//NÚMERO DE BYTES DO ARMAZENAMENTO RESERVADOS PARA USO\n')
    File.write('#define ' + '%s' %
               StorageLayout[5][0] + ' %d' % StorageLayout[5][2] + '\n\n')

    print('%s' % StorageLayout[5][0] + ' %d' % StorageLayout[5][2])

    # ENDEREÇO DO PRIMEIRO UPLOAD
    File.write(
        '//ENDEREÇO PARA VERIFICAR SE É O PRIMEIRO UPLOAD DA VERSÃO DO FIRMWARE\n')
    File.write('#define ' + '%s' %
               StorageLayout[4][0] + ' %d' % StorageLayout[4][2] + '\n\n')

    print('%s' % StorageLayout[4][0] + ' %d' % StorageLayout[4][2])

    print('\n-----------------------------------------------------------DEFINIÇÕES DO CLI------------------------------------------------------------')

    File.write('//ENDEREÇOS PARA O CLI\n')
    Generate_Info_And_Defines(
        DefsCLITable, StorageLayout[1][1], StorageLayout[1][2], '!!!FALHA!!! OS ENDEREÇOS DO CLI ATINGIRAM O NÚMERO MAXIMO DE ENDEREÇOS DISPONIVEIS', 'OS ENDEREÇOS DO CLI FORAM GERADOS COM SUCESSO!')

    print('-------------------------------------------------------------------------------------------------------------------------------------------\n')

    print('-------------------------------------------------------DEFINIÇÕES DAS CONFIGURAÇÕES--------------------------------------------------------')

    File.write('\n//ENDEREÇOS PARA AS CONFIGS\n')
    Generate_Info_And_Defines(
        DefsNormalConfigTable, StorageLayout[2][1], StorageLayout[2][2], '!!!FALHA!!! OS ENDEREÇOS DAS CONFIGURAÇÕES NORMAIS ATINGIRAM O NÚMERO MAXIMO DE ENDEREÇOS DISPONIVEIS', 'OS ENDEREÇOS DAS CONFIGURAÇÕES FORAM GERADOS COM SUCESSO!')

    print('-------------------------------------------------------------------------------------------------------------------------------------------\n')

    print('--------------------------------------------------------DEFINIÇÕES DO MODO WAYPOINT--------------------------------------------------------')

    File.write('\n//CONFIGURAÇÕES E ENDEREÇOS PARA O MODO WAYPOINT\n')
    Generate_WayPoint_Defs(File, DefsWayPointTable)

    print('------------------------------------------------------------------------------------------------------------------------------------------\n')


env.Dump()
try:

    DateAndTime = str(datetime.datetime.now())
    Year = DateAndTime[0:4]
    Day = DateAndTime[5:7]
    Month = DateAndTime[8:10]
    Hours = DateAndTime[11:13]
    Minutes = DateAndTime[14:16]
    Seconds = DateAndTime[17:19]

except:
    print('')

with codecs.open(pathlib.PurePath('__main__').parent / 'src' / 'BAR' / 'BARGENERATED.h', "w", "utf-8-sig") as File:
    Generate_Code(File, "{}/{}/{} ÁS {}:{}:{}".format(Month,
                  Day, Year, Hours, Minutes, Seconds))
