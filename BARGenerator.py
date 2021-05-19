import datetime
import pathlib
import enum

WAYPOINTS_MAXIMUM = 10
OTHERS_PARAMS_MAXIMUM = 3  # ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO


class AddrSizeOf(enum.Enum):
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


StorageLayout = [
    ['Grupo', 'Endereço Inicial', 'Endereço Final'],
    ['CLI', StorageSizeOf.CLI_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CLI_SIZE_FINAL_RESERVED.value],
    ['NORMAL_CONFIG', StorageSizeOf.NORMAL_CONFIG_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.NORMAL_CONFIG_SIZE_FINAL_RESERVED.value],
    ['WAYPOINT', StorageSizeOf.WAYPOINT_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.WAYPOINT_SIZE_FINAL_RESERVED.value],
]

DefsCLITable = [
    ['Nome da Definição', 'OffSet'],
    ['KP_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
]

DefsNormalConfigTable = [
    ['Nome da Definição', 'OffSet'],
    ['ACC_ROLL_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_PITCH_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
    ['ACC_YAW_OFFSET_ADDR', AddrSizeOf.TYPE_16_BITS.value],
]

FinalOfWayPointCoordinates = (
    StorageLayout[3][1] + (WAYPOINTS_MAXIMUM * (AddrSizeOf.TYPE_32_BITS.value * 2)))
FinalOfWayPointCoordinatesWithOffSet = (
    FinalOfWayPointCoordinates + AddrSizeOf.TYPE_32_BITS.value)

DefsWayPointTable = [
    ['Nome da Definição', 'OffSet'],
    ['WAYPOINTS_MAXIMUM', WAYPOINTS_MAXIMUM],
    ['OTHERS_PARAMS_MAXIMUM', OTHERS_PARAMS_MAXIMUM],
    ['INITIAL_ADDR_OF_COORDINATES', StorageLayout[3][1]],
    ['FINAL_ADDR_OF_COORDINATES', FinalOfWayPointCoordinates],
    ['INITIAL_ADDR_OF_OTHERS_PARAMS', FinalOfWayPointCoordinatesWithOffSet],
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

    for TableSizeCount in range(ColumnsCount):
        File.write('#define %s ' % InputTable[TableSizeCount + 1][0])
        File.write(Format_Entry(InputTable[TableSizeCount + 1][1]))
        print('DEF: %s' % InputTable[TableSizeCount + 1][0] + '  ADDR DE ARMAZENAMENTO:%d' %
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
        print('DEF: %s' % InputTable[TableSizeCount + 1][0] + '  ADDR DE ARMAZENAMENTO:%d' %
              (PrevStorageAddress + 1 + InputStorageLayoutMin) + '  TAMANHO:%d' % InputTable[TableSizeCount + 1][1] + ' %s' % Generate_Address_Type_To_Str(InputTable[TableSizeCount + 1][1]))
        PrevStorageAddress = NextStorageAddress

    if (SendMessageSuccess):
        print(InputSuccessMessage)

    print('\n')


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

    File.write('//BASE ADDRESS REGISTER\n\n')
    File.write(
        '//ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NUNCA O EDITE MANUALMENTE!\n\n')
    File.write('//ATUALIZADO EM %s\n\n' % Date)

    # PRIMEIRO UPLOAD ADDR
    File.write('//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE\n')
    File.write('#define FIRMWARE_FIRST_USAGE_ADDR ' + '%d' %
               StorageSizeOf.FIRMWARE_RESERVED_MAGIC_ADDR.value + '\n\n')

    print('\n----------------------------------------------------------------DEFS DO CLI----------------------------------------------------------------')

    File.write('//ADDRs PARA O CLI\n')
    Generate_Info_And_Defines(
        DefsCLITable, StorageLayout[1][1], StorageLayout[1][2], '!!!FALHA!!! OS ADDRs DO CLI ATINGIRAM O NUMERO MAXIMO DE ENDEREÇOS DISPONIVEIS', 'OS ADDRs DO CLI FORAM GERADOS COM SUCESSO!')

    print('----------------------------------------------------------DEFS DAS CONFIG NORMAIS----------------------------------------------------------')

    File.write('\n//ADDRs PARA AS CONFIGS NORMAIS\n')
    Generate_Info_And_Defines(
        DefsNormalConfigTable, StorageLayout[2][1], StorageLayout[2][2], '!!!FALHA!!! OS ADDRs DAS CONFIGS NORMAIS ATINGIRAM O NUMERO MAXIMO DE ENDEREÇOS DISPONIVEIS', 'OS ADDRs DAS CONFIGS FORAM GERADOS COM SUCESSO!')

    print('-----------------------------------------------------------DEFS DO MODO WAYPOINT-----------------------------------------------------------')

    File.write('\n//ADDRs PARA O MODO WAYPOINT\n')
    Generate_WayPoint_Defs(File, DefsWayPointTable)

    print('-------------------------------------------------------------------------------------------------------------------------------------------')


if __name__ == '__main__':

    Output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGENERATED.h'

    Date = datetime.datetime.now()

    with open(Output, 'w') as File:
        Generate_Code(File, Date)
