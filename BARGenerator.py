import datetime
import pathlib
import enum


class AddrSizeOf(enum.Enum):
    TYPE_8_BITS = 0x01
    TYPE_16_BITS = 0x02
    TYPE_32_BITS = 0x04
    TYPE_FLOAT = 0x04


class StorageSizeOf(enum.Enum):
    # 475 BYTES DO ARMAZENAMENTO RESERVADOS PARA O CLI
    CLI_SIZE_INITIAL_RESERVED = 0x00
    CLI_SIZE_FINAL_RESERVED = 0x1DB
    # 215 BYTES DO ARMAZENAMENTO RESERVADOS PARA AS CONFIGURAÇÕES NORMAIS
    NORMAL_CONFIG_SIZE_INITIAL_RESERVED = 0x1E0
    NORMAL_CONFIG_SIZE_FINAL_RESERVED = 0x2B7
    # 109 BYTES DO ARMAZENAMENTO RESERVADOS PARA AS CONFIGURAÇÕES DO MODO WAYPOINT
    WAYPOINT_SIZE_INITIAL_RESERVED = 0x2C0
    WAYPOINT_SIZE_FINAL_RESERVED = 0x32D
    # ENDEREÇO PARA A VERIFICAÇÃO DE UPLOAD DO FIRMWARE
    FIRMWARE_RESERVED_MAGIC_ADDR = 0x5DC


DefsTable = [
    ['Nome da Definição', 'OffSet'],
    ['KP_ACC_AHRS_ADDR',  AddrSizeOf.TYPE_32_BITS.value],
    ['KI_ACC_AHRS_ADDR',  AddrSizeOf.TYPE_32_BITS.value],
    ['KP_MAG_AHRS_ADDR',  AddrSizeOf.TYPE_16_BITS.value],
    ['KI_MAG_AHRS_ADDR',  AddrSizeOf.TYPE_8_BITS.value],
]

# MAPEAMENTO DE ARMAZENAMENTO,SE O ENDEREÇO FINAL FOR ULTRAPASSADO,UM ERRO DE COMPILAÇÃO DEVE SER GERADO,ESSA É A IDEIA
StorageLayout = [
    ['Grupo', 'Endereço Inicial', 'Endereço Final'],
    ['CLI', StorageSizeOf.CLI_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CLI_SIZE_FINAL_RESERVED.value],
    ['NORMAL_CONFIG', StorageSizeOf.NORMAL_CONFIG_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.NORMAL_CONFIG_SIZE_FINAL_RESERVED.value],
    ['WAYPOINT', StorageSizeOf.WAYPOINT_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.WAYPOINT_SIZE_FINAL_RESERVED.value],
]


def Format_Entry(StrIn):
    return '%d' % round(StrIn)


def Generate_Defines(File, DefineName, StorageAddressOffSet):

    File.write('#define %s ' % DefineName)
    File.write(Format_Entry(StorageAddressOffSet))
    File.write("\n")


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
        '//ESSE ARQUIVO FOI GERADO AUTOMATICAMENTE - POR FAVOR,NUNCA O EDITE MANUALMENTE!\n\n')
    File.write('//ATUALIZADO EM %s\n\n' % Date)

    # PRIMEIRO UPLOAD ADDR
    File.write('//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE\n')
    File.write('#define FIRMWARE_FIRST_USAGE_ADDR ' + '%d' %
               StorageSizeOf.FIRMWARE_RESERVED_MAGIC_ADDR.value + '\n\n')

    NextStorageAddress = 0
    PrevStorageAddress = 0

    print('\n--------------------------------------------------------------------------------')

    for TableSizeCount in range(len(DefsTable) - 1):
        NextStorageAddress = NextStorageAddress + \
            DefsTable[TableSizeCount + 1][1]
        Generate_Defines(
            File, DefsTable[TableSizeCount + 1][0], PrevStorageAddress + 1)  # +1 PARA INICIAR A CONTAGEM DO ADDR 1 (UM) AO INVÉS DO 0 (ZERO)
        print('DEF: %s' % DefsTable[TableSizeCount + 1][0] + '  ADDR DE ARMAZENAMENTO:%d' %
              (PrevStorageAddress + 1) + '  TAMANHO:%d' % DefsTable[TableSizeCount + 1][1] + ' BYTE OU BYTES')
        PrevStorageAddress = NextStorageAddress

    print('--------------------------------------------------------------------------------\n')


if __name__ == '__main__':

    Output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGENERATED.h'

    Date = datetime.datetime.now()

    with open(Output, 'w') as File:
        Generate_Code(File, Date)
