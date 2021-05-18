import datetime
import pathlib
import enum


class AddrSizeOf(enum.Enum):
    TYPE_8_BITS = 1
    TYPE_16_BITS = 2
    TYPE_32_BITS = 4
    TYPE_FLOAT = 4


class StorageSizeOf(enum.Enum):
    # 475 BYTES DO ARMAZENAMENTO RESERVADOS PARA O CLI
    CLI_SIZE_INITIAL_RESERVED = 0x00
    CLI_SIZE_FINAL_RESERVED = 0x1DB
    # 215 BYTES DO ARMAZENAMENTO RESERVADOS PARA AS CONFIGURAÇÕES NORMAIS
    NORMAL_CONFIG_SIZE_INITIAL_RESERVED = 0x1E0
    NORMAL_CONFIG_SIZE_FINAL_RESERVED = 0x2B7
    # 109 BYTES DO ARMAZENAMENTO RESERVADOS PARA AS CONFIGURAÇÕES NORMAIS
    WAYPOINT_SIZE_INITIAL_RESERVED = 0x2C0
    WAYPOINT_SIZE_FINAL_RESERVED = 0x32D


DefsTable = [
    ['Nome',            'Tamanho'],
    ['KP_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_ACC_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KP_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
    ['KI_MAG_AHRS_ADDR', AddrSizeOf.TYPE_8_BITS.value],
]

# MAPEAMENTO DE ARMAZENAMENTO,SE O ENDEREÇO FINAL FOR ULTRAPASSADO,UM ERRO DE COMPILAÇÃO DEVE SER GERADO,ESSA É A IDEIA
StorageLayout = [
    ['Nome', 'Endereço Inicial', 'Endereço Final'],
    ['CLI', StorageSizeOf.CLI_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.CLI_SIZE_FINAL_RESERVED.value],
    ['NORMAL_CONFIG', StorageSizeOf.NORMAL_CONFIG_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.NORMAL_CONFIG_SIZE_FINAL_RESERVED.value],
    ['WAYPOINT', StorageSizeOf.WAYPOINT_SIZE_INITIAL_RESERVED.value,
        StorageSizeOf.WAYPOINT_SIZE_FINAL_RESERVED.value],
]


def Format_Entry(x):
    return '%d' % round(x)


def Generate_Defines(Function, DefineName, StorageAddressOffSet):

    Function.write('#define %s ' % DefineName)
    Function.write(Format_Entry(StorageAddressOffSet))
    Function.write("\n")


def Generate_Code(Function, Date):
    # GERA O TOPO DA EXTENSÃO
    Function.write("\
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

    Function.write('#ifndef BAR_H_\n#define BAR_H_\n\n')

    Function.write('//BASE ADDRESS REGISTER\n\n')
    Function.write(
        '//ESSE ARQUIVO FOI GERADO AUTOMATICAMENTE - POR FAVOR,NUNCA O EDITE MANUALMENTE!\n\n')
    Function.write('//ATUALIZADO EM %s\n\n' % Date)

    # PRIMEIRO UPLOAD ADDR
    Function.write('//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE\n')
    Function.write('#define FIRMWARE_FIRST_USAGE_ADDR 1500\n\n')

    NextStorageAddress = 0
    PrevStorageAddress = 0

    print('\n--------------------------------------------------------------------------------')

    for TableSizeCount in range(len(DefsTable) - 1):
        NextStorageAddress = NextStorageAddress + \
            DefsTable[TableSizeCount + 1][1]
        Generate_Defines(
            Function, DefsTable[TableSizeCount + 1][0], PrevStorageAddress)
        print('DEF: %s' % DefsTable[TableSizeCount + 1][0] + '  ADDR DE ARMAZENAMENTO:%d' %
              PrevStorageAddress + '  TAMANHO:%d' % DefsTable[TableSizeCount + 1][1] + ' BYTE OU BYTES')
        PrevStorageAddress = NextStorageAddress

    print('--------------------------------------------------------------------------------\n')

    Function.write('\n#endif\n')


if __name__ == '__main__':

    Output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGENERATED.h'

    Date = datetime.datetime.now()

    with open(Output, 'w') as Function:
        Generate_Code(Function, Date)
