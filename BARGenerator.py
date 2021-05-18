import datetime
import pathlib
import enum


class SizeOf(enum.Enum):
    ADDR_TYPE_8_BITS = 1
    ADDR_TYPE_16_BITS = 2
    ADDR_TYPE_32_BITS = 4
    ADDR_TYPE_FLOAT = 4


AddressTable = [
    ['Name',            'Size'],
    ['KP_ACC_AHRS_ADDR', SizeOf.ADDR_TYPE_8_BITS.value],
    ['KI_ACC_AHRS_ADDR', SizeOf.ADDR_TYPE_8_BITS.value],
    ['KP_MAG_AHRS_ADDR', SizeOf.ADDR_TYPE_8_BITS.value],
    ['KI_MAG_AHRS_ADDR', SizeOf.ADDR_TYPE_8_BITS.value],
]


def format_entry(x): return '%d' % round(x)


def Generate_Defines(Function, DefineName, StorageAddressOffSet):

    Function.write('#define %s ' % DefineName)
    Function.write(format_entry(StorageAddressOffSet))
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
    for TableSizeCount in range(len(AddressTable) - 1):
        NextStorageAddress = NextStorageAddress + \
            AddressTable[TableSizeCount + 1][1]
        Generate_Defines(
            Function, AddressTable[TableSizeCount + 1][0], PrevStorageAddress)
        print('DEF: %s' % AddressTable[TableSizeCount + 1][0] + '  ADDR DE ARMAZENAMENTO:%d' %
              PrevStorageAddress + '  TAMANHO:%d' % AddressTable[TableSizeCount + 1][1] + ' BYTE OU BYTES')
        PrevStorageAddress = NextStorageAddress

    Function.write('\n#endif\n')


if __name__ == '__main__':

    Output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGENERATED.h'

    Date = datetime.datetime.now()

    with open(Output, 'w') as Function:
        Generate_Code(Function, Date)
