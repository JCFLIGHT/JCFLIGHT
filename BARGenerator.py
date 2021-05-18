import datetime
import pathlib
from tabulate import tabulate
import enum


class SizeOf(enum.Enum):
    ADDR_8_BITS = 1
    ADDR_16_BITS = 2
    ADDR_32_BITS = 4
    ADDR_FLOAT = 4


Table = [['Name', 'Size'],
         ['KP_ACC_AHRS_ADDR', SizeOf.ADDR_8_BITS.value],
         ['KI_ACC_AHRS_ADDR', SizeOf.ADDR_8_BITS.value],
         ['KP_MAG_AHRS_ADDR', SizeOf.ADDR_8_BITS.value],
         ['KI_MAG_AHRS_ADDR', SizeOf.ADDR_8_BITS.value]]


def format_entry(x): return '%d' % round(x)


def Generate_Defines(Function, DefineName, ADDROffSet, ADDRMax):

    for ADDRCount in range(ADDRMax):
        Function.write('#define %s ' % DefineName)
        Function.write(format_entry(ADDROffSet))
        Function.write("\n")


def Generate_Defines2(Function, DefineName, ADDROffSet):

    Function.write('#define %s ' % DefineName)
    Function.write(format_entry(ADDROffSet))
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

    # APENAS DE TESTE POR ENQUANTO - SERÁ NECESSARIO CRIAR UMA TABELA CONTENDO O NOME DO PARAMETRO E O TAMANHO QUE ELE OCUPA PARA O OFFSET
    #Generate_Defines(Function, 'KP_ACC_AHRS_ADDR', 1, 1000)

    #Generate_Defines(Function, Table, 1, 1)

    for TableSizeCount in range(len(Table) - 1):
        Generate_Defines2(Function, Table[TableSizeCount + 1][0], TableSizeCount)

    Function.write('\n\n#endif\n')

    print(tabulate(Table, headers='firstrow', tablefmt='fancy_grid'))


if __name__ == '__main__':

    Output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGEN.h'

    Date = datetime.datetime.now()

    with open(Output, 'w') as Function:
        Generate_Code(Function, Date)
