import datetime
import pathlib


def format_entry(x): return '%d' % round(x)


def Generate_Defines(f, DefineName, ADDROffSet, ADDRMax):

    for i in range(ADDRMax):
        f.write('#define %s ' % DefineName)
        f.write(format_entry(ADDROffSet))
        if i != ADDRMax - 1:
            f.write("\n")


def Generate_Code(f, Date):
    #GERA O TOPO DA EXTENSÃO
    f.write('#pragma once\n\n')
    f.write('//BASE ADDRESS REGISTER\n\n')
    f.write(
        '//ESSE ARQUIVO FOI GERADO AUTOMATICAMENTE - POR FAVOR EVITE DE EDITAR O MESMO MANUALMENTE!\n\n')
    f.write('//ATUALIZADO EM %s\n\n' % Date)

    # PRIMEIRO UPLOAD ADDR
    f.write('//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE\n')
    f.write('#define FIRMWARE_FIRST_USAGE_ADDR 1500\n\n')
    
    #APENAS TESTE POR ENQUANTO - SERÁ NECESSARIO CRIAR UMA TABELA CONTENDO O NOME DO PARAMETRO E TAMANHO QUE ELE OCUPA (OffSet)
    Generate_Defines(f, 'KP_ACC_AHRS_ADDR', 1, 1000)


if __name__ == '__main__':

    output = pathlib.PurePath(__file__).parent / \
        'src' / 'BAR' / 'BARGEN.h'

    date = datetime.datetime.now()

    with open(output, 'w') as f:
        Generate_Code(f, date)
