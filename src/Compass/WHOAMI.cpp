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

#include "WHOAMI.h"
#include "I2C/I2C.h"
#include "Compass/COMPASSREAD.h"
#include "FastSerial/PRINTF.h"

//#define PRINTLN_WHOAMI

//É UMA FUNÇÃO DE RETORNO,SE O HMC5x43 FOR ENCONTRADO,A FUNÇÃO RETORNA TRUE,CASO CONTRARIO RETORNA FALSE.
//PRA SABER SE REALMENTE ESTÁ FUNCIONANDO,EU PRECISO ARRUMAR UM COMPASS HMC5843
//E UM HMC5883,AFIM DE VERIFICAR SE ESSA FUNÇÃO REALMENTE FUNCIONA DE FORMA CORRETA,
//ESSA FUNÇÃO FOI RETIRADA DA ARDUPILOT,PORÉM LÁ O BARRAMENTO UTILIZADO É SPI,E AQUI NA JCFLIGHT É I2C,
//MAS A LOGICA DE FUNCIONAMENTO É A MESMA PARA AMBAS.
//SE FUNCIONAR DA MANEIRA CORRETA,ESSA FUNÇÃO SERÁ UTILIZADA PARA SETAR A ORIENTAÇÃO DO HMC5843 EM ORIENTATION.cpp,
//E TODO ESSE COMENTARIO DEVERÁ SER REMOVIDO DAQUI.
bool Check_Whoami()
{
    I2C.SensorsRead(COMPASS.Address, 0x0A);

#ifdef PRINTLN_WHOAMI

    //TABELA ASCII:
    //H =
    //4 =
    //3 =
    DEBUG("BufferData[0]:%d BufferData[1]:%d BufferData[2]:%d",
          BufferData[0],
          BufferData[1],
          BufferData[2]);

#endif

    if (BufferData[0] != 'H' ||
        BufferData[1] != '4' ||
        BufferData[2] != '3')
    {
        //NÃO É UM DISPOSITIVO HMC5x43
        return false;
    }
    return true;
}