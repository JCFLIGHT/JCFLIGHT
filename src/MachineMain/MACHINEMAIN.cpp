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

#include "MACHINEMAIN.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

void MachineInit(void)
{
    //CHECA SE É A PRIMEIRA VEZ QUE O FIRMWARE FOI CARREGADO
    FirmwareOrganizeAllParams();
    //INICIALIZA A SERIAL
    FASTSERIAL.Initialization();
    //INICIALIZA O LED RGB
    RGB.Initialization();
    //INICIALIZA O AHRS
    AHRS.Initialization();
    //CALIBRAÇÃO DOS ESCS
    ESC.Calibration();
    //CARREGA OS VALORES DE CALIBRAÇÃO DA IMU
    UpdateIMUCalibration();
    //INICIALIZA OS DISPOSITIVOS I2C
    I2C.All_Initialization();
    //INICIALIZA AS CONFIGURAÇÕES DOS PINOS IO
    ConfigureRegisters(false);
    //INICIALIZA O TECS
    TECS.Initialization();
    //INICIALIZA O WAYPOINT
    WAYPOINT.Initialization();
    //INICIA O SISTEMA DE TASKS
    TaskSystemInitialization();
}

void MachineRun(void)
{
    //SISTEMA DE TASK EM LOOP INFINITO
    TaskSystemRun();
}