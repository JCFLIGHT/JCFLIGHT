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

#ifndef ACCGYROREAD_H_
#define ACCGYROREAD_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern IMU_Struct IMU;
#define ACC_1G 512                 //1G NA IMU - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GYRO_SCALE (1.0f / 16.4f)  //16.4 - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GRAVITY_CMSS 980.665f      //VALOR DA GRAVIDADE EM CM/S^2
#define CALIBRATING_ACC_CYCLES 200 //NÚMERO DE CICLOS PARA CALIBRAR O ACELEROMETRO
void IMU_Filters_Initialization();
void Acc_Initialization();
void Gyro_Initialization();
void Acc_ReadBufferData();
void Gyro_ReadBufferData();
#endif
