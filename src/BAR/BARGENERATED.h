/* 
   Este arquivo faz parte da JCFLIGHT.   
   JCFLIGHT � um software livre: voc� pode redistribu�-lo e/ou modificar 
   sob os termos da GNU General Public License conforme publicada por 
   a Free Software Foundation, seja a vers�o 3 da Licen�a, ou 
   (� sua escolha) qualquer vers�o posterior. 
   
   JCFLIGHT � distribu�do na esperan�a de ser �til, 
   mas SEM QUALQUER GARANTIA; sem mesmo a garantia impl�cita de 
   COMERCIALIZA��O ou ADEQUA��O A UM DETERMINADO FIM. Veja o 
   GNU General Public License para mais detalhes. 
   
   Voc� deve ter recebido uma c�pia da Licen�a P�blica Geral GNU 
   junto com a JCFLIGHT. Caso contr�rio, consulte <http://www.gnu.org/licenses/>. 
*/

#pragma once

/*

BAR - BASE ADDRESS REGISTER

ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NUNCA O EDITE MANUALMENTE!

ATUALIZADO EM 2021-05-19 16:51:35.966526

*/

//INCREMENTE SEMPRE QUE HOUVER UM NOVO RELEASE
#define FIRMWARE_STORAGE_REVISION 10 //1.0

//NUMERO DE BYTES DO ARMAZENAMENTO RESERVADOS PARA USO
#define TOTAL_SIZE_OF_STORAGE 2000

//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE
#define FIRMWARE_MAGIC_ADDRESS 1500

//ADDRs PARA O CLI
#define KP_ACC_AHRS_ADDR 2
#define KI_ACC_AHRS_ADDR 3
#define KP_MAG_AHRS_ADDR 4
#define KI_MAG_AHRS_ADDR 5

//ADDRs PARA AS CONFIGS
#define ACC_ROLL_OFFSET_ADDR 481
#define ACC_PITCH_OFFSET_ADDR 483
#define ACC_YAW_OFFSET_ADDR 485

//CONFIG E ADDRs PARA O MODO WAYPOINT
#define WAYPOINTS_MAXIMUM 10 //NUMERO MAXIMO DE WAYPOINTS SUPORTADO
#define OTHERS_PARAMS_MAXIMUM 3 //ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO
#define INITIAL_ADDR_OF_COORDINATES 1000
#define FINAL_ADDR_OF_COORDINATES 1080
#define INITIAL_ADDR_OF_OTHERS_PARAMS 1084
#define FINAL_ADDR_OF_OTHERS_PARAMS 1495
