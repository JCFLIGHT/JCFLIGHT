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

#pragma once

/*
BAR - BASE ADDRESS REGISTER

ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NÃO O EDITE MANUALMENTE!

ATUALIZADO EM 19/05/2021 AS 23:03:25
*/

//INCREMENTE SEMPRE QUE HOUVER UM NOVO LANÇAMENTO
#define FIRMWARE_STORAGE_REVISION 10 //1.0

//NÚMERO DE BYTES DO ARMAZENAMENTO RESERVADOS PARA USO
#define TOTAL_SIZE_OF_STORAGE 2000

//ENDEREÇO PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE
#define FIRMWARE_MAGIC_ADDRESS 1500

//ENDEREÇOS PARA O CLI
#define KP_ACC_AHRS_ADDR 2
#define KI_ACC_AHRS_ADDR 3
#define KP_MAG_AHRS_ADDR 4
#define KI_MAG_AHRS_ADDR 5

//ENDEREÇOS PARA AS CONFIGS
#define ACC_ROLL_OFFSET_ADDR 481
#define ACC_PITCH_OFFSET_ADDR 483
#define ACC_YAW_OFFSET_ADDR 485

//CONFIGURAÇÕES E ENDEREÇOS PARA O MODO WAYPOINT
#define WAYPOINTS_MAXIMUM 10 //NÚMERO MAXIMO DE WAYPOINTS SUPORTADO
#define OTHERS_PARAMS_MAXIMUM 3 //ALTITUDE,TEMPO DO GPS-HOLD E O MODO DE VOO
#define INITIAL_ADDR_OF_COORDINATES 1000
#define FINAL_ADDR_OF_COORDINATES 1080
#define INITIAL_ADDR_OF_OTHERS_PARAMS 1084
#define FINAL_ADDR_OF_OTHERS_PARAMS 1495
