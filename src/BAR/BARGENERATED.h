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

//BASE ADDRESS REGISTER

//ESSE ARQUIVO HEADER FOI GERADO AUTOMATICAMENTE - POR FAVOR,NUNCA O EDITE MANUALMENTE!

//ATUALIZADO EM 2021-05-18 23:36:24.625708

//ADDR PARA VERIFICAR O PRIMEIRO UPLOAD DO FIRMWARE
#define FIRMWARE_FIRST_USAGE_ADDR 1500

//ADDRs PARA O CLI
#define KP_ACC_AHRS_ADDR 2
#define KI_ACC_AHRS_ADDR 3
#define KP_MAG_AHRS_ADDR 4
#define KI_MAG_AHRS_ADDR 5

//ADDRs PARA AS CONFIGS NORMAIS
#define ACC_ROLL_OFFSET_ADDR 481
#define ACC_PITCH_OFFSET_ADDR 483
#define ACC_YAW_OFFSET_ADDR 485
