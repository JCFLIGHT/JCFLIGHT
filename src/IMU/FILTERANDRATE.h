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

#ifndef FILTERANDRATE_H_
#define FILTERANDRATE_H_
#include "Common/STRUCTS.h"

enum MPU_DLPF_Enum
{
    MPU_DLPF_10HZ = 5,
    MPU_DLPF_20HZ = 4,
    MPU_DLPF_42HZ = 3,
    MPU_DLPF_98HZ = 2,
    MPU_DLPF_188HZ = 1,
    MPU_DLPF = 1,
    MPU_DLPF_256HZ = 0
};

enum MPU_Gyro_LPF_Enum
{
    MPU_GYRO_LPF = 0,
    GYRO_LPF_256HZ = 0,
    GYRO_LPF_188HZ = 1,
    GYRO_LPF_98HZ = 2,
    GYRO_LPF_42HZ = 3,
    GYRO_LPF_20HZ = 4,
    GYRO_LPF_10HZ = 5,
    GYRO_LPF_5HZ = 6,
    GYRO_LPF_NONE = 7
};

typedef struct
{
    uint8_t GyroLPF;
    uint16_t GyroRateInHz;
    uint8_t Gyro_I2C_Register[2];
} MPUGyroFilterAndRate_Struct;

const MPUGyroFilterAndRate_Struct *IMUChooseGyroConfig(uint8_t DesiredGyroLPF, uint16_t DesiredRateInHz);
#endif