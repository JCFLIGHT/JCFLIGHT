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

#include "FILTERANDRATE.h"
#include "Math/MATHSUPPORT.h"

static const MPUGyroFilterAndRate_Struct MPUGyroConfigs[] = {
    {GYRO_LPF_256HZ, 8000, {MPU_DLPF_256HZ, 0x00}},
    {GYRO_LPF_256HZ, 4000, {MPU_DLPF_256HZ, 0x01}},
    {GYRO_LPF_256HZ, 2000, {MPU_DLPF_256HZ, 0x03}},
    {GYRO_LPF_256HZ, 1000, {MPU_DLPF_256HZ, 0x07}},
    {GYRO_LPF_256HZ, 666, {MPU_DLPF_256HZ, 0x0B}},
    {GYRO_LPF_256HZ, 500, {MPU_DLPF_256HZ, 0x0F}},

    {GYRO_LPF_188HZ, 1000, {MPU_DLPF_188HZ, 0x00}},
    {GYRO_LPF_188HZ, 500, {MPU_DLPF_188HZ, 0x01}},

    {GYRO_LPF_98HZ, 1000, {MPU_DLPF_98HZ, 0x00}},
    {GYRO_LPF_98HZ, 500, {MPU_DLPF_98HZ, 0x01}},

    {GYRO_LPF_42HZ, 1000, {MPU_DLPF_42HZ, 0x00}},
    {GYRO_LPF_42HZ, 500, {MPU_DLPF_42HZ, 0x01}},

    {GYRO_LPF_20HZ, 1000, {MPU_DLPF_20HZ, 0x00}},
    {GYRO_LPF_20HZ, 500, {MPU_DLPF_20HZ, 0x01}},

    {GYRO_LPF_10HZ, 1000, {MPU_DLPF_10HZ, 0x00}},
    {GYRO_LPF_10HZ, 500, {MPU_DLPF_10HZ, 0x01}}};

const MPUGyroFilterAndRate_Struct *IMUChooseGyroConfig(uint8_t DesiredGyroLPF, uint16_t DesiredRateInHz)
{
    int8_t SelectedGyroLPF = MPUGyroConfigs[0].GyroLPF;
    const MPUGyroFilterAndRate_Struct *ReturnPointer = &MPUGyroConfigs[0];

    for (uint16_t IndexCount = 1; IndexCount < (sizeof(MPUGyroConfigs) / sizeof((MPUGyroConfigs)[0])); IndexCount++)
    {
        if (ABS(DesiredGyroLPF - MPUGyroConfigs[IndexCount].GyroLPF) < ABS(DesiredGyroLPF - SelectedGyroLPF))
        {
            SelectedGyroLPF = MPUGyroConfigs[IndexCount].GyroLPF;
            ReturnPointer = &MPUGyroConfigs[IndexCount];
        }
    }

    for (uint16_t IndexCount = 0; IndexCount < (sizeof(MPUGyroConfigs) / sizeof((MPUGyroConfigs)[0])); IndexCount++)
    {
        if ((MPUGyroConfigs[IndexCount].GyroLPF == SelectedGyroLPF) &&
            (ABS(DesiredRateInHz - ReturnPointer->GyroRateInHz) > ABS(DesiredRateInHz - MPUGyroConfigs[IndexCount].GyroRateInHz)))
        {
            ReturnPointer = &MPUGyroConfigs[IndexCount];
        }
    }

    return ReturnPointer;
}