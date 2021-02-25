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

#include "ROTATION.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Common/ENUM.h"
#include "Common/STRUCTS.h"

ClassCompassRotation COMPASSROTATION;

#define HALF_SQRT_2 0.70710678118654757f
void ClassCompassRotation::Rotate()
{
    uint8_t Rotation = STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR);
    int16_t AngleCorretion;

    switch (Rotation)
    {

    case NONE_ROTATION:
    {
        //SEM ROTAÇÃO PARA O COMPASS
        return;
    }

    case COMPASS_ROTATION_YAW_45_DEGREES:
    {
        //YAW DESLOCADO 45 GRAUS
        AngleCorretion = HALF_SQRT_2 * (IMU.CompassRead[PITCH] + IMU.CompassRead[ROLL]);
        IMU.CompassRead[ROLL] = HALF_SQRT_2 * (IMU.CompassRead[ROLL] - IMU.CompassRead[PITCH]);
        IMU.CompassRead[PITCH] = AngleCorretion;
        return;
    }

    case COMPASS_ROTATION_YAW_315_DEGREES:
    {
        //YAW DESLOCADO 315 GRAUS
        AngleCorretion = HALF_SQRT_2 * (IMU.CompassRead[PITCH] - IMU.CompassRead[ROLL]);
        IMU.CompassRead[ROLL] = HALF_SQRT_2 * (IMU.CompassRead[ROLL] + IMU.CompassRead[PITCH]);
        IMU.CompassRead[PITCH] = AngleCorretion;
        return;
    }

    case COMPASS_ROTATION_ROLL_180_YAW_45_DEGREES:
    {
        //ROTAÇÃO PARA OS GPS M7 E M8 COM COMPASS ONBOARD QUE FICA POR BAIXO DA PCB
        //ROLL DESLOCADO 180 GRAUS + YAW DESLOCADO 45 GRAUS
        AngleCorretion = HALF_SQRT_2 * (IMU.CompassRead[PITCH] + IMU.CompassRead[ROLL]);
        IMU.CompassRead[ROLL] = HALF_SQRT_2 * (IMU.CompassRead[ROLL] - IMU.CompassRead[PITCH]);
        IMU.CompassRead[PITCH] = AngleCorretion;
        IMU.CompassRead[YAW] = -IMU.CompassRead[YAW];
        return;
    }

    case COMPASS_ROTATION_PITCH_180_DEGREES:
    {
        IMU.CompassRead[ROLL] = -IMU.CompassRead[ROLL];
        IMU.CompassRead[YAW] = -IMU.CompassRead[YAW];
        return;
    }
    }
}