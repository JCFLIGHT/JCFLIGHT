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
#include "IMU/ACCGYROREAD.h"

ClassCompassRotation COMPASSROTATION;

#define HALF_SQRT_2 0.70710678118654757f

void ClassCompassRotation::Rotate(void)
{
    uint8_t Rotation = STORAGEMANAGER.Read_8Bits(COMPASS_ROTATION_ADDR);
    int16_t AngleCorretion;

    switch (Rotation)
    {

    case ROTATION_NONE:
    case ROTATION_MAX:
        return;

    case ROTATION_YAW_45:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_YAW_90:
    {
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_YAW_135:
    {
        AngleCorretion = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_YAW_180:
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[PITCH] = -IMU.Compass.Read[PITCH];
        return;

    case ROTATION_YAW_225:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[PITCH] - IMU.Compass.Read[ROLL]);
        IMU.Compass.Read[PITCH] = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_YAW_270:
    {
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        return;
    }

    case ROTATION_YAW_315:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[PITCH] - IMU.Compass.Read[ROLL]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_180:
    {
        IMU.Compass.Read[PITCH] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_45:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_90:
    {
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_135:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[PITCH] - IMU.Compass.Read[ROLL]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[PITCH] + IMU.Compass.Read[ROLL]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_PITCH_180:
    {
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_225:
    {
        AngleCorretion = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[PITCH] - IMU.Compass.Read[ROLL]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_270:
    {
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_180_YAW_315:
    {
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_YAW_45:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_YAW_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_YAW_135:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270_YAW_45:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        AngleCorretion = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270_YAW_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270_YAW_135:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        AngleCorretion = -HALF_SQRT_2 * (IMU.Compass.Read[ROLL] + IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[PITCH] = HALF_SQRT_2 * (IMU.Compass.Read[ROLL] - IMU.Compass.Read[PITCH]);
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_PITCH_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_PITCH_270:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -AngleCorretion;
        return;
    }

    case ROTATION_PITCH_180_YAW_90:
    {
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        AngleCorretion = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_PITCH_180_YAW_270:
    {
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_PITCH_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_180_PITCH_90:
    {
        IMU.Compass.Read[PITCH] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270_PITCH_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_PITCH_180:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_270_PITCH_180:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        return;
    }

    case ROTATION_ROLL_90_PITCH_270:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -AngleCorretion;
        return;
    }

    case ROTATION_ROLL_180_PITCH_270:
    {
        IMU.Compass.Read[PITCH] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -AngleCorretion;
        return;
    }

    case ROTATION_ROLL_270_PITCH_270:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_PITCH_180_YAW_90:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[ROLL];
        IMU.Compass.Read[YAW] = -IMU.Compass.Read[YAW];
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = -IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = AngleCorretion;
        return;
    }

    case ROTATION_ROLL_90_YAW_270:
    {
        AngleCorretion = IMU.Compass.Read[YAW];
        IMU.Compass.Read[YAW] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        AngleCorretion = IMU.Compass.Read[ROLL];
        IMU.Compass.Read[ROLL] = IMU.Compass.Read[PITCH];
        IMU.Compass.Read[PITCH] = -AngleCorretion;
        return;
    }

    case ROTATION_YAW_293_PITCH_68_ROLL_90:
    {
        float AngleCorretionX = IMU.Compass.Read[ROLL];
        float AngleCorretionY = IMU.Compass.Read[PITCH];
        float AngleCorretionZ = IMU.Compass.Read[YAW];
        IMU.Compass.Read[ROLL] = 0.143039f * AngleCorretionX + 0.368776f * AngleCorretionY + -0.918446f * AngleCorretionZ;
        IMU.Compass.Read[PITCH] = -0.332133f * AngleCorretionX + -0.856289f * AngleCorretionY + -0.395546f * AngleCorretionZ;
        IMU.Compass.Read[YAW] = -0.932324f * AngleCorretionX + 0.361625f * AngleCorretionY + 0.000000f * AngleCorretionZ;
        return;
    }
    }
}
