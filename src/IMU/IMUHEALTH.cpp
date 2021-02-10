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

#include "IMUHEALTH.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

void CheckAndUpdateIMUCalibration()
{
  CALIBRATION.AccelerometerZero[ROLL] = STORAGEMANAGER.Read_16Bits(ACC_ROLL_ADDR);
  CALIBRATION.AccelerometerZero[PITCH] = STORAGEMANAGER.Read_16Bits(ACC_PITCH_ADDR);
  CALIBRATION.AccelerometerZero[YAW] = STORAGEMANAGER.Read_16Bits(ACC_YAW_ADDR);
  CALIBRATION.AccelerometerScale[ROLL] = STORAGEMANAGER.Read_16Bits(ACC_ROLL_SCALE_ADDR);
  CALIBRATION.AccelerometerScale[PITCH] = STORAGEMANAGER.Read_16Bits(ACC_PITCH_SCALE_ADDR);
  CALIBRATION.AccelerometerScale[YAW] = STORAGEMANAGER.Read_16Bits(ACC_YAW_SCALE_ADDR);
  CALIBRATION.Magnetometer[ROLL] = STORAGEMANAGER.Read_16Bits(MAG_ROLL_ADDR);
  CALIBRATION.Magnetometer[PITCH] = STORAGEMANAGER.Read_16Bits(MAG_PITCH_ADDR);
  CALIBRATION.Magnetometer[YAW] = STORAGEMANAGER.Read_16Bits(MAG_YAW_ADDR);
  //APENAS PARA INDICAR PARA O GCS QUE A IMU NÃO ESTÁ CALIBRADA
  if (CALIBRATION.AccelerometerZero[ROLL] == 0 &&
      CALIBRATION.AccelerometerZero[PITCH] == 0 &&
      CALIBRATION.AccelerometerZero[YAW] == 0)
  {
    CALIBRATION.AccelerometerZero[ROLL] = 0x10FE;
  }
}