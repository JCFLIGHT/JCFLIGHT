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

#include "ACCGYROREAD.h"
#include "I2C/I2C.h"
#include "Filters/KALMANFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#include "Filters/BIQUADFILTER.h"
#include "Scheduler/SCHEDULER.h"
#include "Compass/COMPASSREAD.h"
#include "BAR/BAR.h"
#include "SensorAlignment/ALIGNMENT.h"
#include "Build/BOARDDEFS.h"
#include "PerfomanceCalibration/PERFORMACC.h"
#include "PerfomanceCalibration/PERFORMGYRO.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

#ifndef __AVR_ATmega2560__
//INSTANCIAS PARA O LPF
static BiquadFilter_Struct BiquadAccLPF[3];
static BiquadFilter_Struct BiquadGyroLPF[3];
//INSTANCIAS PARA O NOTCH
static BiquadFilter_Struct BiquadAccNotch[3];
static BiquadFilter_Struct BiquadGyroNotch[3];
#endif

bool ActiveKalman = false;

#ifndef __AVR_ATmega2560__
int16_t Biquad_Acc_LPF = 0;
int16_t Biquad_Gyro_LPF = 0;
int16_t Biquad_Acc_Notch = 0;
int16_t Biquad_Gyro_Notch = 0;
#endif

void IMU_Filters_Initialization()
{
  //ATUALIZA O ESTADO GUARDADO DO ESTADO DO KALMAN
  if (STORAGEMANAGER.Read_8Bits(KALMAN_ADDR) == NONE)
  {
    ActiveKalman = false;
  }
  else
  {
    ActiveKalman = true;
  }
#ifndef __AVR_ATmega2560__
  //CARREGA OS VALORES GUARDADOS DO LPF
  Biquad_Acc_LPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
  Biquad_Gyro_LPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
  //GERA UM COEFICIENTE PARA O LPF DO ACELEROMETRO
  BIQUADFILTER.Settings(&BiquadAccLPF[ROLL], Biquad_Acc_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[PITCH], Biquad_Acc_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[YAW], Biquad_Acc_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  //GERA UM COEFICIENTE PARA O LPF DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroLPF[ROLL], Biquad_Gyro_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[PITCH], Biquad_Gyro_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[YAW], Biquad_Gyro_LPF, 0, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "KHz"), LPF);
  //CARREGA OS VALORES GUARDADOS DO NOTCH
  Biquad_Acc_Notch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
  Biquad_Gyro_Notch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
  //GERA UM COEFICIENTE PARA O NOTCH DO ACELEROMETRO
  BIQUADFILTER.Settings(&BiquadAccNotch[ROLL], Biquad_Acc_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[PITCH], Biquad_Acc_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[YAW], Biquad_Acc_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
  //GERA UM COEFICIENTE PARA O NOTCH DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroNotch[ROLL], Biquad_Gyro_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[PITCH], Biquad_Gyro_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[YAW], Biquad_Gyro_Notch, 1, SCHEDULER_SET_FREQUENCY(THIS_LOOP_FREQUENCY, "HZ"), NOTCH);
#endif
}

void Acc_Initialization()
{
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1C, 0x10);
  if (COMPASS.Type == COMPASS_HMC5883 && I2C.CompassFound)
  {
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6A, 0b00100000);
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x37, 0x00);
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x24, 0x0D);
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x25, 0x80 | COMPASS.Address);
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x26, COMPASS.Register);
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x27, 0x86);
  }
}

void Gyro_Initialization()
{
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6B, 0x80);
  SCHEDULERTIME.Sleep(50);
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x6B, 0x03);
  switch (STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR)) //LPF INTERNO DA IMU
  {

  case 0:
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, 4); //20HZ
    break;

  case 1:
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, 3); //42HZ
    break;

  case 2:
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, 2); //98HZ
    break;

  case 3:
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, 1); //188HZ
    break;

  case 4:
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1A, 0); //256HZ
    break;
  }
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1B, 0x18);
  if (I2C.CompassFound)
  {
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x37, 0x02);
  }
}

#ifdef ESP32
void IMU_Get_Data()
{
  uint8_t Data_Read[14];
  I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x3B, &Data_Read[0], 14);
  IMU.AccelerometerRead[ROLL] = -(Data_Read[0] << 8 | Data_Read[1]);
  IMU.AccelerometerRead[PITCH] = -(Data_Read[2] << 8 | Data_Read[3]);
  IMU.AccelerometerRead[YAW] = Data_Read[4] << 8 | Data_Read[5];
  IMU.GyroscopeRead[PITCH] = -(Data_Read[8] << 8 | Data_Read[9]);
  IMU.GyroscopeRead[ROLL] = Data_Read[10] << 8 | Data_Read[11];
  IMU.GyroscopeRead[YAW] = -(Data_Read[12] << 8 | Data_Read[13]);
}
#endif

void Acc_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(ADDRESS_IMU_MPU6050, 0x3B);
  IMU.AccelerometerRead[ROLL] = -(((BufferData[0] << 8) | BufferData[1]) >> 3);
  IMU.AccelerometerRead[PITCH] = -(((BufferData[2] << 8) | BufferData[3]) >> 3);
  IMU.AccelerometerRead[YAW] = ((BufferData[4] << 8) | BufferData[5]) >> 3;
#elif defined ESP32
  IMU_Get_Data();
#endif

  Accelerometer_Calibration();

  //APLICA O AJUSTE DO ACELEROMETRO
  ApplySensorAlignment(IMU.AccelerometerRead);

  //OBTÉM OS VALORES DO ACELEROMETRO ANTES DOS FILTROS
  IMU.AccelerometerReadNotFiltered[ROLL] = IMU.AccelerometerRead[ROLL];
  IMU.AccelerometerReadNotFiltered[PITCH] = IMU.AccelerometerRead[PITCH];
  IMU.AccelerometerReadNotFiltered[YAW] = IMU.AccelerometerRead[YAW];

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Acc(IMU.AccelerometerRead);
  }

#ifndef __AVR_ATmega2560__

  //LPF
  if (Biquad_Acc_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccLPF[ROLL], IMU.AccelerometerRead[ROLL]);
    IMU.AccelerometerRead[PITCH] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccLPF[PITCH], IMU.AccelerometerRead[PITCH]);
    IMU.AccelerometerRead[YAW] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccLPF[YAW], IMU.AccelerometerRead[YAW]);
  }

  //NOTCH
  if (Biquad_Acc_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccNotch[ROLL], IMU.AccelerometerRead[ROLL]);
    IMU.AccelerometerRead[PITCH] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccNotch[PITCH], IMU.AccelerometerRead[PITCH]);
    IMU.AccelerometerRead[YAW] = BIQUADFILTER.FilterApplyAndGet(&BiquadAccNotch[YAW], IMU.AccelerometerRead[YAW]);
  }
#endif
}

void Gyro_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(ADDRESS_IMU_MPU6050, 0x43);
  IMU.GyroscopeRead[PITCH] = -(((BufferData[0] << 8) | BufferData[1]) >> 2);
  IMU.GyroscopeRead[ROLL] = ((BufferData[2] << 8) | BufferData[3]) >> 2;
  IMU.GyroscopeRead[YAW] = -(((BufferData[4] << 8) | BufferData[5]) >> 2);
#endif

  Gyroscope_Calibration();

  //OBTÉM OS VALORES DO GYROSCOPIO ANTES DOS FILTROS
  IMU.GyroscopeReadNotFiltered[ROLL] = IMU.GyroscopeRead[ROLL];
  IMU.GyroscopeReadNotFiltered[PITCH] = IMU.GyroscopeRead[PITCH];
  IMU.GyroscopeReadNotFiltered[YAW] = IMU.GyroscopeRead[YAW];

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Gyro(IMU.GyroscopeRead);
  }

#ifndef __AVR_ATmega2560__

  //LPF
  if (Biquad_Gyro_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroLPF[ROLL], IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroLPF[PITCH], IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroLPF[YAW], IMU.GyroscopeRead[YAW]);
  }

  //NOTCH
  if (Biquad_Gyro_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroNotch[ROLL], IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroNotch[PITCH], IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BIQUADFILTER.FilterApplyAndGet(&BiquadGyroNotch[YAW], IMU.GyroscopeRead[YAW]);
  }
#endif
}
