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
#include "Build/BOARDDEFS.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "PerformanceCalibration/PERFORMGYRO.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

IMU_Struct IMU;

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
  BIQUADFILTER.Settings(&BiquadAccLPF[ROLL], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[PITCH], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&BiquadAccLPF[YAW], Biquad_Acc_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  //GERA UM COEFICIENTE PARA O LPF DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroLPF[ROLL], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[PITCH], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  BIQUADFILTER.Settings(&BiquadGyroLPF[YAW], Biquad_Gyro_LPF, 0, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), LPF);
  //CARREGA OS VALORES GUARDADOS DO NOTCH
  Biquad_Acc_Notch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
  Biquad_Gyro_Notch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
  //GERA UM COEFICIENTE PARA O NOTCH DO ACELEROMETRO
  BIQUADFILTER.Settings(&BiquadAccNotch[ROLL], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[PITCH], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
  BIQUADFILTER.Settings(&BiquadAccNotch[YAW], Biquad_Acc_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
  //GERA UM COEFICIENTE PARA O NOTCH DO GYROSCOPIO
  BIQUADFILTER.Settings(&BiquadGyroNotch[ROLL], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[PITCH], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
  BIQUADFILTER.Settings(&BiquadGyroNotch[YAW], Biquad_Gyro_Notch, 1, SCHEDULER_SET_PERIOD_US(THIS_LOOP_FREQUENCY), NOTCH);
#endif
}

void Acc_Initialization()
{
  I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x1C, 0x10);
  if (COMPASS.Type == COMPASS_HMC5883 && I2CResources.Found.Compass)
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
  if (I2CResources.Found.Compass)
  {
    I2C.WriteRegister(ADDRESS_IMU_MPU6050, 0x37, 0x02);
  }
}

#ifdef ESP32
void IMU_Get_Data()
{
  uint8_t Data_Read[14];
  I2C.RegisterBuffer(ADDRESS_IMU_MPU6050, 0x3B, &Data_Read[0], 14);
  IMU.Accelerometer.Read[ROLL] = -(Data_Read[0] << 8 | Data_Read[1]);
  IMU.Accelerometer.Read[PITCH] = -(Data_Read[2] << 8 | Data_Read[3]);
  IMU.Accelerometer.Read[YAW] = Data_Read[4] << 8 | Data_Read[5];
  IMU.Gyroscope.Read[PITCH] = -(Data_Read[8] << 8 | Data_Read[9]);
  IMU.Gyroscope.Read[ROLL] = Data_Read[10] << 8 | Data_Read[11];
  IMU.Gyroscope.Read[YAW] = -(Data_Read[12] << 8 | Data_Read[13]);
}
#endif

void Acc_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(ADDRESS_IMU_MPU6050, 0x3B);
  IMU.Accelerometer.Read[ROLL] = -(((I2CResources.Buffer.Data[0] << 8) | I2CResources.Buffer.Data[1]) >> 3);
  IMU.Accelerometer.Read[PITCH] = -(((I2CResources.Buffer.Data[2] << 8) | I2CResources.Buffer.Data[3]) >> 3);
  IMU.Accelerometer.Read[YAW] = ((I2CResources.Buffer.Data[4] << 8) | I2CResources.Buffer.Data[5]) >> 3;
#elif defined ESP32
  IMU_Get_Data();
#endif

  Accelerometer_Calibration();

  //OBTÉM OS VALORES DO ACELEROMETRO ANTES DOS FILTROS
  IMU.Accelerometer.ReadNotFiltered[ROLL] = IMU.Accelerometer.Read[ROLL];
  IMU.Accelerometer.ReadNotFiltered[PITCH] = IMU.Accelerometer.Read[PITCH];
  IMU.Accelerometer.ReadNotFiltered[YAW] = IMU.Accelerometer.Read[YAW];

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Acc(IMU.Accelerometer.Read);
  }

#ifndef __AVR_ATmega2560__
  //LPF
  if (Biquad_Acc_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.Accelerometer.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[ROLL], IMU.Accelerometer.Read[ROLL]);
    IMU.Accelerometer.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[PITCH], IMU.Accelerometer.Read[PITCH]);
    IMU.Accelerometer.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadAccLPF[YAW], IMU.Accelerometer.Read[YAW]);
  }

  //NOTCH
  if (Biquad_Acc_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.Accelerometer.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[ROLL], IMU.Accelerometer.Read[ROLL]);
    IMU.Accelerometer.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[PITCH], IMU.Accelerometer.Read[PITCH]);
    IMU.Accelerometer.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadAccNotch[YAW], IMU.Accelerometer.Read[YAW]);
  }
#endif
}

void Gyro_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(ADDRESS_IMU_MPU6050, 0x43);
  IMU.Gyroscope.Read[PITCH] = -(((I2CResources.Buffer.Data[0] << 8) | I2CResources.Buffer.Data[1]) >> 2);
  IMU.Gyroscope.Read[ROLL] = ((I2CResources.Buffer.Data[2] << 8) | I2CResources.Buffer.Data[3]) >> 2;
  IMU.Gyroscope.Read[YAW] = -(((I2CResources.Buffer.Data[4] << 8) | I2CResources.Buffer.Data[5]) >> 2);
#endif

  Gyroscope_Calibration();

  //OBTÉM OS VALORES DO GYROSCOPIO ANTES DOS FILTROS
  IMU.Gyroscope.ReadNotFiltered[ROLL] = IMU.Gyroscope.Read[ROLL];
  IMU.Gyroscope.ReadNotFiltered[PITCH] = IMU.Gyroscope.Read[PITCH];
  IMU.Gyroscope.ReadNotFiltered[YAW] = IMU.Gyroscope.Read[YAW];

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Gyro(IMU.Gyroscope.Read);
  }

#ifndef __AVR_ATmega2560__
  //LPF
  if (Biquad_Gyro_LPF > 0)
  {
    //APLICA O FILTRO
    IMU.Gyroscope.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[ROLL], IMU.Gyroscope.Read[ROLL]);
    IMU.Gyroscope.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[PITCH], IMU.Gyroscope.Read[PITCH]);
    IMU.Gyroscope.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadGyroLPF[YAW], IMU.Gyroscope.Read[YAW]);
  }

  //NOTCH
  if (Biquad_Gyro_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.Gyroscope.Read[ROLL] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[ROLL], IMU.Gyroscope.Read[ROLL]);
    IMU.Gyroscope.Read[PITCH] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[PITCH], IMU.Gyroscope.Read[PITCH]);
    IMU.Gyroscope.Read[YAW] = BIQUADFILTER.ApplyAndGet(&BiquadGyroNotch[YAW], IMU.Gyroscope.Read[YAW]);
  }
#endif
}