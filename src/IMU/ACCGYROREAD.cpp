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
#include "Common/VARIABLES.h"
#include "Filters/KALMANFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/MATHSUPPORT.h"
#ifndef __AVR_ATmega2560__
#include "Filters/BIQUADFILTER.h"
#else
#include "Filters/PT1.h"
#endif
#include "Compass/COMPASSREAD.h"
#include "BAR/BAR.h"
#include "SensorAlignment/ALIGNMENT.h"
#include "Build/BOARDDEFS.h"
#include "IMUCALIBRATE.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

#ifndef __AVR_ATmega2560__
//INSTANCIAS PARA O LPF
BiQuadFilter BiquadAccLPF[3];
BiQuadFilter BiquadGyroLPF[3];
//INSTANCIAS PARA O NOTCH
BiQuadFilter BiquadAccNotch[3];
BiQuadFilter BiquadGyroNotch[3];
#else
PT1_Filter_Struct PT1_Acc_Pitch;
PT1_Filter_Struct PT1_Acc_Roll;
PT1_Filter_Struct PT1_Acc_Yaw;
PT1_Filter_Struct PT1_Gyro_Pitch;
PT1_Filter_Struct PT1_Gyro_Roll;
PT1_Filter_Struct PT1_Gyro_Yaw;
#endif

bool ActiveKalman = false;
int16_t Acc_LPF = 0;
int16_t Gyro_LPF = 0;
#ifndef __AVR_ATmega2560__
int16_t Acc_Notch = 0;
int16_t Gyro_Notch = 0;
#endif

void IMU_Filters_Initialization()
{
  //ATUALIZA O ESTADO GUARDADO DO ESTADO DO KALMAN
  if (STORAGEMANAGER.Read_8Bits(KALMAN_ADDR) == 0)
  {
    ActiveKalman = false;
  }
  else
  {
    ActiveKalman = true;
  }
  //CARREGA OS VALORES GUARDADOS DO LPF
  Acc_LPF = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
  Gyro_LPF = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
#ifndef __AVR_ATmega2560__
  //CARREGA OS VALORES GUARDADOS DO NOTCH
  Acc_Notch = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
  Gyro_Notch = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
  //GERA UM COEFICIENTE PARA O LPF DO ACELEROMETRO
  BiquadAccLPF[ROLL].Settings(Acc_LPF, THIS_LOOP_FREQUENCY, LPF);
  BiquadAccLPF[PITCH].Settings(Acc_LPF, THIS_LOOP_FREQUENCY, LPF);
  BiquadAccLPF[YAW].Settings(Acc_LPF, THIS_LOOP_FREQUENCY, LPF);
  //GERA UM COEFICIENTE PARA O LPF DO GYROSCOPIO
  BiquadGyroLPF[ROLL].Settings(Gyro_LPF, THIS_LOOP_FREQUENCY, LPF);
  BiquadGyroLPF[PITCH].Settings(Gyro_LPF, THIS_LOOP_FREQUENCY, LPF);
  BiquadGyroLPF[YAW].Settings(Gyro_LPF, THIS_LOOP_FREQUENCY, LPF);
  //GERA UM COEFICIENTE PARA O NOTCH DO ACELEROMETRO
  BiquadAccNotch[ROLL].Settings(Acc_Notch, THIS_LOOP_FREQUENCY, NOTCH);
  BiquadAccNotch[PITCH].Settings(Acc_Notch, THIS_LOOP_FREQUENCY, NOTCH);
  BiquadAccNotch[YAW].Settings(Acc_Notch, THIS_LOOP_FREQUENCY, NOTCH);
  //GERA UM COEFICIENTE PARA O NOTCH DO GYROSCOPIO
  BiquadGyroNotch[ROLL].Settings(Gyro_Notch, THIS_LOOP_FREQUENCY, NOTCH);
  BiquadGyroNotch[PITCH].Settings(Gyro_Notch, THIS_LOOP_FREQUENCY, NOTCH);
  BiquadGyroNotch[YAW].Settings(Gyro_Notch, THIS_LOOP_FREQUENCY, NOTCH);
#endif
}

void Acc_Initialization()
{
  I2C.WriteRegister(0x68, 0x1C, 0x10);
  if (Compass_Type == COMPASS_HMC5883 && COMPASS.FakeHMC5883Address != 0x0D && I2C.CompassFound)
  {
    I2C.WriteRegister(0x68, 0x6A, 0b00100000);
    I2C.WriteRegister(0x68, 0x37, 0x00);
    I2C.WriteRegister(0x68, 0x24, 0x0D);
    I2C.WriteRegister(0x68, 0x25, 0x80 | MagAddress);
    I2C.WriteRegister(0x68, 0x26, MagRegister);
    I2C.WriteRegister(0x68, 0x27, 0x86);
  }
}

void Gyro_Initialization()
{
  I2C.WriteRegister(0x68, 0x6B, 0x80);
  SCHEDULERTIME.Sleep(50);
  I2C.WriteRegister(0x68, 0x6B, 0x03);
  switch (STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR)) //LPF INTERNO DA IMU
  {

  case 0:
    I2C.WriteRegister(0x68, 0x1A, 4); //20HZ
    break;

  case 1:
    I2C.WriteRegister(0x68, 0x1A, 3); //42HZ
    break;

  case 2:
    I2C.WriteRegister(0x68, 0x1A, 2); //98HZ
    break;

  case 3:
    I2C.WriteRegister(0x68, 0x1A, 1); //188HZ
    break;

  case 4:
    I2C.WriteRegister(0x68, 0x1A, 0); //256HZ
    break;
  }
  I2C.WriteRegister(0x68, 0x1B, 0x18);
  if (I2C.CompassFound)
  {
    I2C.WriteRegister(0x68, 0x37, 0x02);
  }
}

#ifdef ESP32
void IMU_Get_Data()
{
  uint8_t Data_Read[14];
  I2C.RegisterBuffer(0x68, 0x3B, &Data_Read[0], 14);
  IMU.AccelerometerRead[PITCH] = -(Data_Read[0] << 8 | Data_Read[1]);
  IMU.AccelerometerRead[ROLL] = -(Data_Read[2] << 8 | Data_Read[3]);
  IMU.AccelerometerRead[YAW] = Data_Read[4] << 8 | Data_Read[5];
  IMU.GyroscopeRead[PITCH] = -(Data_Read[8] << 8 | Data_Read[9]);
  IMU.GyroscopeRead[ROLL] = Data_Read[10] << 8 | Data_Read[11];
  IMU.GyroscopeRead[YAW] = -(Data_Read[12] << 8 | Data_Read[13]);
}
#endif

void Acc_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(0x68, 0x3B);
  IMU.AccelerometerRead[PITCH] = -((BufferData[0] << 8) | BufferData[1]) >> 3;
  IMU.AccelerometerRead[ROLL] = -(((BufferData[2] << 8) | BufferData[3]) >> 3);
  IMU.AccelerometerRead[YAW] = ((BufferData[4] << 8) | BufferData[5]) >> 3;
#elif defined ESP32
  IMU_Get_Data();
#endif

  Accelerometer_Calibration();

  if (CalibratingAccelerometer > 0)
  {
    return;
  }

  //OBTÉM OS VALORES DO ACELEROMETRO ANTES DOS FILTROS
  IMU.AccelerometerReadNotFiltered[ROLL] = IMU.AccelerometerRead[ROLL];
  IMU.AccelerometerReadNotFiltered[PITCH] = IMU.AccelerometerRead[PITCH];
  IMU.AccelerometerReadNotFiltered[YAW] = IMU.AccelerometerRead[YAW];

  //APLICA O AJUSTE DO ACELEROMETRO
  ApplySensorAlignment(IMU.AccelerometerRead);

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Acc(IMU.AccelerometerRead);
  }

  //LPF
  if (Acc_LPF > 0)
  {
//APLICA O FILTRO
#ifndef __AVR_ATmega2560__
    IMU.AccelerometerRead[ROLL] = BiquadAccLPF[ROLL].FilterOutput(IMU.AccelerometerRead[ROLL]);
    IMU.AccelerometerRead[PITCH] = BiquadAccLPF[PITCH].FilterOutput(IMU.AccelerometerRead[PITCH]);
    IMU.AccelerometerRead[YAW] = BiquadAccLPF[YAW].FilterOutput(IMU.AccelerometerRead[YAW]);
#else
    IMU.AccelerometerRead[ROLL] = (int16_t)PT1FilterApply(&PT1_Acc_Roll, IMU.AccelerometerReadNotFiltered[ROLL], Acc_LPF, 1.0f / 1000);
    IMU.AccelerometerRead[PITCH] = (int16_t)PT1FilterApply(&PT1_Acc_Pitch, IMU.AccelerometerReadNotFiltered[PITCH], Acc_LPF, 1.0f / 1000);
    IMU.AccelerometerRead[YAW] = (int16_t)PT1FilterApply(&PT1_Acc_Yaw, IMU.AccelerometerReadNotFiltered[YAW], Acc_LPF, 1.0f / 1000);
#endif
  }

#ifndef __AVR_ATmega2560__
  //NOTCH
  if (Acc_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = BiquadAccNotch[ROLL].FilterOutput(IMU.AccelerometerRead[ROLL]);
    IMU.AccelerometerRead[PITCH] = BiquadAccNotch[PITCH].FilterOutput(IMU.AccelerometerRead[PITCH]);
    IMU.AccelerometerRead[YAW] = BiquadAccNotch[YAW].FilterOutput(IMU.AccelerometerRead[YAW]);
  }
#endif
}

void Gyro_ReadBufferData()
{
#ifdef __AVR_ATmega2560__
  I2C.SensorsRead(0x68, 0x43);
  IMU.GyroscopeRead[PITCH] = -(((BufferData[0] << 8) | BufferData[1]) >> 2);
  IMU.GyroscopeRead[ROLL] = ((BufferData[2] << 8) | BufferData[3]) >> 2;
  IMU.GyroscopeRead[YAW] = -((BufferData[4] << 8) | BufferData[5]) >> 2;
#endif

  Gyroscope_Calibration();

  if (CalibratingGyroscope > 0)
  {
    return;
  }

  //OBTÉM OS VALORES DO GYROSCOPIO ANTES DOS FILTROS
  IMU.GyroscopeReadNotFiltered[ROLL] = IMU.GyroscopeRead[ROLL];
  IMU.GyroscopeReadNotFiltered[PITCH] = IMU.GyroscopeRead[PITCH];
  IMU.GyroscopeReadNotFiltered[YAW] = IMU.GyroscopeRead[YAW];

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Gyro(IMU.GyroscopeRead);
  }

  //LPF
  if (Gyro_LPF > 0)
  {
//APLICA O FILTRO
#ifndef __AVR_ATmega2560__
    IMU.GyroscopeRead[ROLL] = BiquadGyroLPF[ROLL].FilterOutput(IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BiquadGyroLPF[PITCH].FilterOutput(IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BiquadGyroLPF[YAW].FilterOutput(IMU.GyroscopeRead[YAW]);
#else
    IMU.GyroscopeRead[ROLL] = (int16_t)PT1FilterApply(&PT1_Gyro_Roll, IMU.GyroscopeReadNotFiltered[ROLL], Gyro_LPF, 1.0f / 1000);
    IMU.GyroscopeRead[PITCH] = (int16_t)PT1FilterApply(&PT1_Gyro_Pitch, IMU.GyroscopeReadNotFiltered[PITCH], Gyro_LPF, 1.0f / 1000);
    IMU.GyroscopeRead[YAW] = (int16_t)PT1FilterApply(&PT1_Gyro_Yaw, IMU.GyroscopeReadNotFiltered[YAW], Gyro_LPF, 1.0f / 1000);
#endif
  }

#ifndef __AVR_ATmega2560__
  //NOTCH
  if (Gyro_Notch > 0)
  {
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = BiquadGyroNotch[ROLL].FilterOutput(IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BiquadGyroNotch[PITCH].FilterOutput(IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BiquadGyroNotch[YAW].FilterOutput(IMU.GyroscopeRead[YAW]);
  }
#endif
}
