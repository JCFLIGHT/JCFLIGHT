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
#include "Scheduler/SCHEDULER.h"
#endif
#include "Compass/COMPASSREAD.h"
#include "BAR/BAR.h"
#include "SensorAlignment/ALIGNMENT.h"
#include "Build/BOARDDEFS.h"
#include "IMUCALIBRATE.h"

#ifndef __AVR_ATmega2560__
//INSTANCIAS PARA O LPF
BiQuadFilter BiquadAccLPF[3];
BiQuadFilter BiquadGyroLPF[3];
//INSTANCIAS PARA O NOTCH
BiQuadFilter BiquadAccNotch[3];
BiQuadFilter BiquadGyroNotch[3];
#else
static FilterApplyFnPTR AccelerometerLPFApplyFn;
static FilterApplyFnPTR GyroscopeLPFApplyFn;
static PT1Filter_Struct AccelerometerLPFState[3];
static PT1Filter_Struct GyroscopeLPFState[3];
#endif

bool ActiveKalman = false;
uint8_t GyroLPF = 0;
int16_t Acc_LPF = 0;
int16_t Gyro_LPF = 0;
#ifndef __AVR_ATmega2560__
int16_t Acc_Notch = 0;
int16_t Gyro_Notch = 0;
#endif
int16_t Acc_LPFStoredInEEPROM = 0;
int16_t Gyro_LPFStoredInEEPROM = 0;
#ifndef __AVR_ATmega2560__
int16_t Acc_NotchStoredInEEPROM = 0;
int16_t Gyro_NotchStoredInEEPROM = 0;
#endif

#ifdef __AVR_ATmega2560__
static void AccFilterLPF_Initialization(FilterApplyFnPTR *ApplyFn, PT1Filter_Struct State[], uint16_t CutOff)
{
  *ApplyFn = (FilterApplyFnPTR)PT1FilterApply2;
  PT1FilterInit(&State[ROLL].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
  PT1FilterInit(&State[PITCH].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
  PT1FilterInit(&State[YAW].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
}

static void GyroFilterLPF_Initialization(FilterApplyFnPTR *ApplyFn, PT1Filter_Struct State[], uint16_t CutOff)
{
  *ApplyFn = (FilterApplyFnPTR)PT1FilterApply2;
  PT1FilterInit(&State[ROLL].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
  PT1FilterInit(&State[PITCH].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
  PT1FilterInit(&State[YAW].PT1, CutOff, SCHEDULER_PERIOD_HZ(THIS_LOOP_FREQUENCY, "Hz") * 1e-6f);
}
#endif

void IMU_Filters_Initialization()
{
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
#else
  //GERA UMA NOVA FREQUENCIA DE CORTE PARA O LPF DO ACELEROMETRO
  AccFilterLPF_Initialization(&AccelerometerLPFApplyFn, AccelerometerLPFState, Acc_LPF);
  //GERA UMA NOVA FREQUENCIA DE CORTE PARA O LPF DO GYROSCOPIO
  GyroFilterLPF_Initialization(&GyroscopeLPFApplyFn, GyroscopeLPFState, Gyro_LPF);
#endif
}

void IMU_Filters_Update()
{
  //ATUALIZA O VALOR GUARDADO DO ESTADO DO KALMAN
  if (STORAGEMANAGER.Read_8Bits(KALMAN_ADDR) == 0)
  {
    ActiveKalman = false;
  }
  else
  {
    ActiveKalman = true;
  }
  //ATUALIZA OS VALORES GUARDADOS DO LPF
  Acc_LPFStoredInEEPROM = STORAGEMANAGER.Read_16Bits(BI_ACC_LPF_ADDR);
  Gyro_LPFStoredInEEPROM = STORAGEMANAGER.Read_16Bits(BI_GYRO_LPF_ADDR);
#ifndef __AVR_ATmega2560__
  //ATUALIZA OS VALORES GUARDADOS DO NOTCH
  Acc_NotchStoredInEEPROM = STORAGEMANAGER.Read_16Bits(BI_ACC_NOTCH_ADDR);
  Gyro_NotchStoredInEEPROM = STORAGEMANAGER.Read_16Bits(BI_GYRO_NOTCH_ADDR);
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O LPF NO ACC
  if (Acc_LPFStoredInEEPROM != Acc_LPF)
  {
    BiquadAccLPF[ROLL].Settings(Acc_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    BiquadAccLPF[PITCH].Settings(Acc_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    BiquadAccLPF[YAW].Settings(Acc_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    Acc_LPF = Acc_LPFStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O LPF NO GYRO
  if (Gyro_LPFStoredInEEPROM != Gyro_LPF)
  {
    BiquadGyroLPF[ROLL].Settings(Gyro_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    BiquadGyroLPF[PITCH].Settings(Gyro_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    BiquadGyroLPF[YAW].Settings(Gyro_LPFStoredInEEPROM, THIS_LOOP_FREQUENCY, LPF);
    Gyro_LPF = Gyro_LPFStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O NOTCH NO ACC
  if (Acc_NotchStoredInEEPROM != Acc_Notch)
  {
    BiquadAccNotch[ROLL].Settings(Acc_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    BiquadAccNotch[PITCH].Settings(Acc_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    BiquadAccNotch[YAW].Settings(Acc_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    Acc_Notch = Acc_NotchStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O NOTCH NO GYRO
  if (Gyro_NotchStoredInEEPROM != Gyro_Notch)
  {
    BiquadGyroNotch[ROLL].Settings(Gyro_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    BiquadGyroNotch[PITCH].Settings(Gyro_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    BiquadGyroNotch[YAW].Settings(Gyro_NotchStoredInEEPROM, THIS_LOOP_FREQUENCY, NOTCH);
    Gyro_Notch = Gyro_NotchStoredInEEPROM;
  }
#else
  //SE NECESSARIO GERA UMA NOVA FREQUENCIA DE CORTE PARA O LPF NO ACC
  if (Acc_LPFStoredInEEPROM != Acc_LPF)
  {
    AccFilterLPF_Initialization(&AccelerometerLPFApplyFn, AccelerometerLPFState, Acc_LPFStoredInEEPROM);
    Acc_LPF = Acc_LPFStoredInEEPROM;
  }
  //SE NECESSARIO GERA UMA NOVA FREQUENCIA DE CORTE PARA O LPF NO GYRO
  if (Gyro_LPFStoredInEEPROM != Gyro_LPF)
  {
    GyroFilterLPF_Initialization(&GyroscopeLPFApplyFn, GyroscopeLPFState, Gyro_LPFStoredInEEPROM);
    Gyro_LPF = Gyro_LPFStoredInEEPROM;
  }
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
  SCHEDULER.Sleep(50);
  I2C.WriteRegister(0x68, 0x6B, 0x03);
  GyroLPF = STORAGEMANAGER.Read_8Bits(GYRO_LPF_ADDR);
  switch (GyroLPF)
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

void Acc_ReadBufferData()
{
  I2C.SensorsRead(0x68, 0x3B);
  IMU.AccelerometerRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]) >> 3;
  IMU.AccelerometerRead[PITCH] = -(((BufferData[2] << 8) | BufferData[3]) >> 3);
  IMU.AccelerometerRead[YAW] = ((BufferData[4] << 8) | BufferData[5]) >> 3;

#ifdef __AVR_ATmega2560__
  Accelerometer_Calibration();
#endif

  if (CalibratingAccelerometer > 0)
  {
    return;
  }

  //OBTÉM OS VALORES DO ACELEROMETRO ANTES DOS FILTROS
  IMU.AccelerometerReadNotFiltered[ROLL] = IMU.AccelerometerRead[ROLL];
  IMU.AccelerometerReadNotFiltered[PITCH] = IMU.AccelerometerRead[PITCH];
  IMU.AccelerometerReadNotFiltered[YAW] = IMU.AccelerometerRead[YAW];

  ApplySensorAlignment(IMU.AccelerometerRead);

  //KALMAN
  if (ActiveKalman)
  {
    KALMAN.Apply_In_Acc(IMU.AccelerometerRead);
  }

  //LPF
  if (Acc_LPFStoredInEEPROM > 0)
  {
#ifndef __AVR_ATmega2560__
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = BiquadAccLPF[ROLL].FilterOutput(IMU.AccelerometerRead[ROLL]);
    IMU.AccelerometerRead[PITCH] = BiquadAccLPF[PITCH].FilterOutput(IMU.AccelerometerRead[PITCH]);
    IMU.AccelerometerRead[YAW] = BiquadAccLPF[YAW].FilterOutput(IMU.AccelerometerRead[YAW]);
#else
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[ROLL], IMU.AccelerometerReadNotFiltered[ROLL]);
    IMU.AccelerometerRead[PITCH] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[PITCH], IMU.AccelerometerReadNotFiltered[PITCH]);
    IMU.AccelerometerRead[YAW] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[YAW], IMU.AccelerometerReadNotFiltered[YAW]);
#endif
  }
#ifndef __AVR_ATmega2560__
  //NOTCH
  if (Acc_NotchStoredInEEPROM > 0)
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
  I2C.SensorsRead(0x68, 0x43);
  IMU.GyroscopeRead[PITCH] = -(((BufferData[0] << 8) | BufferData[1]) >> 2);
  IMU.GyroscopeRead[ROLL] = ((BufferData[2] << 8) | BufferData[3]) >> 2;
  IMU.GyroscopeRead[YAW] = -((BufferData[4] << 8) | BufferData[5]) >> 2;

#ifdef __AVR_ATmega2560__
  Gyroscope_Calibration();
#endif

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
  if (Gyro_LPFStoredInEEPROM > 0)
  {
#ifndef __AVR_ATmega2560__
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = BiquadGyroLPF[ROLL].FilterOutput(IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BiquadGyroLPF[PITCH].FilterOutput(IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BiquadGyroLPF[YAW].FilterOutput(IMU.GyroscopeRead[YAW]);
#else
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[ROLL], IMU.GyroscopeReadNotFiltered[ROLL]);
    IMU.GyroscopeRead[PITCH] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[PITCH], IMU.GyroscopeReadNotFiltered[PITCH]);
    IMU.GyroscopeRead[YAW] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[YAW], IMU.GyroscopeReadNotFiltered[YAW]);
#endif
  }
#ifndef __AVR_ATmega2560__
  //NOTCH
  if (Gyro_NotchStoredInEEPROM > 0)
  {
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = BiquadGyroNotch[ROLL].FilterOutput(IMU.GyroscopeRead[ROLL]);
    IMU.GyroscopeRead[PITCH] = BiquadGyroNotch[PITCH].FilterOutput(IMU.GyroscopeRead[PITCH]);
    IMU.GyroscopeRead[YAW] = BiquadGyroNotch[YAW].FilterOutput(IMU.GyroscopeRead[YAW]);
  }
#endif
}
