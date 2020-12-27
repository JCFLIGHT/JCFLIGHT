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

#include "COMPASSREAD.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "I2C/I2C.h"
#include "LedRGB/LEDRGB.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/AVRMATH.h"
#include "BAR/BAR.h"
#include "Buzzer/BUZZER.h"
#include "IMU/IMUHEALTH.h"

CompassReadClass COMPASS;

//TEMPO DE ATUALIZAÇÃO DA LEITURA DO COMPASS
//#define COMPASS_UPDATE_FREQUENCY 50000 //5Hz
#define COMPASS_UPDATE_FREQUENCY 100000 //10Hz

//TEMPO MAXIMO DE CALIBRAÇÃO DO COMPASS
#define CALIBRATION_TIME 60 //SEGUNDOS

uint8_t MagOrientation = 0;
static float MagnetometerGain[3] = {1.0, 1.0, 1.0};
static int32_t XYZ_CompassBias[3] = {0, 0, 0};

void CompassReadClass::Initialization()
{
  //SAIA DA FUNÇÃO SE NÃO FOR ENCONTRADO NENHUM COMPASS NO BARRAMENTO I2C
  if (!I2C.CompassFound)
    return;
  if (STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) <= 3)
    MagOrientation = GPS_ONBOARD_COMPASS; //COMPASS ONBOARD NO GPS
  else if (STORAGEMANAGER.Read_8Bits(COMPASS_TYPE_ADDR) >= 4)
    MagOrientation = EXTERNAL_COMPASS; //COMPASS EXTERNO

  if (Compass_Type == COMPASS_AK8975)
  {
    AVRTIME.SchedulerSleep(100);
    I2C.WriteRegister(0x0C, 0x0A, 0x01);
    AVRTIME.SchedulerSleep(100);
  }

  if (Compass_Type == COMPASS_HMC5843)
  {
    AVRTIME.SchedulerSleep(100);
    I2C.WriteRegister(MagAddress, 0x00, 0x71);
    AVRTIME.SchedulerSleep(50);
    I2C.WriteRegister(MagAddress, 0x01, 0x60);
    I2C.WriteRegister(MagAddress, 0x02, 0x01);
    AVRTIME.SchedulerSleep(100);
    InitialReadBufferData();
    AVRTIME.SchedulerSleep(10);
    MagnetometerGain[ROLL] = 1000.0 / ABS_16BITS(IMU.CompassRead[ROLL]);
    MagnetometerGain[PITCH] = 1000.0 / ABS_16BITS(IMU.CompassRead[PITCH]);
    MagnetometerGain[YAW] = 1000.0 / ABS_16BITS(IMU.CompassRead[YAW]);
    I2C.WriteRegister(MagAddress, 0x00, 0x70);
    I2C.WriteRegister(MagAddress, 0x01, 0x20);
    I2C.WriteRegister(MagAddress, 0x02, 0x00);
  }

  if (Compass_Type == COMPASS_HMC5883)
  {
    if (FakeHMC5883Address == 0x0D)
    {
      I2C.WriteRegister(MagAddress, 0x0B, 0x01);
      I2C.WriteRegister(MagAddress, 0x09, 0xC1);
    }
    else
    {
      bool BiasOk = true;
      I2C.WriteRegister(MagAddress, 1, 40);
      I2C.WriteRegister(MagAddress, 2, 1);
      AVRTIME.SchedulerSleep(100);
      InitialReadBufferData();
      if (!PushBias(0x011))
        BiasOk = false;
      if (!PushBias(0x012))
        BiasOk = false;
      if (BiasOk)
      {
        //CALCULA O GANHO PARA CADA EIXO DO COMPASS
        MagnetometerGain[ROLL] = 19024.00 / XYZ_CompassBias[ROLL];
        MagnetometerGain[PITCH] = 19024.00 / XYZ_CompassBias[PITCH];
        MagnetometerGain[YAW] = 19024.00 / XYZ_CompassBias[YAW];
      }
      I2C.WriteRegister(MagAddress, 0, 0x70);
      I2C.WriteRegister(MagAddress, 1, 0x20);
      I2C.WriteRegister(MagAddress, 2, 0x00);
      AVRTIME.SchedulerSleep(100);
    }
  }
}

bool CompassReadClass::PushBias(uint8_t InputBias)
{
  int16_t ABS_MagRead;
  I2C.WriteRegister(MagAddress, 0, InputBias);
  for (uint8_t GetSamplesOfMag = 0; GetSamplesOfMag < 10; GetSamplesOfMag++) //RECOLHE 10 AMOSTRAS
  {
    I2C.WriteRegister(MagAddress, 2, 1);
    AVRTIME.SchedulerSleep(100);
    InitialReadBufferData();
    //VERIFICA SE NENHUMA LEITURA DO MAG IRÁ EXCEDER O LIMITE DE 2^12
    //ROLL
    ABS_MagRead = ABS_16BITS(IMU.CompassRead[ROLL]);
    XYZ_CompassBias[ROLL] += ABS_MagRead;
    if (ABS_MagRead > 4096)
      return false;
    //PITCH
    ABS_MagRead = ABS_16BITS(IMU.CompassRead[PITCH]);
    XYZ_CompassBias[PITCH] += ABS_MagRead;
    if (ABS_MagRead > 4096)
      return false;
    //YAW
    ABS_MagRead = ABS_16BITS(IMU.CompassRead[YAW]);
    XYZ_CompassBias[YAW] += ABS_MagRead;
    if (ABS_MagRead > 4096)
      return false;
  }
  return true; //OK,NENHUMA LEITURA DO MAG EXCEDEU 2^12
}

void CompassReadClass::InitialReadBufferData()
{
  if ((Compass_Type == COMPASS_HMC5843) || (Compass_Type == COMPASS_HMC5883))
  {
    I2C.SensorsRead(MagAddress, 0x03);
    SetOrientation(MagOrientation, Compass_Type);
  }
}

void CompassReadClass::ReadBufferData()
{
  if (Compass_Type == COMPASS_AK8975)
  {
    I2C.SensorsRead(0x0C, 0x03);
    SetOrientation(MagOrientation, Compass_Type);
    I2C.WriteRegister(0x0C, 0x0A, 0x01);
  }
  else if ((Compass_Type == COMPASS_HMC5843) || (Compass_Type == COMPASS_HMC5883))
  {
    if ((COMPASS.FakeHMC5883Address != 0x0D) && (Compass_Type == COMPASS_HMC5883))
    {
      I2C.SensorsRead(0x68, 0x49);
    }
    else
    {
      I2C.SensorsRead(MagAddress, MagRegister);
    }
    SetOrientation(MagOrientation, Compass_Type);
  }
}

void CompassReadClass::Constant_Read()
{
  //SAIA DA FUNÇÃO SE NÃO FOR ENCONTRADO NENHUM COMPASS NO BARRAMENTO I2C
  if (!I2C.CompassFound)
  {
    return;
  }

  CompassTimer = AVRTIME.SchedulerMicros();

  //SAIA DA FUNÇÃO SE O TEMPO DE ATUALIZAÇÃO NÃO FOR VALIDO
  if (!CalibratingCompass && CompassTimer < NextUpdate)
  {
    return;
  }

  //CALCULA O PROXIMO VALOR DE ATUALIZAÇÃO DO COMPASS
  NextUpdate = CompassTimer + COMPASS_UPDATE_FREQUENCY;

  //REALIZA A LEITURA I2C DO COMPASS
  ReadBufferData();

  //APLICA O GANHO CALCULADO
  IMU.CompassRead[ROLL] = IMU.CompassRead[ROLL] * MagnetometerGain[ROLL];
  //APLICA LPF NO ROLL PARA EVITAR SPIKES DURANTE A CALIBRAÇÃO DO COMPASS
  if (!COMMAND_ARM_DISARM)
    MagnetometerRead[ROLL] = MagnetometerRead[ROLL] * 0.9f + IMU.CompassRead[ROLL] * 0.1f;
  //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
  if (!CalibratingCompass)
    IMU.CompassRead[ROLL] -= CALIBRATION.Magnetometer[ROLL];

  //APLICA O GANHO CALCULADO
  IMU.CompassRead[PITCH] = IMU.CompassRead[PITCH] * MagnetometerGain[PITCH];
  //APLICA LPF NO PITCH PARA EVITAR SPIKES DURANTE A CALIBRAÇÃO DO COMPASS
  if (!COMMAND_ARM_DISARM)
    MagnetometerRead[PITCH] = MagnetometerRead[PITCH] * 0.9f + IMU.CompassRead[PITCH] * 0.1f;
  //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
  if (!CalibratingCompass)
    IMU.CompassRead[PITCH] -= CALIBRATION.Magnetometer[PITCH];

  //APLICA O GANHO CALCULADO
  IMU.CompassRead[YAW] = IMU.CompassRead[YAW] * MagnetometerGain[YAW];
  //APLICA LPF NO YAW PARA EVITAR SPIKES DURANTE A CALIBRAÇÃO DO COMPASS
  if (!COMMAND_ARM_DISARM)
    MagnetometerRead[YAW] = MagnetometerRead[YAW] * 0.9f + IMU.CompassRead[YAW] * 0.1f;
  //AJUSTA O VALOR DO COMPASS COM A CALIBRAÇÃO GUARDADA NA EEPROM
  if (!CalibratingCompass)
    IMU.CompassRead[YAW] -= CALIBRATION.Magnetometer[YAW];

  if (CalibratingCompass)
  {
    if (CalibrationTime == 0)
      CalibrationTime = NextUpdate;
    if ((NextUpdate - CalibrationTime) < CALIBRATION_TIME * 1000000)
    {
      RGB.Function(MAGLED);
      if (CalibrationTime == NextUpdate)
      {
        MagCalibrationMinVector[ROLL] = MagnetometerRead[ROLL];
        MagCalibrationMaxVector[ROLL] = MagnetometerRead[ROLL];
        MagCalibrationMinVector[PITCH] = MagnetometerRead[PITCH];
        MagCalibrationMaxVector[PITCH] = MagnetometerRead[PITCH];
        MagCalibrationMinVector[YAW] = MagnetometerRead[YAW];
        MagCalibrationMaxVector[YAW] = MagnetometerRead[YAW];
      }
      if (((int16_t)MagnetometerRead[ROLL]) < MagCalibrationMinVector[ROLL])
        MagCalibrationMinVector[ROLL] = MagnetometerRead[ROLL];
      if (((int16_t)MagnetometerRead[PITCH]) < MagCalibrationMinVector[PITCH])
        MagCalibrationMinVector[PITCH] = MagnetometerRead[PITCH];
      if (((int16_t)MagnetometerRead[YAW]) < MagCalibrationMinVector[YAW])
        MagCalibrationMinVector[YAW] = MagnetometerRead[YAW];
      if (((int16_t)MagnetometerRead[ROLL]) > MagCalibrationMaxVector[ROLL])
        MagCalibrationMaxVector[ROLL] = MagnetometerRead[ROLL];
      if (((int16_t)MagnetometerRead[PITCH]) > MagCalibrationMaxVector[PITCH])
        MagCalibrationMaxVector[PITCH] = MagnetometerRead[PITCH];
      if (((int16_t)MagnetometerRead[YAW]) > MagCalibrationMaxVector[YAW])
        MagCalibrationMaxVector[YAW] = MagnetometerRead[YAW];
      CALIBRATION.Magnetometer[ROLL] = (MagCalibrationMinVector[ROLL] + MagCalibrationMaxVector[ROLL]) >> 1;
      CALIBRATION.Magnetometer[PITCH] = (MagCalibrationMinVector[PITCH] + MagCalibrationMaxVector[PITCH]) >> 1;
      CALIBRATION.Magnetometer[YAW] = (MagCalibrationMinVector[YAW] + MagCalibrationMaxVector[YAW]) >> 1;
    }
    else
    {
      CalibratingCompass = false;
      CalibrationTime = 0;
      STORAGEMANAGER.Write_16Bits(MAG_ROLL_ADDR, CALIBRATION.Magnetometer[ROLL]);
      STORAGEMANAGER.Write_16Bits(MAG_PITCH_ADDR, CALIBRATION.Magnetometer[PITCH]);
      STORAGEMANAGER.Write_16Bits(MAG_YAW_ADDR, CALIBRATION.Magnetometer[YAW]);
      CheckAndUpdateIMUCalibration();
      BEEPER.Play(BEEPER_CALIBRATION_DONE);
    }
  }
  COMPASS.Rotate();
}

void CompassReadClass::SetOrientation(uint8_t Orientation, uint8_t _CompassType)
{

  switch (Orientation)
  {

  case GPS_ONBOARD_COMPASS:
    //ORIENTAÇÃO PARA COMPASS ONBOARD DOS GPS M7 E M8
    if (_CompassType == COMPASS_AK8975)
    {
      //ORIENTAÇÃO PARA O COMPASS AK8975
      IMU.CompassRead[ROLL] = -((BufferData[1] << 8) | BufferData[0]);
      IMU.CompassRead[PITCH] = -((BufferData[3] << 8) | BufferData[2]);
      IMU.CompassRead[YAW] = ((BufferData[5] << 8) | BufferData[4]);
      return;
    }
    else if (_CompassType == COMPASS_HMC5843)
    {
      //ORIENTAÇÃO PARA O COMPASS HMC5843
      IMU.CompassRead[ROLL] = -((BufferData[0] << 8) | BufferData[1]);
      IMU.CompassRead[PITCH] = -((BufferData[2] << 8) | BufferData[3]);
      IMU.CompassRead[YAW] = ((BufferData[4] << 8) | BufferData[5]);
      return;
    }
    else if (_CompassType == COMPASS_HMC5883)
    {
      if (COMPASS.FakeHMC5883Address != 0x0D)
      {
        //ORIENTAÇÃO PARA O COMPASS HMC5883
        IMU.CompassRead[ROLL] = -((BufferData[0] << 8) | BufferData[1]);
        IMU.CompassRead[PITCH] = -((BufferData[4] << 8) | BufferData[5]);
        IMU.CompassRead[YAW] = ((BufferData[2] << 8) | BufferData[3]);
      }
      else
      {
        //ORIENTAÇÃO PARA O COMPASS QMC5883
        IMU.CompassRead[ROLL] = -((BufferData[1] << 8) | BufferData[0]);
        IMU.CompassRead[PITCH] = -((BufferData[3] << 8) | BufferData[2]);
        IMU.CompassRead[YAW] = ((BufferData[5] << 8) | BufferData[4]);
      }
      return;
    }
    break;

  case EXTERNAL_COMPASS:
    //ORIENTAÇÃO NORMAL PARA COMPASS EXTERNO
    if (_CompassType == COMPASS_AK8975)
    {
      //ORIENTAÇÃO PARA O COMPASS AK8975
      IMU.CompassRead[ROLL] = ((BufferData[1] << 8) | BufferData[0]);
      IMU.CompassRead[PITCH] = ((BufferData[3] << 8) | BufferData[2]);
      IMU.CompassRead[YAW] = -((BufferData[5] << 8) | BufferData[4]);
      return;
    }
    else if (_CompassType == COMPASS_HMC5843)
    {
      //ORIENTAÇÃO PARA O COMPASS HMC5843
      IMU.CompassRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]);
      IMU.CompassRead[PITCH] = ((BufferData[2] << 8) | BufferData[3]);
      IMU.CompassRead[YAW] = -((BufferData[4] << 8) | BufferData[5]);
      return;
    }
    else if (_CompassType == COMPASS_HMC5883)
    {
      if (COMPASS.FakeHMC5883Address != 0x0D)
      {
        //ORIENTAÇÃO PARA O COMPASS HMC5883
        IMU.CompassRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]);
        IMU.CompassRead[PITCH] = ((BufferData[4] << 8) | BufferData[5]);
        IMU.CompassRead[YAW] = -((BufferData[2] << 8) | BufferData[3]);
      }
      else
      {
        //ORIENTAÇÃO PARA O COMPASS QMC5883
        IMU.CompassRead[ROLL] = ((BufferData[1] << 8) | BufferData[0]);
        IMU.CompassRead[PITCH] = ((BufferData[3] << 8) | BufferData[2]);
        IMU.CompassRead[YAW] = -((BufferData[5] << 8) | BufferData[4]);
      }
      return;
    }
    break;
  }
}

#define HALF_SQRT_2 0.70710678118654757f
void CompassReadClass::Rotate()
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
