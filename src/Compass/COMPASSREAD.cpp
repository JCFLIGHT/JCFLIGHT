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
#include "StorageManager/EEPROMSTORAGE.h"
#include "I2C/I2C.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Math/MATHSUPPORT.h"
#include "BAR/BAR.h"
#include "ROTATION.h"
#include "ORIENTATION.h"
#include "COMPASSCAL.h"
#include "COMPASSLPF.h"
#include "Common/ENUM.h"
#include "Common/STRUCTS.h"
#include "GPS/GPSSTATES.h"
#include "WHOAMI.h"

CompassReadClass COMPASS;

static int32_t XYZ_CompassBias[3] = {0, 0, 0};

void CompassReadClass::Initialization()
{
  //SAIA DA FUNÇÃO SE NÃO FOR ENCONTRADO NENHUM COMPASS NO BARRAMENTO I2C
  if (!I2C.CompassFound)
  {
    return;
  }

  Check_Whoami();

  if (COMPASS.Type == COMPASS_AK8975)
  {
    SCHEDULERTIME.Sleep(100);
    I2C.WriteRegister(ADDRESS_COMPASS_AK8975, 0x0A, 0x01);
    SCHEDULERTIME.Sleep(100);
  }

  if (COMPASS.Type == COMPASS_HMC5843)
  {
    SCHEDULERTIME.Sleep(100);
    I2C.WriteRegister(COMPASS.Address, 0x00, 0x71);
    SCHEDULERTIME.Sleep(50);
    I2C.WriteRegister(COMPASS.Address, 0x01, 0x60);
    I2C.WriteRegister(COMPASS.Address, 0x02, 0x01);
    SCHEDULERTIME.Sleep(100);
    COMPASS.InitialReadBufferData();
    SCHEDULERTIME.Sleep(10);
    COMPASS.MagnetometerGain[ROLL] = 1000.0 / ABS(IMU.CompassRead[ROLL]);
    COMPASS.MagnetometerGain[PITCH] = 1000.0 / ABS(IMU.CompassRead[PITCH]);
    COMPASS.MagnetometerGain[YAW] = 1000.0 / ABS(IMU.CompassRead[YAW]);
    I2C.WriteRegister(COMPASS.Address, 0x00, 0x70);
    I2C.WriteRegister(COMPASS.Address, 0x01, 0x20);
    I2C.WriteRegister(COMPASS.Address, 0x02, 0x00);
  }

  if (COMPASS.Type == COMPASS_HMC5883)
  {
    bool BiasOk = true;
    I2C.WriteRegister(COMPASS.Address, 1, 40);
    I2C.WriteRegister(COMPASS.Address, 2, 1);
    SCHEDULERTIME.Sleep(100);
    COMPASS.InitialReadBufferData();
    if (!COMPASS.PushBias(0x011))
    {
      BiasOk = false;
    }
    if (!COMPASS.PushBias(0x012))
    {
      BiasOk = false;
    }
    if (BiasOk)
    {
      //CALCULA O GANHO PARA CADA EIXO DO COMPASS
      COMPASS.MagnetometerGain[ROLL] = 19024.00 / XYZ_CompassBias[ROLL];
      COMPASS.MagnetometerGain[PITCH] = 19024.00 / XYZ_CompassBias[PITCH];
      COMPASS.MagnetometerGain[YAW] = 19024.00 / XYZ_CompassBias[YAW];
    }
    I2C.WriteRegister(COMPASS.Address, 0, 0x70);
    I2C.WriteRegister(COMPASS.Address, 1, 0x20);
    I2C.WriteRegister(COMPASS.Address, 2, 0x00);
    SCHEDULERTIME.Sleep(100);
  }

  if (COMPASS.Type == COMPASS_QMC5883)
  {
    I2C.WriteRegister(COMPASS.Address, 0x0B, 0x01);
    I2C.WriteRegister(COMPASS.Address, 0x09, 0xC1);
  }
}

bool CompassReadClass::PushBias(uint8_t InputBias)
{
  int16_t ABS_MagRead;
  I2C.WriteRegister(COMPASS.Address, 0, InputBias);
  for (uint8_t GetSamplesOfMag = 0; GetSamplesOfMag < 10; GetSamplesOfMag++) //RECOLHE 10 AMOSTRAS
  {
    I2C.WriteRegister(COMPASS.Address, 2, 1);
    SCHEDULERTIME.Sleep(100);
    COMPASS.InitialReadBufferData();
    //VERIFICA SE NENHUMA LEITURA DO MAG IRÁ EXCEDER O LIMITE DE 2^12
    //ROLL
    ABS_MagRead = ABS(IMU.CompassRead[ROLL]);
    XYZ_CompassBias[ROLL] += ABS_MagRead;
    if (ABS_MagRead > 4096)
    {
      return false;
    }
    //PITCH
    ABS_MagRead = ABS(IMU.CompassRead[PITCH]);
    XYZ_CompassBias[PITCH] += ABS_MagRead;
    if (ABS_MagRead > 4096)
    {
      return false;
    }
    //YAW
    ABS_MagRead = ABS(IMU.CompassRead[YAW]);
    XYZ_CompassBias[YAW] += ABS_MagRead;
    if (ABS_MagRead > 4096)
    {
      return false;
    }
  }
  return true; //OK,NENHUMA LEITURA DO MAG EXCEDEU 2^12
}

void CompassReadClass::InitialReadBufferData()
{
  if ((COMPASS.Type == COMPASS_HMC5843) || (COMPASS.Type == COMPASS_HMC5883))
  {
    I2C.SensorsRead(COMPASS.Address, 0x03);
    COMPASSORIENTATION.SetOrientation(COMPASS.Type);
    COMPASSROTATION.Rotate();
  }
}

void CompassReadClass::ReadBufferData()
{
  if (Get_GPS_Type(GPS_UBLOX))
  {
    if (COMPASS.Type == COMPASS_AK8975)
    {
      I2C.SensorsRead(ADDRESS_COMPASS_AK8975, 0x03);
      I2C.WriteRegister(ADDRESS_COMPASS_AK8975, 0x0A, 0x01);
    }
    else if ((COMPASS.Type == COMPASS_HMC5843) || (COMPASS.Type == COMPASS_HMC5883))
    {
      I2C.SensorsRead(ADDRESS_IMU_MPU6050, 0x49);
    }
    else if (COMPASS.Type == COMPASS_QMC5883)
    {
      I2C.SensorsRead(COMPASS.Address, COMPASS.Register);
    }
    COMPASSORIENTATION.SetOrientation(COMPASS.Type);
  }
  else if (Get_GPS_Type(GPS_DJI_NAZA))
  {
    COMPASSORIENTATION.SetOrientation(COMPASS_DJI_NAZA);
  }
}

void CompassReadClass::Constant_Read()
{
  //SAIA DA FUNÇÃO SE NÃO FOR ENCONTRADO NENHUM COMPASS NO BARRAMENTO I2C
  if (!I2C.CompassFound && Get_GPS_Type(GPS_UBLOX))
  {
    return;
  }

  //REALIZA A LEITURA I2C DO COMPASS
  COMPASS.ReadBufferData();

  //APLICA OS AJUSTES DE BIAS CALCULADOS
  if (Get_GPS_Type(GPS_UBLOX))
  {
    COMPASSCAL.ApplyGain();
  }

  //APLICA O LPF PARA REDUZIR SPIKES DURANTE A CALIBRAÇÃO
  COMPASSLPF.ApplyFilter();

  //APLICA A CALIBRAÇÃO DO COMPASS
  COMPASSCAL.ApplyCalibration();

  //CORRE A CALIBRAÇÃO DO COMPASS
  COMPASSCAL.RunningCalibration();

  //APLICA A ROTAÇÃO DO COMPASS
  COMPASSROTATION.Rotate();
}

void CompassReadClass::UpdateCompassCalibration()
{
  COMPASSCAL.UpdateCompassCalibration();
}
