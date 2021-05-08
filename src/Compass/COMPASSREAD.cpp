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
#include "Common/ENUM.h"
#include "Common/STRUCTS.h"
#include "GPS/GPSSTATES.h"
#include "WHOAMI.h"
#include "PerformanceCalibration/PERFORMACC.h"
#include "PerformanceCalibration/PERFORMCOMPASS.h"
#include "IMU/ACCGYROREAD.h"
#include "FastSerial/PRINTF.h"

CompassReadClass COMPASS;

void CompassReadClass::Initialization(void)
{
  //USA ISSO POR ENQUANTO,POR QUE AS FUNÇÕES COMENTADAS ABAIXO NÃO FUNCIONAM CORRETAMENTE NO ATMEGA2560
  if (!I2CResources.Found.Compass)
  {
    return;
  }

  /*
  if (GetAK8975DeviceDetected())
  {
    IMU.Compass.Type = COMPASS_AK8975;
    I2CResources.Found.Compass = true;
    LOG("DISPOSITIVO ENCONTRADO - 0x0C < AK8975");
    LINE_SPACE;
  }
  else if (GetHMC5883DeviceDetected())
  {
    IMU.Compass.Type = COMPASS_HMC5883;
    I2CResources.Found.Compass = true;
    LOG("DISPOSITIVO ENCONTRADO - 0x1E << HMC5883");
    LINE_SPACE;
  }
  else if (GetQMC5883DeviceDetected())
  {
    IMU.Compass.Type = COMPASS_QMC5883;
    I2CResources.Found.Compass = true;
    LOG("DISPOSITIVO ENCONTRADO - 0x0D << QMC5883");
    LINE_SPACE;
  }
  */

  if (IMU.Compass.Type == COMPASS_AK8975)
  {
    I2C.WriteRegister(ADDRESS_COMPASS_AK8975, 0x0A, 0x01);
    SCHEDULERTIME.Sleep(10);
  }

  if (IMU.Compass.Type == COMPASS_HMC5883)
  {
    I2C.WriteRegister(ADDRESS_COMPASS_HMC5883, 0x00, 0x78);
    SCHEDULERTIME.Sleep(5);

    I2C.WriteRegister(ADDRESS_COMPASS_HMC5883, 0x01, 0x20);
    SCHEDULERTIME.Sleep(5);

    I2C.WriteRegister(ADDRESS_COMPASS_HMC5883, 0x02, 0x00);
    SCHEDULERTIME.Sleep(100);
  }

  if (IMU.Compass.Type == COMPASS_QMC5883)
  {
    I2C.WriteRegister(ADDRESS_COMPASS_QMC5883, 0x0B, 0x01);
    I2C.WriteRegister(ADDRESS_COMPASS_QMC5883, 0x09, 0x1D);
  }

  if (Get_GPS_Type(GPS_DJI_NAZA))
  {
    IMU.Compass.Type = COMPASS_DJI_NAZA;
    I2CResources.Found.Compass = true;
  }

  COMPASSCALIBRATION.Initialization();
}

void CompassReadClass::ReadBufferData(void)
{
  switch (IMU.Compass.Type)
  {

  case COMPASS_AK8975:
    I2C.RegisterBuffer(ADDRESS_COMPASS_AK8975, 0x03, I2CResources.Buffer.Data, 0x06);
    I2C.WriteRegister(ADDRESS_COMPASS_AK8975, 0x0A, 0x01);
    break;

  case COMPASS_HMC5883:
    I2C.RegisterBuffer(ADDRESS_COMPASS_HMC5883, 0x03, I2CResources.Buffer.Data, 0x06);
    break;

  case COMPASS_QMC5883:
    I2C.RegisterBuffer(ADDRESS_COMPASS_QMC5883, 0x00, I2CResources.Buffer.Data, 0x06);
    break;
  }
}

void CompassReadClass::Constant_Read(void)
{
  //SAIA DA FUNÇÃO SE NÃO FOR ENCONTRADO NENHUM COMPASS NO BARRAMENTO I2C
  if (!I2CResources.Found.Compass)
  {
    return;
  }

  //REALIZA A LEITURA I2C DO COMPASS
  COMPASS.ReadBufferData();

  //ROTACIONA OS EIXOS DO COMPASS DE ACORDO COM A CONFIGURAÇÃO DO GCS
  COMPASSORIENTATION.SetOrientation(IMU.Compass.Type);

  //CORRE A CALIBRAÇÃO DO COMPASS
  COMPASSCALIBRATION.Update();

  //APLICA A CALIBRAÇÃO DO COMPASS
  COMPASSCALIBRATION.Apply();

  //APLICA A ROTAÇÃO DO COMPASS
  COMPASSROTATION.Rotate();
}