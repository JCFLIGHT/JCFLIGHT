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

#include "WHOAMI.h"
#include "I2C/I2C.h"
#include "Scheduler/SCHEDULERTIME.h"

bool GetAK8975DeviceDetected(void)
{
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {
        SCHEDULERTIME.Sleep(30);
        uint8_t CompareByte = 0;
        I2C.RegisterBuffer(ADDRESS_COMPASS_AK8975, 0x00, &CompareByte, 0x01);
        if (CompareByte == 0x48)
        {
            return true;
        }
    }
    return false;
}

bool GetHMC5883DeviceDetected(void)
{
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {
        SCHEDULERTIME.Sleep(30);
        uint8_t CompareByte = 0;
        I2C.RegisterBuffer(ADDRESS_COMPASS_HMC5883, 0x0A, &CompareByte, 0x01);
        if (CompareByte == 0x48)
        {
            return true;
        }
    }
    return false;
}

bool GetQMC5883DeviceDetected(void)
{
    for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
    {
        I2C.WriteRegister(ADDRESS_COMPASS_QMC5883, 0x0A, 0x80);
        SCHEDULERTIME.Sleep(30);

        uint8_t CompareByte = 0;
        I2C.RegisterBuffer(ADDRESS_COMPASS_QMC5883, 0x0D, &CompareByte, 0x01);

        if (CompareByte == 0xFF)
        {
            I2C.RegisterBuffer(ADDRESS_COMPASS_QMC5883, 0x09, &CompareByte, 0x01);
            if (CompareByte == 0x00)
            {
                return true;
            }
        }
    }
    return false;
}