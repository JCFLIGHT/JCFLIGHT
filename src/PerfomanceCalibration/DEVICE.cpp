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

#include "DEVICE.h"
#include "Math/MATHSUPPORT.h"

void DevicePushValues(Device_Struct *Device, float Value)
{
    Device->MeasureCount++;
    if (Device->MeasureCount == 1)
    {
        Device->OldMeasure = Device->NewMeasure = Value;
        Device->OldValue = 0.0f;
    }
    else
    {
        Device->NewMeasure = Device->OldMeasure + (Value - Device->OldMeasure) / Device->MeasureCount;
        Device->NewValue = Device->OldValue + (Value - Device->OldMeasure) * (Value - Device->NewMeasure);
        Device->OldMeasure = Device->NewMeasure;
        Device->OldValue = Device->NewValue;
    }
}

void DeviceClear(Device_Struct *Device)
{
    Device->MeasureCount = 0;
}

float DeviceVariance(Device_Struct *Device)
{
    return ((Device->MeasureCount > 1) ? Device->NewValue / (Device->MeasureCount - 1) : 0.0f);
}

float DeviceStandardDeviation(Device_Struct *Device)
{
    return Fast_SquareRoot(DeviceVariance(Device));
}