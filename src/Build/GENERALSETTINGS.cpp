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

#include "GENERALSETTINGS.h"
#include "ParamsToGCS/CHECKSUM.h"
#include "GPSNavigation/NAVIGATION.h"
#include "IMU/ACCGYROREAD.h"
#include "PID/PIDXYZ.h"
#include "AirPlane/AIRPLANE.h"
#include "AltitudeHoldControl/ALTITUDEHOLD.h"
#include "IOMCU/IOMCU.h"
#include "FastSerial/UART2MODE.h"
#include "FlightModes/AUXFLIGHT.h"
#include "RadioControl/CURVESRC.h"
#include "PID/TPA.h"
#include "RadioControl/DECODE.h"
#include "PID/PIDPARAMS.h"
#include "Filters/KALMANFILTER.h"
#include "RadioControl/RCCONFIG.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "AirSpeed/AIRSPEED.h"
#include "AHRS/AHRS.h"
#include "SafetyButton/SAFETYBUTTON.h"
#include "Param/PARAM.h"
#include "RadioControl/RCSMOOTH.h"
#include "ServosMaster/SERVOSMASTER.h"
#include "BatteryMonitor/BATTERY.h"

void GeneralSettingsInitialization()
{
  PARAM.Initialization();
  AUXFLIGHT.Initialization();
  CurvesRC_SetValues();
  TPA_Initialization();
  CurvesRC_CalculeValue();
  DECODE.Initialization();
  UART2Mode_Initialization();
  CHECKSUM.UpdateChannelsReverse();
  RTH_Altitude_EEPROM();
  LoadPID();
  LoadGPSParameters();
  PIDXYZ.Initialization();
  KALMAN.Init();
  IMU_Filters_Initialization();
  RCCONFIG.Init();
  AltitudeHold_Update_Params();
  WayPoint_Initialization();
  AIRSPEED.Initialization();
  GCS.UpdateParametersToGCS();
  AHRS.Initialization();
  SAFETYBUTTON.Initialization();
  RCInterpolationInit();
  SERVOSMASTER.Initialization();
  BATTERY.Initialization();
}