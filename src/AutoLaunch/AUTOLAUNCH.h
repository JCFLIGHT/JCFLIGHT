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

#ifndef AUTOLAUNCH_H_
#define AUTOLAUNCH_H_
#include "Arduino.h"
class AutoLaunchClass
{
public:
  void Update();

private:
  const bool GetSwingVelocityState();
  void AutoLaunchDetector();
  void RCControllerThrottle_Apply_Logic(bool SlowThr);
  int16_t CalculeControllToPitch(float AngleInDegrees, int16_t InclinationMaxOfStabilize);
  void RCControllerYawPitchRoll_Apply_Logic(bool SlowControll);
  bool GetStateOfThrottle();
  bool GetValidStateToRunLaunch();
  bool GetIMUAngleBanked(float VectorPitch, bool CheckIMUInclination);
  bool AutoLaunchTimerOverFlow();
  bool AutoLaunchMaxAltitudeReached(void);
  bool AutoLaunchCompleted();
  void SetPlaneType();
  void ResetParameters();
};
extern AutoLaunchClass AUTOLAUNCH;
#endif
