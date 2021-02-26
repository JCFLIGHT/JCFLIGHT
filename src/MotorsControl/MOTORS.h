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

#ifndef MOTORS_H_
#define MOTORS_H_
#include "Build/LIBDEPENDENCIES.h"
extern int16_t MotorControl[8];
void ConfigureRegisters(bool Run_Calibrate_ESC);
void ApplyMixingForMotorsAndServos(float DeltaTime);
void PulseInAllMotors(int16_t Pulse);
void ShutDownAllMotorsAndServos();
void ApplyPWMControlForMotorsAndServos();
#endif
