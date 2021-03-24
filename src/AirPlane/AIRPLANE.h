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

#ifndef AIRPLANE_H_
#define AIRPLANE_H_
#include "Build/LIBDEPENDENCIES.h"
#define MAX_SUPPORTED_SERVOS 4
#define SAVE_SERVO_MIDDLE(Address, Value) STORAGEMANAGER.Write_16Bits(Address, Value) //SALVA O PONTO MÉDIO DOS SERVOS NA EEPROM
#define GET_SERVO_MIN(Address) STORAGEMANAGER.Read_16Bits(Address)                    //OBTÉM O VALOR DO PULSO MINIMO DOS SERVOS
#define GET_SERVO_MIDDLE(Address) STORAGEMANAGER.Read_16Bits(Address)                 //OBTÉM O VALOR DO PULSO MÉDIO DOS SERVOS
#define GET_SERVO_MAX(Address) STORAGEMANAGER.Read_16Bits(Address)                    //OBTÉM O VALOR DO PULSO MAXIMO DOS SERVOS
#define GET_SERVO_DIRECTION(Bit) (((Bit) > 0) ? -1 : 1)                               //OBTÉM A DIREÇÃO DOS SERVOS
#define GET_SERVO_RATE(Address) STORAGEMANAGER.Read_16Bits(Address)                   //OBTÉM O VALOR DO RATE DOS SERVOS
class AirPlaneClass
{
public:
  void Mode_ConventionalPlane_Run();
  void Mode_FixedWing_Run();
  void Mode_PlaneVTail_Run();
};
extern AirPlaneClass AIR_PLANE;
#endif
