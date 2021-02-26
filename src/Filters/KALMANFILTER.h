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

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_
#include "Build/LIBDEPENDENCIES.h"
typedef struct
{
  float Q; //RUIDO GERADO NO PROCESSO
  float R; //RUIDO DE MEDIÇÃO
  float X; //VALOR
  float P; //ERRO ESTIMADO
} StructKalmanState;
class KALMANCLASS
{
public:
  KALMANCLASS(){};
  void Init();
  void Apply_In_Acc(int16_t ApplyInAcc[3]);
  void Apply_In_Gyro(int16_t ApplyInGyro[3]);

private:
  void Accel();
  void Gyro();
  void Update(StructKalmanState *State, int16_t *ErrorInput);
  void State(StructKalmanState *State, float Q, float R, float P, float Initial_Value);
};
extern KALMANCLASS KALMAN;
#endif
