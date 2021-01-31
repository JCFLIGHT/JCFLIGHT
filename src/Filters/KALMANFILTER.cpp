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

#include "KALMANFILTER.h"
#include "Build/GCC.h"

FILE_COMPILE_FOR_SPEED

//CALCULO DE FILTRAGEM DO KALMAN
//x_k = Ax_{k-1} + Bu_k + w_k
//z_k = Hx_k + v_k

//CALCULO DE ATUALIZAÇÃO DE TEMPO DO KALMAN
//x_k = Ax_{k - 1} + Uu_k
//P_k = AP_{k - 1}A^T + Q

//CALCULO DE ATUALIZAÇÃO DE MEDIÇÃO DO KALMAN
//K_k = P_k H^T(HP_kH^T + R)^T
//x_k = x_k + K_k(z_k - Hx_k)
//P_k = (I - K_kH)P_k

KALMANCLASS KALMAN;

//3 EIXOS ACELEROMETRO
StructKalmanState Kalman_Acc_X;
StructKalmanState Kalman_Acc_Y;
StructKalmanState Kalman_Acc_Z;

//3 EIXOS GYROSCOPIO
StructKalmanState Kalman_Gyro_X;
StructKalmanState Kalman_Gyro_Y;
StructKalmanState Kalman_Gyro_Z;

void KALMANCLASS::State(StructKalmanState *State, float Q, float R, float P, float Initial_Value)
{
  State->Q = Q;
  State->R = R;
  State->P = P;
  State->X = Initial_Value;
}

void KALMANCLASS::Update(StructKalmanState *State, int16_t *ErrorInput)
{
  float K;
  float Measurement = *ErrorInput;
  State->P = State->P + State->Q;
  //ATUALIZA A MEDIÇÃO
  K = State->P * (1.0 / (State->P + State->R));
  State->X = State->X + K * (Measurement - State->X);
  State->P = (1 - K) * State->P;
  Measurement = State->X;
  *ErrorInput = (int16_t)Measurement;
}

void KALMANCLASS::Accel()
{
#define PROCESSNOISE 0.0625 //RUIDO DO PROCESSO
#define MEDICIONNOISE 1.0   //RUIDO DE MEDIÇÃO
#define ESTIMATEERROR 0.22  //ERRO ESTIMADO

  KALMAN.State(&Kalman_Acc_X, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);
  KALMAN.State(&Kalman_Acc_Y, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);
  KALMAN.State(&Kalman_Acc_Z, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);

#undef PROCESSNOISE
#undef MEDICIONNOISE
#undef ESTIMATEERROR
}

void KALMANCLASS::Gyro()
{
#define PROCESSNOISE 3.0    //RUIDO DO PROCESSO
#define MEDICIONNOISE 0.125 //RUIDO DE MEDIÇÃO
#define ESTIMATEERROR 0.42  //ERRO ESTIMADO

  KALMAN.State(&Kalman_Gyro_X, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);
  KALMAN.State(&Kalman_Gyro_Y, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);
  KALMAN.State(&Kalman_Gyro_Z, PROCESSNOISE, MEDICIONNOISE, ESTIMATEERROR, 0);

#undef PROCESSNOISE
#undef MEDICIONNOISE
#undef ESTIMATEERROR
}

//APLICA O FILTRO KALMAN NO ACELEROMETRO
void KALMANCLASS::Apply_In_Acc(int16_t ApplyInAcc[3])
{
  KALMAN.Update(&Kalman_Acc_X, &ApplyInAcc[0]);
  KALMAN.Update(&Kalman_Acc_Y, &ApplyInAcc[1]);
  KALMAN.Update(&Kalman_Acc_Z, &ApplyInAcc[2]);
}

//APLICA O FILTRO KALMAN NO GYROSCOPIO
void KALMANCLASS::Apply_In_Gyro(int16_t ApplyInGyro[3])
{
  KALMAN.Update(&Kalman_Gyro_X, &ApplyInGyro[0]);
  KALMAN.Update(&Kalman_Gyro_Y, &ApplyInGyro[1]);
  KALMAN.Update(&Kalman_Gyro_Z, &ApplyInGyro[2]);
}

//INICIA O FILTRO RESPECTIVO DE CADA SENSOR
void KALMANCLASS::Init()
{
  KALMAN.Accel();
  KALMAN.Gyro();
}
