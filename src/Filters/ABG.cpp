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

#include "ABG.h"
#include "Math/MATHSUPPORT.h"

/*
  AUTOR:QUICK-FLASH - EMUFLIGHT
  O PDF ESTÁ NA PASTA "DOCS" COM O NOME "ABGFilter.pdf"
  https://github.com/emuflight/EmuFlight/blob/master/src/main/common/filter.c
*/

static void PT1FilterInitialization(PT1_Filter_For_ABG_Struct *Filter_Pointer, float K)
{
  Filter_Pointer->State = 0.0f;
  Filter_Pointer->K = K;
}

static float PT1FilterCalculeGain(uint16_t CutOffFrequency, float DeltaTime)
{
  const float RC = 0.5f / (3.1415926535897932384626433832795f * CutOffFrequency);
  return DeltaTime / (RC + DeltaTime);
}

static float PT1FilterApply(PT1_Filter_For_ABG_Struct *Filter_Pointer, float Input)
{
  Filter_Pointer->State = Filter_Pointer->State + Filter_Pointer->K * (Input - Filter_Pointer->State);
  return Filter_Pointer->State;
}

void ABG_Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime)
{
  const float Alpha2 = Alpha * 0.001f;
  const float xi = Fast_Pow(-Alpha2 + 1.0f, 0.25);
  Filter_Pointer->xK = 0.0f;
  Filter_Pointer->vK = 0.0f;
  Filter_Pointer->aK = 0.0f;
  Filter_Pointer->jK = 0.0f;
  Filter_Pointer->A = Alpha2;
  Filter_Pointer->B = (1.0f / 6.0f) * Fast_Pow(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
  Filter_Pointer->G = 2 * Fast_Pow(1.0f - xi, 3) * (1 + xi);
  Filter_Pointer->E = (1.0f / 6.0f) * Fast_Pow(1 - xi, 4);
  Filter_Pointer->DeltaTime = DeltaTime;
  Filter_Pointer->DeltaTime2 = DeltaTime * DeltaTime;
  Filter_Pointer->DeltaTime3 = DeltaTime * DeltaTime * DeltaTime;
  Filter_Pointer->Boost = (BoostGain * BoostGain / 1000000) * 0.003;
  Filter_Pointer->HalfLife = HalfLife != 0 ? Fast_Pow(0.5f, DeltaTime / HalfLife / 100.0f) : 1.0f;

  PT1FilterInitialization(&Filter_Pointer->BoostFilter, PT1FilterCalculeGain(100, DeltaTime));
  PT1FilterInitialization(&Filter_Pointer->VelocityFilter, PT1FilterCalculeGain(75, DeltaTime));
  PT1FilterInitialization(&Filter_Pointer->AcelerationFilter, PT1FilterCalculeGain(50, DeltaTime));
  PT1FilterInitialization(&Filter_Pointer->JerkFilter, PT1FilterCalculeGain(25, DeltaTime));
}

float AlphaBetaGammaApply(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Input)
{
  float rK; //ERRO RESIDUAL

  Filter_Pointer->xK *= Filter_Pointer->HalfLife;
  Filter_Pointer->vK *= Filter_Pointer->HalfLife;
  Filter_Pointer->aK *= Filter_Pointer->HalfLife;
  Filter_Pointer->jK *= Filter_Pointer->HalfLife;

  //ATUALZIA O ESTADO ESTIMADO DO SISTEMA
  Filter_Pointer->xK += Filter_Pointer->DeltaTime * Filter_Pointer->vK + (1.0f / 2.0f) * Filter_Pointer->DeltaTime2 * Filter_Pointer->aK + (1.0f / 6.0f) * Filter_Pointer->DeltaTime3 * Filter_Pointer->jK;

  //ATUALIZA A VELOCIDADE ESTIMADA
  Filter_Pointer->vK += Filter_Pointer->DeltaTime * Filter_Pointer->aK + 0.5f * Filter_Pointer->DeltaTime2 * Filter_Pointer->jK;
  Filter_Pointer->aK += Filter_Pointer->DeltaTime * Filter_Pointer->jK;

  //CALCULA O ERRO RESIDUAL
  rK = Input - Filter_Pointer->xK;

  //AUMENTA ARTIFICIALMENTE O ERRO PARA AUMENTAR A RESPOSTA DO FILTRO
  rK += PT1FilterApply(&Filter_Pointer->BoostFilter, (ABS(rK) * rK * Filter_Pointer->Boost));

  //ATUALIZA A ESTIMATIVA DE ERRO RESIDUAL
  Filter_Pointer->xK += Filter_Pointer->A * rK;
  Filter_Pointer->vK += Filter_Pointer->B / Filter_Pointer->DeltaTime * rK;
  Filter_Pointer->aK += Filter_Pointer->G / (2.0f * Filter_Pointer->DeltaTime2) * rK;
  Filter_Pointer->jK += Filter_Pointer->E / (6.0f * Filter_Pointer->DeltaTime3) * rK;

  //FILTRA ALGUNS RECURSOS
  Filter_Pointer->vK = PT1FilterApply(&Filter_Pointer->VelocityFilter, Filter_Pointer->vK);
  Filter_Pointer->aK = PT1FilterApply(&Filter_Pointer->AcelerationFilter, Filter_Pointer->aK);
  Filter_Pointer->jK = PT1FilterApply(&Filter_Pointer->JerkFilter, Filter_Pointer->jK);

  return Filter_Pointer->xK;
}