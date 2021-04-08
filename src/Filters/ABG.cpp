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

void ABG_Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime)
{
    const float Alpha2 = Alpha * 0.001f;
    const float xi = Fast_Pow(-Alpha2 + 1.0f, 0.25);
    Filter_Pointer->xK = 0.0f;
    Filter_Pointer->vK = 0.0f;
    Filter_Pointer->aK = 0.0f;
    Filter_Pointer->jK = 0.0f;
    Filter_Pointer->a = Alpha2;
    Filter_Pointer->b = (1.0f / 6.0f) * Fast_Pow(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
    Filter_Pointer->g = 2 * Fast_Pow(1.0f - xi, 3) * (1 + xi);
    Filter_Pointer->e = (1.0f / 6.0f) * Fast_Pow(1 - xi, 4);
    Filter_Pointer->DeltaTime = DeltaTime;
    Filter_Pointer->DeltaTime2 = DeltaTime * DeltaTime;
    Filter_Pointer->DeltaTime3 = DeltaTime * DeltaTime * DeltaTime;
    Filter_Pointer->Boost = (BoostGain * BoostGain / 1000000) * 0.003;
    Filter_Pointer->HalfLife = HalfLife != 0 ? Fast_Pow(0.5f, DeltaTime / HalfLife / 100.0f) : 1.0f;
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
    rK += (ABS(rK) * rK * Filter_Pointer->Boost);

    //ATUALIZA A ESTIMATIVA DE ERRO RESIDUAL
    Filter_Pointer->xK += Filter_Pointer->a * rK;
    Filter_Pointer->vK += Filter_Pointer->b / Filter_Pointer->DeltaTime * rK;
    Filter_Pointer->aK += Filter_Pointer->g / (2.0f * Filter_Pointer->DeltaTime2) * rK;
    Filter_Pointer->jK += Filter_Pointer->e / (6.0f * Filter_Pointer->DeltaTime3) * rK;

    return Filter_Pointer->xK;
}