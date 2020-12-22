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

#ifndef BATTLEVELS_H_
#define BATTLEVELS_H_
//PARAMETROS PARA AS CELULAS DA BATERIA
#define BATT_LOW_CELLS 3.6f
#define BATT_HIGH_CELLS 4.2f

//PARAMETROS DE TENSÃO PARA BATERIA DE 3 CELULAS
#define BATT_3S_LOW_VOLTAGE 3 * BATT_LOW_CELLS
#define BATT_3S_HIGH_VOLTAGE 3 * BATT_HIGH_CELLS
#define BATT_3S_SAFE_LOW_VOLTAGE 10.0f
#define BATT_3S_SAFE_HIGH_VOLTAGE 13.0f

//PARAMETROS DE TENSÃO PARA BATERIA DE 4 CELULAS
#define BATT_4S_LOW_VOLTAGE 4 * BATT_LOW_CELLS
#define BATT_4S_HIGH_VOLTAGE 4 * BATT_HIGH_CELLS
#define BATT_4S_SAFE_LOW_VOLTAGE 14.0f
#define BATT_4S_SAFE_HIGH_VOLTAGE 17.5f

//PARAMETROS DE TENSÃO PARA BATERIA DE 6 CELULAS
#define BATT_6S_LOW_VOLTAGE 6 * BATT_LOW_CELLS
#define BATT_6S_HIGH_VOLTAGE 6 * BATT_HIGH_CELLS
#define BATT_6S_SAFE_LOW_VOLTAGE 20.0f
#define BATT_6S_SAFE_HIGH_VOLTAGE 26.0f
#endif