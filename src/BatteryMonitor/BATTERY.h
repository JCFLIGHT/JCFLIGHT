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

#ifndef BATTERY_H_
#define BATTERY_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern Battery_Struct Battery;
class BatteryClass
{
public:
  uint8_t GetPercentage(void);
  float Get_Current_In_Mah(void);
  float Get_Max_Voltage_Calced(void);
  void Initialization(void);
  void Update_Voltage(void);
  void Update_Current(void);
  float Get_Actual_Voltage(void);
  float Get_Actual_Current(void);
  void Calculate_Total_Current_In_Mah(void);
  uint32_t GetWatts(void);
  void Exhausted(void);

private:
  uint8_t CalculatePercentage(float BattVoltage, float BattMinVolt, float BattMaxVolt);
  float AutoBatteryMin(float BattVoltage);
  float AutoBatteryMax(float BattVoltage);
};
extern BatteryClass BATTERY;
#endif