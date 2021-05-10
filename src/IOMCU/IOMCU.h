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

#ifndef IOMCU_H_
#define IOMCU_H_
#include "Build/LIBDEPENDENCIES.h"
class GCSClass
{
public:
  bool ConfigFlight = false;
  bool CliMode = false;
  void Serial_Parse_Protocol(void);
  void LoadAllParameters(void);
  void Send_String_To_GCS(const char *String);
  void Default_All_Configs(void);

private:
  void Update_BiDirect_Protocol(uint8_t TaskOrderGCS);
  void First_Packet_Request_Parameters(void);
  void Second_Packet_Request_Parameters(void);
  void WayPoint_Request_Coordinates_Parameters(void);
  void WayPoint_Request_Misc_Parameters(void);
  void Save_Basic_Configuration(void);
  void Save_Radio_Control_Configuration(void);
  void Save_Medium_Configuration(void);
  void Default_Basic_Configuration(void);
  void Default_Medium_Configuration(void);
  void Default_RadioControl_Configuration(void);
};
extern GCSClass GCS;
#endif
