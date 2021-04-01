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

#ifndef PARAM_H_
#define PARAM_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern Struct_JCF_Param_Adjustable JCF_Param;
class ParamClass
{
public:
  void Initialization(void);
  void SerialProcess(void);

private:
  bool PrintMessage = false;
  char SerialBuffer[48];
  uint8_t Actual_Format_Version = 10; //1.0
  uint32_t SerialBufferIndex = 0;
  void Load_Sketch(void);
  void Set_And_Save(char *TerminalCommandLine);
};
extern ParamClass PARAM;
#endif