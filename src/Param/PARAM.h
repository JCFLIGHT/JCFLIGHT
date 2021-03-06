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
extern JCF_Param_Adjustable_Struct JCF_Param;
class ParamClass
{
public:
  void Initialization(void);
  void Default_List(void);
  void Update(void);

private:
  bool PrintMessage = false;
  char SerialBuffer[48];
  uint32_t SerialBufferIndex = 0;
  void Load_Sketch(void);
  void Process_Command(char *TerminalCommandLine);
};
extern ParamClass PARAM;
#endif