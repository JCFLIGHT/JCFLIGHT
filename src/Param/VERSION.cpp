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

#include "VERSION.h"
#include "VARPARAM.h"
#include "FastSerial/PRINTF.h"

class VersionClass
{
public:
    static const int16_t Actual_Format_Version = 10; //1.0
    JC_Int16 Format_Version;
    VersionClass() : Format_Version(Actual_Format_Version, Param_Format_Version, ProgmemString("SYS_VER"))
    {
    }
};

static VersionClass Version;

void CheckVersion()
{
    if (!Version.Format_Version.Load() || Version.Format_Version != Version.Actual_Format_Version)
    {
        PRINTF.SendToConsole(ProgmemString("Primeira linkagem,limpando a EEPROM...\n"));
        VarParam::Erase_All();
        Version.Format_Version.Set_And_Save(Version.Actual_Format_Version);
        PRINTF.SendToConsole(ProgmemString("Ok...Parametros reconfigurados\n"));
    }
    else
    {
        VarParam::Load_all();
    }
}