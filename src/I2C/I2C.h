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

#ifndef I2C_H_
#define I2C_H_
#include "Build/LIBDEPENDENCIES.h"
#include "Common/STRUCTS.h"
extern I2C_Resources_Struct I2CResources;
class I2CPROTOCOL
{
public:
  uint8_t ReadACK(void);
  uint8_t ReadNAK(void);
  void Initialization(void);
  void All_Initialization(void);
  void Restart(uint8_t Address);
  void Write(uint8_t SendData);
  void Stop(void);
  void SensorsRead(uint8_t Address, uint8_t Register);
  void WriteRegister(uint8_t Address, uint8_t Register, uint8_t Value);
  void RegisterBuffer(uint8_t Address, uint8_t Register, uint8_t *Buffer, uint8_t Size);

private:
  void SearchDevicesInBarrament(void);
  uint8_t StartToSearchDevices(void);
  uint8_t SendHexadecimalValues(uint8_t Address);
};
extern I2CPROTOCOL I2C;
#endif
