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

#ifndef SAFETYBUTTON_H_
#define SAFETYBUTTON_h_
#include "Arduino.h"
class SAFETYBUTTONCLASS
{
public:
  bool SafeButtonEnabled();
  bool GetSafeStateToOutput();
  void Initialization();
  void UpdateRoutine();

private:
  enum class Led_Pattern : uint16_t
  {
    FMU_INIT_ARM = 0x0003,
    FMU_REFUSE_TO_ARM = 0x5555,
    FMU_SAFE_TO_ARM = 0xffff,
  };
  bool GetButtonInterval();
  bool GetButtonState();
  bool WaitToNextProcess = false;
  bool SafeStateToApplyPulse = false;
  uint8_t DetectRise = 0;
  uint8_t Blink_Counter = 0;
  uint32_t LastDebounceTime = 0;
  void UpdateLedStatus(enum Led_Pattern Instance);
  void SetStateToLed(bool State);
  void FlashButton();
};
extern SAFETYBUTTONCLASS SAFETYBUTTON;
#endif