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

#include "BITARRAY.h"
#include "Common/ENUM.h"

//ESSE BIT ARRAY ESTÁ EM uint8_t (BYTE -> 0 - 255)

#define BITARRAY_BIT_OP(Array, Bit, Operator) ((Array)[(Bit) / (sizeof((Array)[0]) * 8)] Operator(1 << ((Bit) % (sizeof((Array)[0]) * 8))))

typedef uint8_t BitArrayElement8Bits;

bool Once_Previous_State[SIZE_OF_FLIGHT_MODES];

uint8_t SetFlightMode[SIZE_OF_FLIGHT_MODES];

bool BitArrayGet(const BitArrayElement8Bits *Array, unsigned Bit)
{
    return BITARRAY_BIT_OP((BitArrayElement8Bits *)Array, Bit, &);
}

void BitArraySet(BitArrayElement8Bits *Array, unsigned Bit)
{
    BITARRAY_BIT_OP((BitArrayElement8Bits *)Array, Bit, |=);
}

void BitArrayClear(BitArrayElement8Bits *Array, unsigned Bit)
{
    BITARRAY_BIT_OP((BitArrayElement8Bits *)Array, Bit, &= ~);
}

bool IS_FLIGHT_MODE_ACTIVE(uint8_t FlightModeName)
{
    return BitArrayGet(SetFlightMode, FlightModeName);
}

bool IS_FLIGHT_MODE_ACTIVE_ONCE(uint8_t FlightModeName)
{
    const bool Once_Actual_State = IS_FLIGHT_MODE_ACTIVE(FlightModeName);

    if (!Once_Actual_State)
    {
        Once_Previous_State[FlightModeName] = false;
    }

    if (Once_Previous_State[FlightModeName] != Once_Actual_State && Once_Actual_State)
    {
        Once_Previous_State[FlightModeName] = Once_Actual_State;
        return true;
    }

    return false;
}

void RESET_THIS_FLIGHT_MODE_ONCE(uint8_t FlightModeName)
{
    Once_Previous_State[FlightModeName] = false;
}

void ENABLE_THIS_FLIGHT_MODE(uint8_t FlightModeName)
{
    BitArraySet(SetFlightMode, FlightModeName);
}

void DISABLE_THIS_FLIGHT_MODE(uint8_t FlightModeName)
{
    BitArrayClear(SetFlightMode, FlightModeName);
}

void ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(uint8_t FlightModeName, bool Dependency)
{
    if (Dependency)
    {
        BitArraySet(SetFlightMode, FlightModeName);
    }
    else
    {
        BitArrayClear(SetFlightMode, FlightModeName);
    }
}

bool IS_STATE_ACTIVE(uint8_t StateName)
{
    return BitArrayGet(SetFlightMode, StateName);
}

void ENABLE_THIS_STATE(uint8_t StateName)
{
    BitArraySet(SetFlightMode, StateName);
}

void DISABLE_THIS_STATE(uint8_t StateName)
{
    BitArrayClear(SetFlightMode, StateName);
}

void ENABLE_DISABLE_THIS_STATE_WITH_DEPENDENCY(uint8_t StateName, bool Dependency)
{
    if (Dependency)
    {
        BitArraySet(SetFlightMode, StateName);
    }
    else
    {
        BitArrayClear(SetFlightMode, StateName);
    }
}