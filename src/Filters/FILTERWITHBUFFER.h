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

#ifndef FILTERWITHBUFFER_H_
#define FILTERWITHBUFFER_H_

#include <inttypes.h>
#include "FILTER.h"

template <class T, uint8_t FILTER_SIZE>
class FilterWithBuffer : public Filter<T>
{
public:
  FilterWithBuffer();
  virtual T Apply(T Sample);
  virtual void Reset();

protected:
  T Samples[FILTER_SIZE];
  uint8_t Sample_Index;
};

typedef FilterWithBuffer<int16_t, 2> FilterWithBufferInt16_Size2;
typedef FilterWithBuffer<int16_t, 3> FilterWithBufferInt16_Size3;
typedef FilterWithBuffer<int16_t, 4> FilterWithBufferInt16_Size4;
typedef FilterWithBuffer<int16_t, 5> FilterWithBufferInt16_Size5;
typedef FilterWithBuffer<int16_t, 6> FilterWithBufferInt16_Size6;
typedef FilterWithBuffer<int16_t, 7> FilterWithBufferInt16_Size7;
typedef FilterWithBuffer<uint16_t, 2> FilterWithBufferUInt16_Size2;
typedef FilterWithBuffer<uint16_t, 3> FilterWithBufferUInt16_Size3;
typedef FilterWithBuffer<uint16_t, 4> FilterWithBufferUInt16_Size4;
typedef FilterWithBuffer<uint16_t, 5> FilterWithBufferUInt16_Size5;
typedef FilterWithBuffer<uint16_t, 6> FilterWithBufferUInt16_Size6;
typedef FilterWithBuffer<uint16_t, 7> FilterWithBufferUInt16_Size7;
typedef FilterWithBuffer<int32_t, 2> FilterWithBufferInt32_Size2;
typedef FilterWithBuffer<int32_t, 3> FilterWithBufferInt32_Size3;
typedef FilterWithBuffer<int32_t, 4> FilterWithBufferInt32_Size4;
typedef FilterWithBuffer<int32_t, 5> FilterWithBufferInt32_Size5;
typedef FilterWithBuffer<int32_t, 6> FilterWithBufferInt32_Size6;
typedef FilterWithBuffer<int32_t, 7> FilterWithBufferInt32_Size7;
typedef FilterWithBuffer<uint32_t, 2> FilterWithBufferUInt32_Size2;
typedef FilterWithBuffer<uint32_t, 3> FilterWithBufferUInt32_Size3;
typedef FilterWithBuffer<uint32_t, 4> FilterWithBufferUInt32_Size4;
typedef FilterWithBuffer<uint32_t, 5> FilterWithBufferUInt32_Size5;
typedef FilterWithBuffer<uint32_t, 6> FilterWithBufferUInt32_Size6;
typedef FilterWithBuffer<uint32_t, 7> FilterWithBufferUInt32_Size7;

template <class T, uint8_t FILTER_SIZE>
FilterWithBuffer<T, FILTER_SIZE>::FilterWithBuffer() : Filter<T>(), Sample_Index(0)
{
  Reset();
}

template <class T, uint8_t FILTER_SIZE>
void FilterWithBuffer<T, FILTER_SIZE>::Reset()
{
  Filter<T>::Reset();
  for (int8_t i = 0; i < FILTER_SIZE; i++)
  {
    Samples[i] = 0;
  }
  Sample_Index = 0;
}

template <class T, uint8_t FILTER_SIZE>
T FilterWithBuffer<T, FILTER_SIZE>::Apply(T Sample)
{
  Samples[Sample_Index++] = Sample;
  if (Sample_Index >= FILTER_SIZE)
  {
    Sample_Index = 0;
  }
  return Sample;
}
#endif
