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

#include "VARPARAM.h"
#include "Scheduler/SCHEDULERTIME.h"
#ifdef __AVR_ATmega2560__
#include <avr/eeprom.h>
#endif

VarParam *VarParam::_Variables;
VarParam *VarParam::_Grouped_Variables;
uint16_t VarParam::_Tail_Sentinel;

//VARIAVEIS ÚNICAS
VarParam::VarParam(Var_Key Param_Key, const char *Name, Flags _Flags) : _Group(NULL),
                                                                        _Key(Param_Key | Param_Key_Not_Located),
                                                                        _Name(Name),
                                                                        _Flags(_Flags)
{
    if (!Has_Flags(Param_Flag_Unlisted))
    {
        _Link = _Variables;
        _Variables = this;
    }
}

//GRUPO DE VARIAVEIS
VarParam::VarParam(VarParam_Group *Param_Group, Var_Key Index, const char *Name, Flags _Flags) : _Group(Param_Group),
                                                                                                 _Key(Index),
                                                                                                 _Name(Name),
                                                                                                 _Flags(_Flags)
{
    VarParam **Var_Param;
    Var_Param = &_Grouped_Variables;
    size_t LoopCount = 0;

    while (*Var_Param != NULL)
    {
        if (LoopCount++ > Param_Num_Max)
        {
            return;
        }

        if ((*Var_Param)->_Key >= _Key)
        {
            break;
        }
        Var_Param = &((*Var_Param)->_Link);
    }
    _Link = *Var_Param;
    *Var_Param = this;
}

VarParam::~VarParam(void)
{
    VarParam **Var_Param;

    if (_Group)
    {
        Var_Param = &_Grouped_Variables;
    }
    else
    {
        Var_Param = &_Variables;
    }

    size_t LoopCount = 0;

    while (*Var_Param)
    {

        if (LoopCount++ > Param_Num_Max)
        {
            return;
        }

        if (*Var_Param == this)
        {
            *Var_Param = _Link;
            break;
        }

        Var_Param = &((*Var_Param)->_Link);
    }

    if (Has_Flags(Param_Flag_Is_Group))
    {
        Var_Param = &_Grouped_Variables;
        size_t LoopCount = 0;

        while (*Var_Param)
        {

            if (LoopCount++ > Param_Num_Max)
            {
                return;
            }

            if ((*Var_Param)->_Group == this)
            {
                *Var_Param = (*Var_Param)->_Link;
                continue;
            }
            Var_Param = &((*Var_Param)->_Link);
        }
    }
}

MetaClass::MetaClass(void)
{
}

MetaClass::~MetaClass()
{
}

size_t MetaClass::Serialize(void *Buffer, size_t BufferSize) const
{
    return 0;
}

size_t MetaClass::Unserialize(void *Buffer, size_t BufferSize)
{
    return 0;
}

void VarParam::Copy_Name(char *Buffer, size_t Buffer_Size) const
{
    Buffer[0] = '\0';
    if (_Name)
    {
        if (_Group)
        {
            _Group->Copy_Name(Buffer, Buffer_Size);
        }
        strlcat_P(Buffer, _Name, Buffer_Size);
    }
}

VarParam *VarParam::Find(const char *Name)
{
    VarParam *Var_Param;
    size_t LoopCount = 0;

    for (Var_Param = First(); Var_Param; Var_Param = Var_Param->Next())
    {

        if (LoopCount++ > Param_Num_Max)
        {
            return NULL;
        }

        char Name_Buffer[32];

        Var_Param->Copy_Name(Name_Buffer, sizeof(Name_Buffer));

        if (!strcmp(Name, Name_Buffer))
        {
            return Var_Param;
        }
    }
    return NULL;
}

VarParam *VarParam::Find(Var_Key Key)
{
    VarParam *Var_Param;
    size_t LoopCount = 0;

    for (Var_Param = First(); Var_Param; Var_Param = Var_Param->Next())
    {
        if (LoopCount++ > Param_Num_Max)
            return NULL;
        if (Key == Var_Param->Key())
        {
            return Var_Param;
        }
    }
    return NULL;
}

bool VarParam::Save(void)
{
    uint8_t VectorBuffer[Param_Size_Max];
    size_t Size;

    if (_Group)
    {
        return _Group->Save();
    }

    if (!EEPROM_Locate(true))
    {
        return false;
    }

    Size = Serialize(VectorBuffer, sizeof(VectorBuffer));

    if (Size == 0)
    {
        return false;
    }

    if (Size <= sizeof(VectorBuffer))
    {
        uint8_t *EEPROM_AddressCount = (uint8_t *)_Key;
        for (size_t Index = 0; Index < Size; Index++, EEPROM_AddressCount++)
        {
            uint8_t NewVectorBuffer;

#ifdef __AVR_ATmega2560__
            if (eeprom_read_byte(EEPROM_AddressCount) != VectorBuffer[Index])
            {
                eeprom_write_byte(EEPROM_AddressCount, VectorBuffer[Index]);
            }

            NewVectorBuffer = eeprom_read_byte(EEPROM_AddressCount);
#endif

            if (NewVectorBuffer != VectorBuffer[Index])
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

bool VarParam::Save_All(void)
{
    bool Result = true;
    VarParam *Var_Param = _Variables;
    size_t LoopCount = 0;

    while (Var_Param)
    {
        if (LoopCount++ > Param_Num_Max)
        {
            return false;
        }

        if (!Var_Param->Has_Flags(Param_Flag_No_Auto_Load) && (Var_Param->_Key != Param_Key_None))
        {
            if (!Var_Param->Save())
            {
                Result = false;
            }
        }
        Var_Param = Var_Param->_Link;
    }
    return Result;
}

bool VarParam::Load(void)
{
    uint8_t VectorBuffer[Param_Size_Max];
    size_t Size;

    if (_Group)
    {
        return _Group->Load();
    }

    if (!EEPROM_Locate(false))
    {
        return false;
    }

    Size = Serialize(NULL, 0);

    if (Size == 0)
    {
        return false;
    }

    if (Size <= sizeof(VectorBuffer))
    {

#ifdef __AVR_ATmega2560__
        eeprom_read_block(VectorBuffer, (void *)_Key, Size);
#endif

        return Unserialize(VectorBuffer, Size);
    }
    return false;
}

bool VarParam::Load_All(void)
{
    bool Result = true;
    VarParam *Var_Param = _Variables;
    size_t LoopCount = 0;

    while (Var_Param)
    {
        if (LoopCount++ > Param_Num_Max)
        {
            return false;
        }

        if (!Var_Param->Has_Flags(Param_Flag_No_Auto_Load) && (Var_Param->_Key != Param_Key_None))
        {
            if (!Var_Param->Load())
            {
                Result = false;
            }
        }
        Var_Param = Var_Param->_Link;
    }
    return Result;
}

void VarParam::Erase_All()
{
    VarParam *Var_Param;
    Var_Param = _Variables;
    size_t LoopCount = 0;

    while (Var_Param)
    {
        if (LoopCount++ > Param_Num_Max)
        {
            return;
        }

        Var_Param->_Key = Var_Param->Key() | Param_Key_Not_Located;
        Var_Param = Var_Param->_Link;
    }

    for (uint16_t Index = 0; Index < Param_EEPROM_Size; Index++)
    {
        uint8_t *EEPROM_AddressCount = (uint8_t *)Param_EEPROM_Size;

#ifdef __AVR_ATmega2560__
        if (eeprom_read_byte(EEPROM_AddressCount) != 0)
        {
            eeprom_write_byte(EEPROM_AddressCount, 0);
        }
#endif
    }

    _Tail_Sentinel = 0;
}

VarParam::Var_Key VarParam::Key(void)
{
    Var_Header Var_Header;

    if (_Group)
    {
        return Param_Key_None;
    }

    if (_Key & Param_Key_Not_Located)
    {
        return _Key & Param_Key_Mask;
    }

#ifdef __AVR_ATmega2560__
    eeprom_read_block(&Var_Header, (void *)(_Key - sizeof(Var_Header)), sizeof(Var_Header));
#endif

    return Var_Header.Key;
}

VarParam *VarParam::Next(void)
{
    if (_Link)
    {
        return _Link;
    }

    if (!_Group)
    {
        return _Grouped_Variables;
    }

    return NULL;
}

VarParam *VarParam::First_Member(VarParam_Group *Group)
{
    VarParam **Var_Param;
    Var_Param = &_Grouped_Variables;
    size_t LoopCount = 0;

    while (*Var_Param)
    {

        if (LoopCount++ > Param_Num_Max)
        {
            return NULL;
        }

        if ((*Var_Param)->_Group == Group)
        {
            return *Var_Param;
        }
        Var_Param = &((*Var_Param)->_Link);
    }
    return NULL;
}

VarParam *VarParam::Next_Member()
{
    VarParam *Var_Param;
    Var_Param = _Link;
    size_t LoopCount = 0;

    while (Var_Param)
    {

        if (LoopCount++ > Param_Num_Max)
            return NULL;

        if (Var_Param->_Group == _Group)
        {
            return Var_Param;
        }
        Var_Param = Var_Param->_Link;
    }
    return NULL;
}

bool VarParam::EEPROM_Scan(void)
{
    struct EEPROM_Header EE_Header;
    struct Var_Header Var_Header;
    VarParam *Var_Param;
    uint16_t EEPROM_Address;

    _Tail_Sentinel = 0;
    EEPROM_Address = 0;

#ifdef __AVR_ATmega2560__
    eeprom_read_block(&EE_Header, (void *)EEPROM_Address, sizeof(EE_Header));
#endif

    if ((EE_Header.Magic != Param_EEPROM_Magic) || (EE_Header.Revision != Param_EEPROM_Revision))
    {
        return false;
    }

    EEPROM_Address = sizeof(EE_Header);

    size_t LoopCount = 0;

    while (EEPROM_Address < (Param_EEPROM_Size - sizeof(Var_Header) - 1))
    {

        if (LoopCount++ > Param_Num_Max)
        {
            return NULL;
        }

#ifdef __AVR_ATmega2560__
        eeprom_read_block(&Var_Header, (void *)EEPROM_Address, sizeof(Var_Header));
#endif

        if (Var_Header.Key == Param_Key_Sentinel)
        {
            break;
        }

        if (Param_EEPROM_Size <= (EEPROM_Address + sizeof(Var_Header) + Var_Header.Size + 1 + sizeof(Var_Header)))
        {

            return false;
        }

        Var_Param = _Variables;

        size_t LoopCount2 = 0;

        while (Var_Param)
        {
            if (LoopCount2++ > Param_Num_Max)
            {
                return false;
            }

            if (Var_Param->Key() == Var_Header.Key)
            {
                Var_Param->_Key = EEPROM_Address + sizeof(Var_Header);
                break;
            }

            Var_Param = Var_Param->_Link;
        }
        EEPROM_Address += sizeof(Var_Header) + Var_Header.Size + 1;
    }

    Var_Param = _Variables;

    size_t LoopCount3 = 0;

    while (Var_Param)
    {
        if (LoopCount3++ > Param_Num_Max)
            return false;
        if (Var_Param->_Key & Param_Key_Not_Located)
        {
            Var_Param->_Key |= Param_Key_Not_Allocated;
        }
        Var_Param = Var_Param->_Link;
    }

    _Tail_Sentinel = EEPROM_Address;
    return true;
}

bool VarParam::EEPROM_Locate(bool allocate)
{
    Var_Header Var_Header;
    Var_Key New_Location;
    size_t Size;

    if (_Group || (_Key == Param_Key_None))
    {
        return false;
    }

    if (!(_Key & Param_Key_Not_Located))
    {
        return true;
    }

    if (!(_Key & Param_Key_Not_Allocated))
    {
        EEPROM_Scan();

        if (!(_Key & Param_Key_Not_Located))
        {
            return true;
        }
    }

    if (!allocate)
    {
        return false;
    }

    Size = Serialize(NULL, 0);

    if ((Size == 0) || (Size > Param_Size_Max))
    {
        return false;
    }

    if ((_Tail_Sentinel + Size + sizeof(Var_Header) * 2) > Param_EEPROM_Size)
    {
        return false;
    }

    if (_Tail_Sentinel == 0)
    {
        uint8_t Pad_Size;

        EEPROM_Header EE_Header;

        EE_Header.Magic = Param_EEPROM_Magic;
        EE_Header.Revision = Param_EEPROM_Revision;
        EE_Header.Spare = 0;

#ifdef __AVR_ATmega2560__
        eeprom_write_block(&EE_Header, (void *)0, sizeof(EE_Header));
#endif

        _Tail_Sentinel = sizeof(EE_Header);

        Pad_Size = (((uint8_t)SCHEDULERTIME.GetMicros()) % Param_Size_Max) + 1;
        Var_Header.Key = Param_Key_Pad;
        Var_Header.Size = Pad_Size - 1;
        Var_Header.Spare = 0;

#ifdef __AVR_ATmega2560__
        eeprom_write_block(&Var_Header, (void *)_Tail_Sentinel, sizeof(Var_Header));
#endif

        _Tail_Sentinel += sizeof(Var_Header) + Pad_Size;
    }

    New_Location = _Tail_Sentinel;
    _Tail_Sentinel += sizeof(Var_Header) + Size;

    Var_Header.Key = Param_Key_Sentinel;
    Var_Header.Size = 0;
    Var_Header.Spare = 0;

#ifdef __AVR_ATmega2560__
    eeprom_write_block(&Var_Header, (void *)_Tail_Sentinel, sizeof(Var_Header));
#endif

    Var_Header.Key = Key();
    Var_Header.Size = Size - 1;

#ifdef __AVR_ATmega2560__
    eeprom_write_block(&Var_Header, (void *)New_Location, sizeof(Var_Header));
#endif

    _Key = New_Location + sizeof(Var_Header);
    return true;
}

size_t VarParam_Group::Serialize(void *Buffer, size_t Buffer_Size) const
{
    return const_cast<VarParam_Group *>(this)->Serialize_Unserialize(Buffer, Buffer_Size, true);
}

size_t VarParam_Group::Unserialize(void *Buffer, size_t Buffer_Size)
{
    return Serialize_Unserialize(Buffer, Buffer_Size, false);
}

size_t VarParam_Group::Serialize_Unserialize(void *Buffer, size_t Buffer_Size, bool Do_Serialize)
{
    VarParam *Var_Param;
    size_t Size, Total_Size;
    Var_Param = First_Member(this);
    Total_Size = 0;
    size_t LoopCount = 0;

    while (Var_Param)
    {

        if (LoopCount++ > Param_Num_Max)
        {
            return false;
        }

        if (Do_Serialize)
        {
            Size = Var_Param->Serialize(Buffer, Buffer_Size);
        }
        else
        {
            Size = Var_Param->Unserialize(Buffer, Buffer_Size);
        }

        if (Size == 0)
        {
            return 0;
        }

        Total_Size += Size;

        if (Size <= Buffer_Size)
        {
            Buffer_Size -= Size;
            Buffer = (void *)((uint8_t *)Buffer + Size);
        }

        Var_Param = Var_Param->Next_Member();
    }
    return Total_Size;
}
