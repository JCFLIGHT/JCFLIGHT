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

#ifndef VARPARAM_H_
#define VARPARAM_H_

#include "Build/LIBDEPENDENCIES.h"

enum //255 KEYS NO MAXIMO
{
    Param_Format_Version = 0,
    VarParam_Test,
    VarParam_Test2
};

class VarParam_Group;

class MetaClass
{
public:
    MetaClass(void);

    virtual ~MetaClass();

    virtual size_t Serialize(void *Buffer, size_t BufferSize) const;

    virtual size_t Unserialize(void *Buffer, size_t BufferSize);
};

class VarParam : public MetaClass
{
public:
    struct EEPROM_Header
    {
        uint16_t Magic;
        uint8_t Revision;
        uint8_t Spare;
    };

    static const uint16_t Param_EEPROM_Magic = 0x434A;
    static const uint16_t Param_EEPROM_Revision = 1;

    typedef uint16_t Var_Key;

    struct Var_Header
    {
        uint8_t Size : 6;
        uint8_t Spare : 2;
        uint8_t Key;
    };

    static const Var_Key Param_Key_None = 0xffff;

    static const Var_Key Param_Key_Not_Located = (Var_Key)1 << 15;

    static const Var_Key Param_Key_Not_Allocated = (Var_Key)1 << 14;

    static const Var_Key Param_Key_Sentinel = 0xff;

    static const Var_Key Param_Key_Pad = 0xfe;

    static const Var_Key Param_Key_Mask = (Var_Key)(~(Param_Key_Not_Located | Param_Key_Not_Allocated));

    static const size_t Param_Size_Max = 64;

    static const size_t Param_Num_Max = 255;

    typedef uint8_t Flags;
    static const Flags Param_Flags_None = 0;

    static const Flags Param_Flag_No_Auto_Load = (1 << 0);

    static const Flags Param_Flag_Is_Group = (1 << 2);

    static const Flags Param_Flag_Unlisted = (1 << 3);

    //VARIAVEIS ÚNICAS
    VarParam(Var_Key Key = Param_Key_None, const char *Name = NULL, Flags _Flags = Param_Flags_None);

    //GRUPO DE VARIAVEIS
    VarParam(VarParam_Group *Group, Var_Key Index, const char *Name, Flags _Flags = Param_Flags_None);

    ~VarParam(void);

    void Copy_Name(char *Buffer, size_t BufferSize) const;

    static VarParam *Find(const char *Name);

    static VarParam *Find(Var_Key Key);

    bool Save(void);

    bool Load(void);

    static bool Save_All(void);

    static bool Load_all(void);

    static void Erase_All(void);

    bool Has_Flags(Flags FlagValue) const
    {
        return (_Flags & FlagValue) == FlagValue;
    }

    VarParam_Group *Group(void)
    {
        return _Group;
    }

    static VarParam *First(void)
    {
        return _Variables;
    }

    VarParam *Next(void);

    static VarParam *First_Member(VarParam_Group *Group);

    VarParam *Next_Member();

    Var_Key Key(void);

private:
    VarParam_Group *_Group;
    VarParam *_Link;
    Var_Key _Key;
    const char *_Name;
    uint8_t _Flags;

    static VarParam *_Variables;
    static VarParam *_Grouped_Variables;

    static uint16_t _Tail_Sentinel;

    static const uint16_t Param_EEPROM_Size = 4096;

    static bool EEPROM_Scan(void);

    bool EEPROM_Locate(bool allocate);
};

class VarParam_Group : public VarParam
{
public:
    VarParam_Group(Var_Key With_Key = Param_Key_None, const char *Name = NULL, Flags _Flags = Param_Flags_None) : VarParam(With_Key, Name, _Flags | Param_Flag_Is_Group)
    {
    }

    virtual size_t Serialize(void *Buffer, size_t Buffer_Size) const;

    virtual size_t Unserialize(void *Buffer, size_t Buffer_Size);

private:
    size_t Serialize_Unserialize(void *Buffer, size_t Buffer_Size, bool Do_Serialize);
};

template <typename T>
class VarParamT : public VarParam
{
public:
    VarParamT<T>(const T Initial_Value = 0,
                 Var_Key With_Key = Param_Key_None,
                 const char *Name = NULL,
                 Flags _Flags = Param_Flags_None) : VarParam(With_Key, Name, _Flags),
                                                    _Value(Initial_Value)
    {
    }

    VarParamT<T>(VarParam_Group *With_Group,
                 Var_Key Index,
                 T Initial_Value,
                 const char *Name = NULL,
                 Flags _Flags = Param_Flags_None) : VarParam(With_Group, Index, Name, _Flags),
                                                    _Value(Initial_Value)
    {
    }

    virtual size_t Serialize(void *Buffer, size_t Size) const
    {
        if (Size >= sizeof(_Value))
        {
            *(T *)Buffer = _Value;
        }
        return sizeof(_Value);
    }

    virtual size_t Unserialize(void *Buffer, size_t Size)
    {
        if (Size >= sizeof(_Value))
        {
            _Value = *(T *)Buffer;
        }
        return sizeof(_Value);
    }

    T Get(void) const
    {
        return _Value;
    }

    void Set(T v)
    {
        _Value = v;
    }

    bool Set_And_Save(T v)
    {
        Set(v);
        return Save();
    }

    operator T &()
    {
        return _Value;
    }

    VarParamT<T> &operator=(VarParamT<T> &Value)
    {
        return Value;
    }

    VarParamT<T> &operator=(T Value)
    {
        _Value = Value;
        return *this;
    }

protected:
    T _Value;
};

template <typename T>
class VarParamS : public VarParam
{
public:
    VarParamS<T>(Var_Key With_Key = Param_Key_None,
                 const char *Name = NULL,
                 Flags _Flags = Param_Flags_None) : VarParam(With_Key, Name, _Flags)
    {
    }

    VarParamS<T>(VarParam_Group *With_Group,
                 Var_Key Index,
                 const char *Name = NULL,
                 Flags _Flags = Param_Flags_None) : VarParam(With_Group, Index, Name, _Flags)
    {
    }

    virtual size_t Serialize(void *Buffer, size_t Size) const
    {
        if (Size >= sizeof(_Value))
        {
            memcpy(Buffer, &_Value, sizeof(_Value));
        }
        return sizeof(_Value);
    }

    virtual size_t Unserialize(void *Buffer, size_t Size)
    {
        if (Size >= sizeof(_Value))
        {
            memcpy(&_Value, Buffer, sizeof(_Value));
        }
        return sizeof(_Value);
    }

    T &Get()
    {
        return _Value;
    }

    const T &Get() const
    {
        return _Value;
    }

    void Set(T v)
    {
        _Value = v;
    }

    void Set_And_Save(T v)
    {
        Set(v);
        Save();
    }

protected:
    T _Value;
};

template <typename T, uint8_t N>
class VarParamA : public VarParam
{
public:
    VarParamA<T, N>(Var_Key With_Key = Param_Key_None,
                    const char *Name = NULL,
                    Flags _Flags = Param_Flags_None) : VarParam(With_Key, Name, _Flags)
    {
    }

    VarParamA<T, N>(VarParam_Group *With_Group,
                    Var_Key Index,
                    const char *Name = NULL,
                    Flags _Flags = Param_Flags_None) : VarParam(With_Group, Index, Name, _Flags)
    {
    }

    virtual size_t Serialize(void *Buffer, size_t Size) const
    {
        if (Size >= sizeof(_Value))
        {
            memcpy(Buffer, &_Value[0], sizeof(_Value));
        }
        return sizeof(_Value);
    }

    virtual size_t Unserialize(void *Buffer, size_t Size)
    {
        if (Size >= sizeof(_Value))
        {
            memcpy(&_Value[0], Buffer, sizeof(_Value));
        }
        return sizeof(_Value);
    }

    T &operator[](uint8_t i)
    {
        return _Value[i];
    }

    T Get(uint8_t i) const
    {
        if (i < N)
        {
            return _Value[i];
        }
        else
        {
            return (T)0;
        }
    }

    void Set(uint8_t i, T v)
    {
        if (i < N)
        {
            _Value[i] = v;
        }
    }

    VarParamA<T, N> &operator=(VarParamA<T, N> &v)
    {
        return v;
    }

protected:
    T _Value[N];
};

#define VARDEF(_t, _n) typedef VarParamT<_t> JC_##_n;
VARDEF(float, Float);
VARDEF(int8_t, Int8);
VARDEF(uint8_t, UInt8);
VARDEF(int16_t, Int16);
VARDEF(uint16_t, Uint16);
VARDEF(int32_t, Int32);
VARDEF(uint32_t, Uint32);

class JC_Float16 : public JC_Float
{
public:
    JC_Float16(float Initial_Value = 0,
               Var_Key With_Key = Param_Key_None,
               const char *Name = NULL,
               Flags _Flags = Param_Flags_None) : JC_Float(Initial_Value, With_Key, Name, _Flags)
    {
    }

    JC_Float16(VarParam_Group *With_Group,
               Var_Key Index,
               float Initial_Value = 0,
               const char *Name = NULL,
               Flags _Flags = Param_Flags_None) : JC_Float(With_Group, Index, Initial_Value, Name, _Flags)
    {
    }

    virtual size_t Serialize(void *Buffer, size_t Size) const
    {
        uint16_t *ScaleValue = (uint16_t *)Buffer;

        if (Size >= sizeof(*ScaleValue))
        {
            *ScaleValue = _Value * 1024.0;
        }
        return sizeof(*ScaleValue);
    }

    virtual size_t Unserialize(void *Buffer, size_t Size)
    {
        uint16_t *ScaleValue = (uint16_t *)Buffer;

        if (Size >= sizeof(*ScaleValue))
        {
            _Value = (float)*ScaleValue / 1024.0;
            return sizeof(*ScaleValue);
        }
        return 0;
    }

    JC_Float16 &operator=(JC_Float16 &Value)
    {
        return Value;
    }

    JC_Float16 &operator=(float Value)
    {
        _Value = Value;
        return *this;
    }
};

#endif
