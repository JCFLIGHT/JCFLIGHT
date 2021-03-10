#include "VARTEST.h"
#include "VERSION.h"
#include "FastSerial/PRINTF.h"

//APENAS TESTE,SERÁ REMOVIDO FUTURAMENTE

bool initt = false;

void VARTEST::TestVar()
{
    if (!initt)
    {
        CheckVersion();
        //VariableTest1.Set_And_Save(88);
        //VariableTest1.Load();
        initt = true;
    }

    DEBUG("VT1:%d VT2:%d VT3:%.2f VT4:%ld",
          VariableTest1.Get(),
          VariableTest2.Get(),
          VariableTest3.Get(),
          VariableTest4.Get());
}

void VARTEST2::TestVar()
{
    DEBUG("VT1:%d VT2:%d VT3:%.2f VT4:%ld",
          VariableTest1.Get(),
          VariableTest2.Get(),
          VariableTest3.Get(),
          VariableTest4.Get());
}