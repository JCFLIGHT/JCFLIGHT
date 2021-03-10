#pragma once

#include "VARPARAM.h"

class VARTEST
{
public:
    void TestVar();

#define VAR1_DEFAULT_VAL 12
#define VAR2_DEFAULT_VAL 23545
#define VAR3_DEFAULT_VAL 56.85f
#define VAR4_DEFAULT_VAL 486347

    VARTEST(VarParam::Var_Key Key) : Var_Group(Key, ProgmemString("VAR_TEST")),
                                     VariableTest1(&Var_Group, 0, VAR1_DEFAULT_VAL, ProgmemString("VT1")),
                                     VariableTest2(&Var_Group, 1, VAR2_DEFAULT_VAL, ProgmemString("VT2")),
                                     VariableTest3(&Var_Group, 2, VAR3_DEFAULT_VAL, ProgmemString("VT3")),
                                     VariableTest4(&Var_Group, 3, VAR4_DEFAULT_VAL, ProgmemString("VT4"))
    {
    }

protected:
    VarParam_Group Var_Group;
    JC_Int8 VariableTest1;
    JC_Int16 VariableTest2;
    JC_Float VariableTest3;
    JC_Int32 VariableTest4;
};

class VARTEST2
{
public:
    void TestVar();

#define VAR1_DEFAULT_VAL2 70
#define VAR2_DEFAULT_VAL2 10203
#define VAR3_DEFAULT_VAL2 104.24f
#define VAR4_DEFAULT_VAL2 506040

    VARTEST2(VarParam::Var_Key Key) : Var_Group(Key, ProgmemString("VAR_TEST2")),
                                      VariableTest1(&Var_Group, 0, VAR1_DEFAULT_VAL2, ProgmemString("VT1")),
                                      VariableTest2(&Var_Group, 1, VAR2_DEFAULT_VAL2, ProgmemString("VT2")),
                                      VariableTest3(&Var_Group, 2, VAR3_DEFAULT_VAL2, ProgmemString("VT3")),
                                      VariableTest4(&Var_Group, 3, VAR4_DEFAULT_VAL2, ProgmemString("VT4"))
    {
    }

protected:
    VarParam_Group Var_Group;
    JC_Int8 VariableTest1;
    JC_Int16 VariableTest2;
    JC_Float VariableTest3;
    JC_Int32 VariableTest4;
};