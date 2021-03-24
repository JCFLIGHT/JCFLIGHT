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

#pragma once

#include "VARPARAM.h"

/**************************
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
NÃO INDENTE ESSA EXTENSÃO
***************************/

enum VarParam_Enum
{
    Param_System_Version = 0,
    Param_Throttle_Min,
    Param_Throttle_Max,
    Param_Throttle_FailSafe,
    Param_Proportional_Roll,
    Param_Proportional_Pitch,
    Param_Proportional_Yaw,
    Param_Integral_Roll,
    Param_Integral_Pitch,
    Param_Integral_Yaw,
    Param_Derivative_Roll,
    Param_Derivative_Pitch,
    Param_Derivative_Yaw,
    Param_FF_Or_CD_Roll,
    Param_FF_Or_CD_Pitch,
    Param_FF_Or_CD_Yaw,
    Param_Proportional_AutoLevel,
    Param_Integral_AutoLevel,
    Param_Proportional_AltHold,
}; //255 KEYS NO MAXIMO

class ParametersClass
{
public:
    static const int16_t Actual_Format_Version = 10; //1.0
    JC_Int16 Format_Version;

    //RADIO
    JC_Int16 Throttle_Min;
    JC_Int16 Throttle_Max;
    JC_Int16 Throttle_FailSafe;

    //PID
    JC_UInt8 Proportional_Roll;
    JC_UInt8 Proportional_Pitch;
    JC_UInt8 Proportional_Yaw;
    JC_UInt8 Integral_Roll;
    JC_UInt8 Integral_Pitch;
    JC_UInt8 Integral_Yaw;
    JC_UInt8 Derivative_Roll;
    JC_UInt8 Derivative_Pitch;
    JC_UInt8 Derivative_Yaw;
    JC_UInt8 FF_Or_CD_Roll;
    JC_UInt8 FF_Or_CD_Pitch;
    JC_UInt8 FF_Or_CD_Yaw;
    JC_UInt8 Proportional_AutoLevel;
    JC_UInt8 Integral_AutoLevel;
    JC_UInt8 Proportional_AltHold;

    uint8_t SafeArea;

    ParametersClass() : //VARIAVEL               VALOR PADRÃO                 KEY                              NOME
                        Format_Version           (Actual_Format_Version,      Param_System_Version,            ProgramMemoryString("SYS_VER")),
                        Throttle_Min             (1000,                       Param_Throttle_Min,              ProgramMemoryString("THR_MIN")),
                        Throttle_Max             (2000,                       Param_Throttle_Max,              ProgramMemoryString("THR_MAX")),
                        Throttle_FailSafe        (970,                        Param_Throttle_FailSafe,         ProgramMemoryString("THR_FAILSAFE")),
                        Proportional_Roll        (40,                         Param_Proportional_Roll,         ProgramMemoryString("P_ROLL")),
                        Proportional_Pitch       (40,                         Param_Proportional_Pitch,        ProgramMemoryString("P_PITCH")),
                        Proportional_Yaw         (85,                         Param_Proportional_Yaw,          ProgramMemoryString("P_YAW")),
                        Integral_Roll            (30,                         Param_Integral_Roll,             ProgramMemoryString("I_ROLL")),
                        Integral_Pitch           (30,                         Param_Integral_Pitch,            ProgramMemoryString("I_PITCH")),
                        Integral_Yaw             (45,                         Param_Integral_Yaw,              ProgramMemoryString("I_YAW")),
                        Derivative_Roll          (23,                         Param_Derivative_Roll,           ProgramMemoryString("D_ROLL")),
                        Derivative_Pitch         (23,                         Param_Derivative_Pitch,          ProgramMemoryString("D_PITCH")),
                        Derivative_Yaw           (0,                          Param_Derivative_Yaw,            ProgramMemoryString("D_YAW")),
                        FF_Or_CD_Roll            (60,                         Param_FF_Or_CD_Roll,             ProgramMemoryString("FF_CD_ROLL")),
                        FF_Or_CD_Pitch           (60,                         Param_FF_Or_CD_Pitch,            ProgramMemoryString("FF_CD_PITCH")),
                        FF_Or_CD_Yaw             (60,                         Param_FF_Or_CD_Yaw,              ProgramMemoryString("FF_CD_YAW")),
                        Proportional_AutoLevel   (20,                         Param_Proportional_AutoLevel,    ProgramMemoryString("P_AL")),
                        Integral_AutoLevel       (15,                         Param_Integral_AutoLevel,        ProgramMemoryString("I_AL")),
                        Proportional_AltHold     (50,                         Param_Proportional_AltHold,      ProgramMemoryString("P_AH")),
                        
                        SafeArea(0) //ISSO SERVE PRA VOCÊ NÃO SE PREOCUPAR COM A VIRGULA FINAL
    {
    }
    void Initialization();
};

//#define VARPARAM_TEST

#ifdef VARPARAM_TEST
static ParametersClass Parameters;
#endif

void ParametersClass::Initialization() //PRIMEIRA INSTRUÇÃO DE MAQUINA A SER CARREGADA NO SETUP
{
#ifdef VARPARAM_TEST
    if (!Parameters.Format_Version.Load() || Parameters.Format_Version != ParametersClass::Actual_Format_Version)
    {
        LOG("Primeira linkagem,limpando a EEPROM...");
        VarParam::Erase_All();
        Parameters.Format_Version.Set_And_Save(ParametersClass::Actual_Format_Version);
        LOG("Ok...Parametros reconfigurados com valores de fabrica");
    }
    else
    {
        VarParam::Load_All();
    }
#endif
}