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
    Param_Proportional_Yaw
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

    uint8_t SafeArea;

    ParametersClass() : //VARIAVEL         VALOR PADRÃO                 KEY                              NOME
                        Format_Version     (Actual_Format_Version,      Param_System_Version,            ProgramMemoryString("SYS_VER")),
                        Throttle_Min       (1000,                       Param_Throttle_Min,              ProgramMemoryString("THR_MIN")),
                        Throttle_Max       (2000,                       Param_Throttle_Max,              ProgramMemoryString("THR_MAX")),
                        Throttle_FailSafe  (970,                        Param_Throttle_FailSafe,         ProgramMemoryString("THR_FAILSAFE")),
                        Proportional_Roll  (40,                         Param_Proportional_Roll,         ProgramMemoryString("P_ROLL")),
                        Proportional_Pitch (40,                         Param_Proportional_Pitch,        ProgramMemoryString("P_PITCH")),
                        Proportional_Yaw   (85,                         Param_Proportional_Yaw,          ProgramMemoryString("P_YAW")),

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
        PRINTF.SendToConsole(ProgramMemoryString("Primeira linkagem,limpando a EEPROM...\n"));
        VarParam::Erase_All();
        Parameters.Format_Version.Set_And_Save(ParametersClass::Actual_Format_Version);
        PRINTF.SendToConsole(ProgramMemoryString("Ok...Parametros reconfigurados\n"));
    }
    else
    {
        VarParam::Load_All();
    }
#endif
}