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

#include "PIDPARAMS.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"
#include "Common/ENUM.h"

PID_Terms_Struct GET_SET[SIZE_OF_PID_PARAMS];

void Load_All_PID_Params(void)
{
    /*
    if (GetMultirotorEnabled()) //MULTIROTORES
    {
        //PID ROLL
        GET_SET[PID_ROLL].kP = 40;
        GET_SET[PID_ROLL].kI = 30;
        GET_SET[PID_ROLL].kD = 23;
        GET_SET[PID_ROLL].kFF = 60;

        //PID PITCH
        GET_SET[PID_PITCH].kP = 40;
        GET_SET[PID_PITCH].kI = 30;
        GET_SET[PID_PITCH].kD = 23;
        GET_SET[PID_PITCH].kFF = 60;

        //PID YAW
        GET_SET[PID_YAW].kP = 85;
        GET_SET[PID_YAW].kI = 45;
        GET_SET[PID_YAW].kD = 0;
        GET_SET[PID_YAW].kFF = 60;

        //PI DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].kP = 20;
        GET_SET[PI_AUTO_LEVEL].kI = 15;
    }
    else if (GetAirPlaneEnabled()) //AEROS
    {
        //PID ROLL
        GET_SET[PID_ROLL].kP = 5;
        GET_SET[PID_ROLL].kI = 7;
        GET_SET[PID_ROLL].kD = 0;
        GET_SET[PID_ROLL].kFF = 50;

        //PID PITCH
        GET_SET[PID_PITCH].kP = 5;
        GET_SET[PID_PITCH].kI = 7;
        GET_SET[PID_PITCH].kD = 0;
        GET_SET[PID_PITCH].kFF = 50;

        //PID YAW
        GET_SET[PID_YAW].kP = 6;
        GET_SET[PID_YAW].kI = 10;
        GET_SET[PID_YAW].kD = 0;
        GET_SET[PID_YAW].kFF = 60;

        //PID DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].kP = 20;
        GET_SET[PI_AUTO_LEVEL].kI = 5;
    }
*/

    //PID PARA A RETENÇÃO DE ALTITUDE (ALTITUDE-HOLD)
    //GET_SET[PID_VELOCITY_Z].kP = 50;
    //GET_SET[PID_VELOCITY_Z].kI = 20;
    //GET_SET[PID_VELOCITY_Z].kD = 16;

    //PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    //GET_SET[PID_GPS_POSITION].kP = 100;
    //GET_SET[PID_GPS_POSITION].kI = 90;
    //GET_SET[PID_GPS_POSITION].kD = 0;

    //APENAS TEMPORARIAMENTE
    uint8_t ActualFrameType = STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR);
    if (ActualFrameType == QUAD_X ||
        ActualFrameType == HEXA_X ||
        ActualFrameType == HEXA_I ||
        ActualFrameType == ZMR_250 ||
        ActualFrameType == TBS)
    {
        //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
        GET_SET[PID_GPS_POSITION_RATE].kP = 70;
        GET_SET[PID_GPS_POSITION_RATE].kI = 20;
        GET_SET[PID_GPS_POSITION_RATE].kD = 20;

        //RATE DE NAVEGAÇÃO (RTH E MISSÃO)
        GET_SET[PID_GPS_NAVIGATION_RATE].kP = 25;
        GET_SET[PID_GPS_NAVIGATION_RATE].kI = 33;
        GET_SET[PID_GPS_NAVIGATION_RATE].kD = 83;
    }
    else //AEROS
    {
        //RATE DO PID DA RETENÇÃO DE POSIÇÃO (CRUISE OU CIRCULOS)
        GET_SET[PID_GPS_POSITION_RATE].kP = 75;
        GET_SET[PID_GPS_POSITION_RATE].kI = 5;
        GET_SET[PID_GPS_POSITION_RATE].kD = 8;

        //RATE DE NAVEGAÇÃO DO CONTROLE DO LEME
        GET_SET[PID_GPS_NAVIGATION_RATE].kP = 30;
        GET_SET[PID_GPS_NAVIGATION_RATE].kI = 2;
        GET_SET[PID_GPS_NAVIGATION_RATE].kD = 0;
    }

    /*
    //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPS_POSITION_RATE].kP = 70;
    GET_SET[PID_GPS_POSITION_RATE].kI = 20;
    GET_SET[PID_GPS_POSITION_RATE].kD = 20;

    //RATE DE NAVEGAÇÃO (RTH E MISSÃO)
    GET_SET[PID_GPS_NAVIGATION_RATE].kP = 25;
    GET_SET[PID_GPS_NAVIGATION_RATE].kI = 33;
    GET_SET[PID_GPS_NAVIGATION_RATE].kD = 83;
*/

    //HEADING-HOLD RATE
    //GET_SET[P_YAW_RATE].kP = 60;
    //GET_SET[P_YAW_RATE_LIMIT].MaxValue = 90;

    GET_SET[PID_ROLL].kP = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    GET_SET[PID_ROLL].kI = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    GET_SET[PID_ROLL].kD = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    GET_SET[PID_ROLL].kFF = STORAGEMANAGER.Read_8Bits(FF_OR_CD_ROLL_ADDR);

    GET_SET[PID_PITCH].kP = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    GET_SET[PID_PITCH].kI = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    GET_SET[PID_PITCH].kD = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    GET_SET[PID_PITCH].kFF = STORAGEMANAGER.Read_8Bits(FF_OR_CD_PITCH_ADDR);

    GET_SET[PID_YAW].kP = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    GET_SET[PID_YAW].kI = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    GET_SET[PID_YAW].kD = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    GET_SET[PID_YAW].kFF = STORAGEMANAGER.Read_8Bits(FF_OR_CD_YAW_ADDR);

    GET_SET[PI_AUTO_LEVEL].kP = STORAGEMANAGER.Read_8Bits(KP_AUTOLEVEL_ADDR);
    GET_SET[PI_AUTO_LEVEL].kI = STORAGEMANAGER.Read_8Bits(KI_AUTOLEVEL_ADDR);

    GET_SET[PID_ALTITUDE].kP = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);
    GET_SET[PID_ALTITUDE].kI = STORAGEMANAGER.Read_8Bits(KI_ALTITUDE_ADDR);
    GET_SET[PID_ALTITUDE].kD = STORAGEMANAGER.Read_8Bits(KD_ALTITUDE_ADDR);

    GET_SET[PID_VELOCITY_Z].kP = STORAGEMANAGER.Read_8Bits(KP_VEL_Z_ADDR);
    GET_SET[PID_VELOCITY_Z].kI = STORAGEMANAGER.Read_8Bits(KI_VEL_Z_ADDR);
    GET_SET[PID_VELOCITY_Z].kD = STORAGEMANAGER.Read_8Bits(KD_VEL_Z_ADDR);

    GET_SET[PID_GPS_POSITION].kP = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    GET_SET[PID_GPS_POSITION].kI = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);

    GET_SET[P_YAW_RATE].kP = STORAGEMANAGER.Read_8Bits(KP_HEADING_HOLD_ADDR);
    GET_SET[P_YAW_RATE_LIMIT].MaxValue = STORAGEMANAGER.Read_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR);

    GET_SET[ROLL_BANK_MAX].MaxValue = STORAGEMANAGER.Read_8Bits(ROLL_BANK_ADDR);
    GET_SET[PITCH_BANK_MIN].MaxValue = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MIN_ADDR);
    GET_SET[PITCH_BANK_MAX].MaxValue = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MAX_ADDR);
    GET_SET[ATTACK_BANK_MAX].MaxValue = STORAGEMANAGER.Read_8Bits(ATTACK_BANK_ADDR);
    GET_SET[GPS_BANK_MAX].MaxValue = STORAGEMANAGER.Read_8Bits(GPS_BANK_ADDR);

    GET_SET[MAX_PITCH_LEVEL].MaxValue = STORAGEMANAGER.Read_8Bits(MAX_PITCH_LEVEL_ADDR);
    GET_SET[MAX_ROLL_LEVEL].MaxValue = STORAGEMANAGER.Read_8Bits(MAX_ROLL_LEVEL_ADDR);
}

void UpdateValuesOfPID()
{
    if (GET_SET[PID_UPDATED].State)
    {
        return;
    }

    uint8_t ActualFrameType = STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR);
    static uint8_t PreviousFrameType = ActualFrameType;

    //RECONFIGURA TODO O PID SE O FRAME SELECIONADO FOR DIFERENTE DO ANTERIOR
    //ISSO É NECESSARIO APENAS PARA A TROCA DO PERFIL DE MULTIROTOR PARA AERO E VICE-VERSA
    if (PreviousFrameType != ActualFrameType)
    {
        for (uint8_t IndexCount = 0; IndexCount < 5; IndexCount++)
        {
            if (ActualFrameType == QUAD_X ||
                ActualFrameType == HEXA_X ||
                ActualFrameType == HEXA_I ||
                ActualFrameType == ZMR_250 ||
                ActualFrameType == TBS)
            {
                //PITCH
                STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 23);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 60);

                //ROLL
                STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 23);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 60);

                //YAW
                STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 85);
                STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 45);
                STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

                //AUTO-NÍVEL
                STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 15);

                //ALTITUDE-HOLD
                STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, 3);
                STORAGEMANAGER.Write_8Bits(KI_ALTITUDE_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KD_ALTITUDE_ADDR, 10);

                //GPS-HOLD
                STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, 100);
                STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, 90);

                //ÂNGULOS DE NAVEGAÇÃO
                STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);

                //RTH ALTITUDE
                STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, 10);
            }
            else if (ActualFrameType == AIR_PLANE ||
                     ActualFrameType == FIXED_WING ||
                     ActualFrameType == AIR_PLANE_VTAIL)
            {
                //PITCH
                STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 7);
                STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 0);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 50);

                //ROLL
                STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 7);
                STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 0);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 50);

                //YAW
                STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 6);
                STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 10);
                STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

                //AUTO-NÍVEL
                STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 5);

                //ALTITUDE-HOLD
                STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(KI_ALTITUDE_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KD_ALTITUDE_ADDR, 10);

                //AUTO-THROTTLE
                STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, 150);
                STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, 70);

                //ÂNGULOS DE NAVEGAÇÃO
                STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 45);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 25);
                STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 75);
                STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);

                //RTH ALTITUDE
                STORAGEMANAGER.Write_8Bits(RTH_ALTITUDE_ADDR, 40);
            }

            //VELOCIDADE Z
            STORAGEMANAGER.Write_8Bits(KP_VEL_Z_ADDR, 50);
            STORAGEMANAGER.Write_8Bits(KI_VEL_Z_ADDR, 20);
            STORAGEMANAGER.Write_8Bits(KD_VEL_Z_ADDR, 16);

            //HEADING-HOLD
            STORAGEMANAGER.Write_8Bits(KP_HEADING_HOLD_ADDR, 60);
            STORAGEMANAGER.Write_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR, 90);
        }
        PreviousFrameType = ActualFrameType;
    }

    Load_All_PID_Params();

    GET_SET[PID_UPDATED].State = true;
}
