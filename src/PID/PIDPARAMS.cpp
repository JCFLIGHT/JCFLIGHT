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
#include "IOMCU/IOMCU.h"
#include "BAR/BAR.h"
#include "Common/ENUM.h"

PID_Terms_Struct GET_SET[SIZE_OF_PID_PARAMS];

void LoadPID()
{
    /*
    if (GetFrameStateOfMultirotor()) //MULTIROTORES
    {
        //PID ROLL
        GET_SET[PID_ROLL].Proportional = 40;
        GET_SET[PID_ROLL].Integral = 30;
        GET_SET[PID_ROLL].Derivative = 23;
        GET_SET[PID_ROLL].FeedForward = 60;

        //PID PITCH
        GET_SET[PID_PITCH].Proportional = 40;
        GET_SET[PID_PITCH].Integral = 30;
        GET_SET[PID_PITCH].Derivative = 23;
        GET_SET[PID_PITCH].FeedForward = 60;

        //PID YAW
        GET_SET[PID_YAW].Proportional = 85;
        GET_SET[PID_YAW].Integral = 45;
        GET_SET[PID_YAW].Derivative = 0;
        GET_SET[PID_YAW].FeedForward = 60;

        //PI DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].Proportional = 20;
        GET_SET[PI_AUTO_LEVEL].Integral = 15;
    }
    else if (GetFrameStateOfAirPlane()) //AEROS
    {
        //PID ROLL
        GET_SET[PID_ROLL].Proportional = 5;
        GET_SET[PID_ROLL].Integral = 7;
        GET_SET[PID_ROLL].Derivative = 0;
        GET_SET[PID_ROLL].FeedForward = 50;

        //PID PITCH
        GET_SET[PID_PITCH].Proportional = 5;
        GET_SET[PID_PITCH].Integral = 7;
        GET_SET[PID_PITCH].Derivative = 0;
        GET_SET[PID_PITCH].FeedForward = 50;

        //PID YAW
        GET_SET[PID_YAW].Proportional = 6;
        GET_SET[PID_YAW].Integral = 10;
        GET_SET[PID_YAW].Derivative = 0;
        GET_SET[PID_YAW].FeedForward = 60;

        //PID DO MODO AUTO-NIVEL
        GET_SET[PI_AUTO_LEVEL].Proportional = 20;
        GET_SET[PI_AUTO_LEVEL].Integral = 5;
    }
*/

    //PID PARA A RETENÇÃO DE ALTITUDE (ALTITUDE-HOLD)
    //GET_SET[PID_ALTITUDE].Proportional = 50;
    GET_SET[PID_ALTITUDE].Integral = 20;
    GET_SET[PID_ALTITUDE].Derivative = 16;

    //PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    //GET_SET[PID_GPS_POSITION].Proportional = 100;
    //GET_SET[PID_GPS_POSITION].Integral = 90;
    //GET_SET[PID_GPS_POSITION].Derivative = 0;

    //RATE DO PID DA RETENÇÃO DE POSIÇÃO (GPS-HOLD)
    GET_SET[PID_GPS_POSITION_RATE].Proportional = 70;
    GET_SET[PID_GPS_POSITION_RATE].Integral = 20;
    GET_SET[PID_GPS_POSITION_RATE].Derivative = 20;

    //RATE DE NAVEGAÇÃO (RTH E MISSÃO)
    GET_SET[PID_GPS_NAVIGATION_RATE].Proportional = 25;
    GET_SET[PID_GPS_NAVIGATION_RATE].Integral = 33;
    GET_SET[PID_GPS_NAVIGATION_RATE].Derivative = 83;

    //HEADING-HOLD RATE
    //GET_SET[P_YAW_RATE].Proportional = 60;
    //GET_SET[P_YAW_RATE_LIMIT].MinMaxValue = 90;
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
    //ISSO É NECESSARIO PARA A TROCA DO PERFIL DE MULTIROTOR PARA AERO E VICE-VERSA
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

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 60);

                //ROLL
                STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 23);

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 60);

                //YAW
                STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 85);
                STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 45);
                STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

                //AUTO-NÍVEL
                STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 15);

                //ÂNGULOS DE NAVEGAÇÃO
                STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 30);
                STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);
            }
            else if (ActualFrameType == AIR_PLANE ||
                     ActualFrameType == FIXED_WING ||
                     ActualFrameType == PLANE_VTAIL)
            {
                //PITCH
                STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, 7);
                STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, 0);

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_PITCH_ADDR, 50);

                //ROLL
                STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, 5);
                STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, 7);
                STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, 0);

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_ROLL_ADDR, 50);

                //YAW
                STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, 6);
                STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, 10);
                STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, 0);

                //FEED-FORWAD OU CONTROLE DERIVATIVO
                STORAGEMANAGER.Write_8Bits(FF_OR_CD_YAW_ADDR, 60);

                //AUTO-NÍVEL
                STORAGEMANAGER.Write_8Bits(KP_AUTOLEVEL_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(KI_AUTOLEVEL_ADDR, 5);

                //ÂNGULOS DE NAVEGAÇÃO
                STORAGEMANAGER.Write_8Bits(ROLL_BANK_ADDR, 45);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MIN_ADDR, 20);
                STORAGEMANAGER.Write_8Bits(PITCH_BANK_MAX_ADDR, 25);
                STORAGEMANAGER.Write_8Bits(ATTACK_BANK_ADDR, 40);
                STORAGEMANAGER.Write_8Bits(GPS_BANK_ADDR, 30);
            }

            //ALTITUDE-HOLD
            STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, 50);

            //GPS-HOLD
            STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, 100);
            STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, 90);

            //HEADING-HOLD
            STORAGEMANAGER.Write_8Bits(KP_HEADING_HOLD_ADDR, 60);
            STORAGEMANAGER.Write_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR, 90);
        }
        PreviousFrameType = ActualFrameType;
    }

    GET_SET[PID_ROLL].Proportional = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);
    GET_SET[PID_ROLL].Integral = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);
    GET_SET[PID_ROLL].Derivative = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);
    GET_SET[PID_ROLL].FeedForward = STORAGEMANAGER.Read_8Bits(FF_OR_CD_ROLL_ADDR);

    GET_SET[PID_PITCH].Proportional = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);
    GET_SET[PID_PITCH].Integral = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);
    GET_SET[PID_PITCH].Derivative = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);
    GET_SET[PID_PITCH].FeedForward = STORAGEMANAGER.Read_8Bits(FF_OR_CD_PITCH_ADDR);

    GET_SET[PID_YAW].Proportional = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);
    GET_SET[PID_YAW].Integral = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);
    GET_SET[PID_YAW].Derivative = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);
    GET_SET[PID_YAW].FeedForward = STORAGEMANAGER.Read_8Bits(FF_OR_CD_YAW_ADDR);

    GET_SET[PI_AUTO_LEVEL].Proportional = STORAGEMANAGER.Read_8Bits(KP_AUTOLEVEL_ADDR);
    GET_SET[PI_AUTO_LEVEL].Integral = STORAGEMANAGER.Read_8Bits(KI_AUTOLEVEL_ADDR);

    GET_SET[PID_ALTITUDE].Proportional = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);

    GET_SET[PID_GPS_POSITION].Proportional = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);
    GET_SET[PID_GPS_POSITION].Integral = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);

    GET_SET[P_YAW_RATE].Proportional = STORAGEMANAGER.Read_8Bits(KP_HEADING_HOLD_ADDR);
    GET_SET[P_YAW_RATE_LIMIT].MinMaxValue = STORAGEMANAGER.Read_8Bits(HEADING_HOLD_RATE_LIMIT_ADDR);

    GET_SET[ROLL_BANK_MAX].MinMaxValue = STORAGEMANAGER.Read_8Bits(ROLL_BANK_ADDR);
    GET_SET[PITCH_BANK_MIN].MinMaxValue = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MIN_ADDR);
    GET_SET[PITCH_BANK_MAX].MinMaxValue = STORAGEMANAGER.Read_8Bits(PITCH_BANK_MAX_ADDR);
    GET_SET[ATTACK_BANK_MAX].MinMaxValue = STORAGEMANAGER.Read_8Bits(ATTACK_BANK_ADDR);
    GET_SET[GPS_BANK_MAX].MinMaxValue = STORAGEMANAGER.Read_8Bits(GPS_BANK_ADDR);

    GET_SET[MAX_PITCH_LEVEL].MinMaxValue = STORAGEMANAGER.Read_8Bits(MAX_PITCH_LEVEL_ADDR);
    GET_SET[MAX_ROLL_LEVEL].MinMaxValue = STORAGEMANAGER.Read_8Bits(MAX_ROLL_LEVEL_ADDR);

    GET_SET[PID_UPDATED].State = true;
}
