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

#include "AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "ParamsToGCS/SETFLIGHTMODES.h"
#include "BAR/BAR.h"
#include "FailSafe/FAILSAFE.h"
#include "FrameStatus/FRAMESTATUS.h"
#include "Common/ENUM.h"
#include "Common/RCDEFINES.h"
#include "RadioControl/DECODE.h"
#include "BitArray/BITARRAY.h"

//***********************************************************
//CONFIGURAÇÃO DAS CHAVES AUXILIARES PARA OS MODOS DE VOO
//***********************************************************

AUXFLIGHTCLASS AUXFLIGHT;

//VARIAVEIS PARA OBTÉR O VALOR EM uS QUE ESTÁ CONFIGURADO OS MODOS
bool AltitudeHoldControlAux,
    GPSHoldControlAux,
    RTHControlAux,
    SimpleControlAux,
    AcroControlAux,
    AttackControlAux,
    AutoFlipControlAux,
    WayPointControlAux,
    ArmDisarmControlAux,
    AutoLandControlAux,
    ParachuteControlAux;

//VARIAVEIS DE CARREGAMENTO DA EEPROM
uint8_t GPSHoldConfig,
    RTHConfig,
    SimpleConfig,
    GimbalConfig,
    AltitudeHoldConfig,
    AcroConfig,
    AttackConfig,
    AutoFlipConfig,
    WayPointConfig,
    FlightMode,
    ArmDisarmConfig,
    AutoLandConfig,
    ParachuteConfig;

uint8_t Channel_Low[MAX_AUX_CHANNELS];
uint8_t Channel_Middle[MAX_AUX_CHANNELS];
uint8_t Channel_High[MAX_AUX_CHANNELS];
uint8_t Channel_Levels_Count = 0;

void AUXFLIGHTCLASS::Initialization(void)
{
  SimpleConfig = STORAGEMANAGER.Read_8Bits(SIMPLE_ADDR);         //CHAVE AUX ATRIBUIDA PARA O MODO SIMPLES
  AltitudeHoldConfig = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR); //CHAVE AUX ATRIBUIDA PARA O MODO ALT-HOLD
  GPSHoldConfig = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);      //CHAVE AUX ATRIBUIDA PARA O MODO GPS-HOLD
  RTHConfig = STORAGEMANAGER.Read_8Bits(RTH_ADDR);               //CHAVE AUX ATRIBUIDA PARA O MODO RTH
  AcroConfig = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);         //CHAVE AUX ATRIBUIDA PARA O MODO ACRO
  AttackConfig = STORAGEMANAGER.Read_8Bits(ATACK_ADDR);          //CHAVE AUX ATRIBUIDA PARA O MODO ATTACK
  ParachuteConfig = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR);   //CONFIGURAÇÃO DO PARACHUTE
  AutoFlipConfig = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);     //CHAVE AUX ATRIBUIDA PARA O MODO AUTO-FLIP
  GimbalConfig = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);         //CANAL AUX ATRIBUIDO PARA O CONTROLE DO GIMBAL
  SetPlatformType(STORAGEMANAGER.Read_8Bits(FRAME_TYPE_ADDR));    //TIPO DE FRAME SELECIONADO
  ArmDisarmConfig = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);   //CHAVE ATRIBUIDA AO ARMDISARM VIA CHAVE AUX
  WayPointConfig = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR);  //CHAVE ATRIBUIDA AO MODO WAYPOINT
  AutoLandConfig = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);     //CHAVE ATRIBUIDA AO AUTO LAND
}

bool GetFlightModeState(uint8_t _Channel)
{

  if (_Channel == 0) //NO GCS O BOX EM "NENHUM" É IGUAL A ZERO
  {
    return false;
  }

  uint8_t ConfiguredChannel = 0;

  for (uint8_t IndexCount = 0; IndexCount <= (MAX_AUX_CHANNELS * 3); IndexCount++)
  {
    if (IndexCount == _Channel)
    {
      ConfiguredChannel = IndexCount / 3;
      //VERIFICA AS CASAS DECIMAIS
      if (((float)IndexCount / 3 >= (ConfiguredChannel + .1f)) &&
          ((float)IndexCount / 3 <= (ConfiguredChannel + .9f)))
      {
        ConfiguredChannel += 1;
      }
      break;
    }
  }

  if (Channel_Levels_Count < MAX_AUX_CHANNELS)
  {

    if (Channel_Levels_Count == 0)
    {
      Channel_Low[Channel_Levels_Count] = 1;
      Channel_Middle[Channel_Levels_Count] = 2;
      Channel_High[Channel_Levels_Count] = 3;
    }
    else
    {
      Channel_Low[Channel_Levels_Count] = Channel_Low[Channel_Levels_Count - 1] + 3;
      Channel_Middle[Channel_Levels_Count] = Channel_Middle[Channel_Levels_Count - 1] + 3;
      Channel_High[Channel_Levels_Count] = Channel_High[Channel_Levels_Count - 1] + 3;
    }

    Channel_Levels_Count++;
    return false; //NÃO ATUALIZA NENHUM MODO DE VOO QUANDO A SOMA ESTIVER EM PROCESSAMENTO
  }

  if (_Channel == Channel_Low[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + 3) < MIN_STICKS_PULSE + FLIGHT_MODE_PULSE_OFF_SET;
  }
  else if (_Channel == Channel_Middle[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + 3) > (MIDDLE_STICKS_PULSE - FLIGHT_MODE_PULSE_OFF_SET) &&
           DECODE.GetRxChannelOutput(ConfiguredChannel + 3) < (MIDDLE_STICKS_PULSE + FLIGHT_MODE_PULSE_OFF_SET);
  }
  else if (_Channel == Channel_High[ConfiguredChannel - 1])
  {
    return DECODE.GetRxChannelOutput(ConfiguredChannel + 3) > MAX_STICKS_PULSE - FLIGHT_MODE_PULSE_OFF_SET;
  }

  return false;
}

void AUXFLIGHTCLASS::FlightModesAuxSelect(void)
{
  SetFlightModeToGCS();

  if (SystemInFailSafe())
  {
    return;
  }

  ArmDisarmControlAux = GetFlightModeState(ArmDisarmConfig);
  AcroControlAux = GetFlightModeState(AcroConfig);
  AltitudeHoldControlAux = GetFlightModeState(AltitudeHoldConfig);
  RTHControlAux = GetFlightModeState(RTHConfig);
  WayPointControlAux = GetFlightModeState(WayPointConfig);
  SimpleControlAux = GetFlightModeState(SimpleConfig);
  GPSHoldControlAux = GetFlightModeState(GPSHoldConfig);
  AutoLandControlAux = GetFlightModeState(AutoLandConfig);
  AttackControlAux = GetFlightModeState(AttackConfig);
  AutoFlipControlAux = GetFlightModeState(AutoFlipConfig);
  ParachuteControlAux = GetFlightModeState(ParachuteConfig);

  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(STABILIZE_MODE, !AcroControlAux);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(ALTITUDE_HOLD_MODE, AltitudeHoldControlAux);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(RTH_MODE, RTHControlAux);
  ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(WAYPOINT_MODE, WayPointControlAux);

  if (GetMultirotorEnabled())
  {
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(SIMPLE_MODE, SimpleControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(POS_HOLD_MODE, GPSHoldControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(LAND_MODE, AutoLandControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(ATTACK_MODE, AttackControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(FLIP_MODE, AutoFlipControlAux);
  }
  else if (GetAirPlaneEnabled())
  {
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(MANUAL_MODE, SimpleControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(CIRCLE_MODE, GPSHoldControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(CRUISE_MODE, AutoLandControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(LAUNCH_MODE, AttackControlAux);
    ENABLE_DISABLE_THIS_FLIGHT_MODE_WITH_DEPENDENCY(TURN_MODE, AutoFlipControlAux);
  }
}

void AUXFLIGHTCLASS::Update(void)
{
  AUXFLIGHT.FlightModesAuxSelect();
}