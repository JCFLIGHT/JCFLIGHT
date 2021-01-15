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

#include "FLIPMODE.h"
#include "Common/VARIABLES.h"
#include "FrameStatus/FRAMESTATUS.h"

//*******************************************************
//PROCEDIMENTO AUTOMATICO DE ACROBACIA
//*******************************************************

#define THROTTLE_INCREMENT 170  //VALOR DE INCREMENTAÇÃO NO THROTTLE ANTES DO INICIO DO FLIP
#define THROTTLE_DECREMENT 120  //VALOR DE DECREMENTAÇÃO COM O FLIP EM PROCESSO
#define FLIP_ROTATION_RATE 400  //VALOR PARA FORÇAR O PID A REALIZAR UM GIRO (400 GRAUS/S)
#define FLIP_RECOVERY_ANGLE 450 //VALOR DO CONTROLADOR DE ATTITUDE PARA DETECTAR QUE OS ANGULOS NORMALIZARAM (45 GRAUS)
#define GRAVITY_1G 512          //VALOR DE 1G DA GRAVIDADE NA IMU
#define FLIP_TIMEOUT 100        //ESTOURO DE TEMPO PARA CONSIDERAR QUE O FLIP ACONTECEU (1 SEGUNDO)

bool LockPitchAndRollRC = false;
bool LockProtection = false;
bool ApplyFlipRoll = false;
bool ApplyFlipPitch = false;
int8_t FlipDirection = 0;
uint8_t FlipState = STAGEWAITING;
uint8_t FlipTimer = 0;
int16_t SampleRoll = 1500;
int16_t SamplePitch = 1500;
int16_t FlipAngleValue = 0;
uint16_t RecoverThrottle = 0;
uint16_t ValueOfFlipToRoll = 0;
uint16_t ValueOfFlipToPitch = 0;

void FlipModeRun()
{
  if (GetFrameStateOfAirPlane())
  {
    if (SetFlightModes[FLIP_MODE])
    {
      TurnCoordinatorMode = true;
    }
    else
    {
      TurnCoordinatorMode = false;
    }
    return; //FAÇA UMA RAPIDA SAÍDA SE O MODO PLANE ESTIVER ATIVADO
  }

  if (!SetFlightModes[STABILIZE_MODE])
  {
    return; //FAÇA UMA RAPIDA SAIDA SE O MODO ACRO ESTIVER ATIVADO
  }

  if (!COMMAND_ARM_DISARM)
  {
    return; //FAÇA UMA RAPIDA SAIDA SE A CONTROLADORA ESTIVER DESARMADA
  }

  if (SetFlightModes[FLIP_MODE]) //MODO FLIP ATIVADO?SIM...
  {
    LockPitchAndRollRC = true; //BLOQUEIA OS VALORES DE PITCH E ROLL PARA O CONTROLADOR PID
    //RADIO READ ROLL
    if (SampleRoll > 1700 && !LockProtection)
    {
      FlipDirection = +1;
      ApplyFlipRoll = true;
    }
    else if (SampleRoll < 1300 && !LockProtection)
    {
      FlipDirection = -1;
      ApplyFlipRoll = true;
    }
    //RADIO READ PITCH
    if (SamplePitch > 1700 && !LockProtection)
    {
      FlipDirection = +1;
      ApplyFlipPitch = true;
    }
    else if (SamplePitch < 1300 && !LockProtection)
    {
      FlipDirection = -1;
      ApplyFlipPitch = true;
    }
  }

  if (FlipDirection == 0)
  {
    return; //NÃO FAZ O QUE ESTÁ ABAIXO SE FLIP DIRECTION FOR IGUAL A ZERO
  }

  //ESTADO DE MAQUINA
  switch (FlipState)
  {

  case STAGEONE:
    LockProtection = true;
    RecoverThrottle = RadioControllOutput[THROTTLE];
    FlipState = STAGETWO;
    break;

  case STAGETWO:
    RadioControllOutput[THROTTLE] += THROTTLE_INCREMENT;
    FlipState = STAGETHREE;
    break;

  case STAGETHREE:
    FlipAngleValue = FLIP_ROTATION_RATE * FlipDirection;
    if (ApplyFlipPitch)
    {
      FlipRollPitchAxis(true, PITCH);
    }
    if (ApplyFlipRoll)
    {
      FlipRollPitchAxis(true, ROLL);
    }
    RadioControllOutput[THROTTLE] -= THROTTLE_DECREMENT;
    FlipTimer = 0;
    FlipState = STAGERECOVER;
    break;

  case STAGERECOVER:
    if (FlipTimer >= FLIP_TIMEOUT)
    {
      if (((ATTITUDE.AngleOut[ROLL] < FLIP_RECOVERY_ANGLE) && (ValueOfFlipToRoll == GRAVITY_1G)) ||
          ((ATTITUDE.AngleOut[PITCH] < FLIP_RECOVERY_ANGLE) && (ValueOfFlipToPitch == GRAVITY_1G)))
      {
        ApplyFlipRoll = false;
        ApplyFlipPitch = false;
        FlipRollPitchAxis(false, ROLL);
        FlipRollPitchAxis(false, PITCH);
        FlipState = STAGEABANDON;
      }
    }
    else
    {
      FlipTimer++;
    }
    break;

  case STAGEABANDON:
    RadioControllOutput[THROTTLE] = RecoverThrottle;
    LockProtection = false;
    FlipTimer = 0;
    ValueOfFlipToRoll = 0;
    FlipDirection = 0;
    FlipState = STAGEWAITING;
    break;

  case STAGEWAITING:
    //AGUARDANDO PARA O INICIO DO MODO FLIP
    LockPitchAndRollRC = false;
    RecoverThrottle = 0;
    if (FlipDirection != 0)
    {
      FlipState = STAGEONE;
    }
    break;
  }
}

void FlipRollPitchAxis(bool _Do_Flip, uint8_t Axis)
{
  if (!_Do_Flip)
  {
    //RETORNA OU CONTINUA NO MODO STABILIZE E FAZ UMA RAPIDA SAÍDA
    ValueOfFlipToRoll = ValueOfFlipToPitch = 0;
    return;
  }
  //ROLL
  if (Axis == ROLL && _Do_Flip)
  {
    ValueOfFlipToRoll = GRAVITY_1G;
    return;
  }
  //PITCH
  if (Axis == PITCH && _Do_Flip)
  {
    ValueOfFlipToPitch = GRAVITY_1G;
    return;
  }
}