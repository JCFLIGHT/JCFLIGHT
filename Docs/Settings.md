# Variáveis do CLI

| Nome | Valor Padrão | Min | Max | Descrição |
| ------------- | ------------- | --- | --- | ----------- |
| AirPlane_Wheels | 0 | 0 | 1 | 0 - Sem trem de pouso / 1 - Com trem de pouso (Apenas para o Auto-Launch) |
| AirSpeedAutoCalScale | 0 | 0 | 1 | `0 - Desativado / 1 - Ativado` ~ Calibração automática em voo da escala do Tubo de Pitot.Um novo valor será armazenado no parâmetro ´AirSpeed Fator´ a cada 2 minutos. |
| AirSpeed_Samples | 15 | 0 | 255 | Número de amostras para calibrar o Tubo de Pitot |
| Arm_Time_Safety | 2 | 0 | 255 | Tempo seguro para armar com os sticks em posição de armamento [Segundos] |
| AutoDisarm_Throttle_Min | 1100 | 800 | 1500 | Valor maximo do Throttle tolerado para iniciar a contagem do Auto-Desarmamento [uS] |
| AutoDisarm_Time | 5 | 0 | 255 | Estouro de tempo para desarmar a controladora em nivel baixo de Throttle [Segundos] |
| AutoDisarm_YPR_Max | 1550 | 800 | 2200 | Valor maximo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS] |
| AutoDisarm_YPR_Min | 1450 | 800 | 1500 | Valor minimo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS] |
| AutoLaunch_AHRS_BankAngle | 25 | 0 | 255 | Ângulo no AHRS para considerar que o Auto-Launch deve iniciar [Graus] |
| AutoLaunch_Altitude | 0 | 0 | 255 | Cancela o Auto-Launch após atingir essa altitude (O tempo acima será ignorado) [Metros] |
| AutoLaunch_Elevator | 18 | 0 | 100 | Inclinação no Pitch (Elevator) ao fazer o Auto-Launch [Graus] |
| AutoLaunch_Exit | 5000 | 0 | 30000 | Cancela o Auto-Launch após o estouro desse tempo [MillisSegundos] |
| AutoLaunch_IMU_BankAngle | -450 | -1000 | 1000 | Ângulo na IMU para considerar que o Auto-Launch deve iniciar [Graus*10] |
| AutoLaunch_MaxThrottle | 1700 | 1000 | 2000 | Valor do Throttle aplicado ao motor durante o Auto-Launch [uS] |
| AutoLaunch_SpinUp | 100 | 0 | 2000 | Valor de incrimentação no Throttle para Aeros com rodas [uS] |
| AutoLaunch_SpinUp_Time | 300 | 0 | 5000 | Tempo de incrimentação no Throttle para Aeros com rodas [MillisSegundos] |
| AutoLaunch_Trigger_Motor_Delay | 1500 | 0 | 10000 | Tempo para iniciar o motor após o status de lançado [MillisSegundos] |
| AutoLaunch_Velocity_Thresh | 3 | 0 | 20 | Velocidade da IMU ou GPS para validar o Auto-Launch [Metros/Segundo] |
| Compass_Cal_Timer | 60 | 0 | 120 | Tempo maximo de calibração do Compass [Segundos] |
| CrashCheck_BankAngle | 3 | 0 | 20 | Valor da aceleração da IMU [Metros/Segundo^2] |
| CrashCheck_Timer | 2 | 0 | 255 | Estouro de tempo para validar o Crash [Segundos] |
| Disarm_Time_Safety | 2 | 0 | 255 | Tempo seguro para desarmar com os sticks em posição de desarmamento [Segundos] |
| GPS_Baud_Rate | 4 | 0 | 4 | 0 - 9600KBPS / 1 - 19200KBPS / 2 - 38400KBPS / 3 - 57600KBPS / 4 - 115200KBPS |
| GPS_RTH_Land_Radius | 10 | 0 | 255 | Em modo RTH,inicia o Land se o UAV estiver dentro do tamanho desse raio definido aqui,caso contrario,o UAV irá subir até a altitude definido em 'RTH Altitude' nas configurações basicas,voltar ao Home-Point,e fazer o Land [Metros] |
| GPS_TiltCompensation | 20 | 0 | 100 | Valor para compensar o rate de navegação em modo WayPoint e RTH (Multirotores apenas) |
| GPS_WP_Radius | 2 | 0 | 255 | Raio do ponto para validar que o mesmo foi alcançado em modo WayPoint e RTH [Metros] |
| GimbalMaxValue | 2000 | 800 | 2200 | Valor maximo do pulso a ser aplicado no Gimbal [uS] |
| GimbalMinValue | 1000 | 800 | 2200 | Valor minimo do pulso a ser aplicado no Gimbal [uS] |
| Navigation_Vel | 400 | 0 | 400 | Velocidade maxima de navegação em modos de voo que utilizam o GPS [Centimetos/Segundo] |
| ThrottleMixGain | 1.0 | 0 | 1 | Valor de ganho do Throttle para o mixer do PID |
| kI_Acc_AHRS | 50 | 0 | 255 | Ganho Integral para correção da estimativa de Attitude |
| kI_Mag_AHRS | 0 | 0 | 255 | Ganho Integral para correção da estimativa de direção do Yaw |
| kP_Acc_AHRS | 25 | 0 | 255 | Ganho Proporcional para correção da estimativa de Attitude |
| kP_Mag_AHRS | 10 | 0 | 255 | Ganho Proporcional para correção da estimativa de direção do Yaw |

> Esse arquivo é gerado automaticamente,não o edite manualmente!