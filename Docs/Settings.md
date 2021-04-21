# Variáveis do CLI

| Nome          | Valor Padrão  | Descrição |
| ------------- | ------------- | ----------- |
| kP_Acc_AHRS   | 25          | Ganho Proporcional para correção da estimativa de Attitude |
| kI_Acc_AHRS   | 50          | Ganho Integral para correção da estimativa de Attitude |
| kP_Mag_AHRS   | 10          | Ganho Proporcional para correção da estimativa de direção do Yaw |
| kI_Mag_AHRS   | 0          | Ganho Integral para correção da estimativa de direção do Yaw  |
| AutoLaunch_AHRS_BankAngle   | 25          | Ângulo no AHRS para considerar que o Auto-Launch deve iniciar [Graus] |
| AutoLaunch_IMU_BankAngle   | -450          | Ângulo na IMU para considerar que o Auto-Launch deve iniciar [Graus*10] |
| AutoLaunch_Velocity_Thresh   | 3          | Velocidade da IMU ou GPS para validar o Auto-Launch [Metros/Segundo] |
| AutoLaunch_Trigger_Motor_Delay   | 1500          | Tempo para iniciar o motor após o status de lançado [MillisSegundos] |
| AutoLaunch_Elevator   | 18          | Inclinação no Pitch (Elevator) ao fazer o Auto-Launch [Graus]  |
| AutoLaunch_SpinUp   | 100          | Valor de incrimentação no Throttle para Aeros com rodas [uS]  |
| AutoLaunch_SpinUp_Time   | 300          | Tempo de incrimentação no Throttle para Aeros com rodas [MillisSegundos]  |
| AutoLaunch_MaxThrottle   | 1700          | Valor do Throttle aplicado ao motor durante o Auto-Launch [uS]  |
| AutoLaunch_Exit   | 5000          | Cancela o Auto-Launch após o estouro desse tempo [MillisSegundos]  |
| AutoLaunch_Altitude   | 0          | Cancela o Auto-Launch após atingir essa altitude (O tempo acima será ignorado) [Metros]  |
| Batt_Voltage_Factor   | 259.489         | Fator de calibração para converter a leitura ADC em Tensão |
| Batt_Amps_Per_Volt   | 62.0         | Fator de calibração para converter a leitura ADC em Corrente |
| Batt_Amps_OffSet   | 0         | Ajuste fino do valor da Corrente |
| CrashCheck_BankAngle   | 30         | Ângulo da IMU a ser considerado como Crash [Radianos*100] |
| CrashCheck_Time   | 2         | Estouro de tempo para validar o Crash [Segundos] |
| GimbalMinValue   | 1000         | Valor minimo do pulso a ser aplicado no Gimbal [uS] |
| GimbalMiddleValue   | 1500         | Valor médio do pulso a ser aplicado no Gimbal [uS] |
| GimbalMaxValue   | 2000         | Valor maximo do pulso a ser aplicado no Gimbal [uS] |
| Land_CheckAcc   | 3         | Valor da aceleração da IMU [Metros/Segundo^2] |
| Land_LPF   | 1         | Valor da frequêcnia de corte da aceleração da IMU (Recomendo não alterar) [Hz] |
| ThrottleFactor   | 1.0         | Valor do ganho do Throttle para o mixer do PID |
| AutoDisarm_Time   | 5         | Estouro de tempo para desarmar a controladora em nivel baixo de Throttle [Segundos] |
| AutoDisarm_Throttle_Min   | 1100         | Valor maximo do Throttle tolerado para iniciar a contagem do Auto-Desarmamento [uS] |
| AutoDisarm_YPR_Min   | 1450         | Valor minimo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS] |
| AutoDisarm_YPR_Max   | 1550         | Valor maximo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS] |
| AirPlane_Wheels   | 0         | 0 - Sem trem de pouso / 1 - Com trem de pouso (Apenas para o Auto-Launch) |
| GPS_Baud_Rate   | 4         | 0 - 9600KBPS / 1 - 19200KBPS / 2 - 38400KBPS / 3 - 57600KBPS / 4 - 115200KBPS |
| Navigation_Vel   | 400        | Velocidade maxima de navegação em modos de voo que utilizam o GPS [Centimetos/Segundo] |
| GPS_WP_Radius   | 2         | Raio do ponto para validar que o mesmo foi alcançado em modo WayPoint e RTH [Metros] |
| GPS_RTH_Land_Radius   | 10         | Em modo RTH,inicia o Land se o UAV estiver dentro do tamanho desse raio definido aqui,caso contrario,o UAV irá subir até a altitude definido em 'RTH Altitude' nas configurações basicas,voltar ao Home-Point,e fazer o Land [Metros] |
| GPS_TiltCompensation   | 20         | Valor para compensar o rate de navegação em modo WayPoint e RTH |
| AirSpeed_Samples   | 15         | Número de amostras para calibrar o Tubo de Pitot |
| AirSpeed_Factor   | 1.9936         | Escala dinâmica para o Tubo de Pitot |
| Arm_Time_Safety   | 2         | Tempo seguro para armar com os sticks em posição de armamento [Segundos] |
| Disarm_Time_Safety   | 2         | Tempo seguro para desarmar com os sticks em posição de desarmamento [Segundos] |
| Compass_Cal_Timer   | 60        | Tempo maximo de calibração do Compass [Segundos] |
| AutoPilotMode   | 1        | `0 - Piloto automático do tipo Attitude` ~ O usuário irá controlar o ângulo de inclinação do Roll e Pitch.Quando os sticks do Roll e Pitch do rádio são manipulados pelo piloto manual,o sistema de navegação inercial será ignorado,e os comandos irão passar diretamente no modo By-Pass.Assim que os sticks Roll e Pitch do rádio estiverem centralizados (1500uS),o sistema de navegação inercial irá retomar o piloto automático e manter a posição do multirotor. `1 - Piloto automático do tipo Cruise` ~ O sistema de navegação inercial estará sempre ativo e o usuário irá controlar a velocidade de navegação do Roll e Pitch,e o piloto automático irá calcular um novo controle de Attitude,assim mantendo a posição do multirotor constantemente. |
| AirSpeedAutoCalScale   | 0        | `0 - Desativado` ~ `1 - Ativado` |