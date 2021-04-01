# Variáveis do CLI

| Nome          | Valor Padrão  | Descrição |
| ------------- | ------------- | ----------- |
| kP_Acc_AHRS   | 25          | Ganho Proporcional para correção da estimativa de Attitude |
| kI_Acc_AHRS   | 50          | Ganho Integral para correção da estimativa de Attitude |
| kP_Mag_AHRS   | 10          | Ganho Proporcional para correção da estimativa de direção do Yaw |
| kI_Mag_AHRS   | 0          | Ganho Integral para correção da estimativa de direção do Yaw  |
| AutoLaunch_AHRS_BankAngle   | 25          | Ângulo no AHRS para considerar que o AutoLaunch deve iniciar [Graus] |
| AutoLaunch_IMU_BankAngle   | 450          | Ângulo na IMU para considerar que o AutoLaunch deve iniciar [Graus*10] |
| AutoLaunch_IMU_Swing   | 100          | Velocidade da IMU para validar o AutoLaunch [DPS] |
| AutoLaunch_Trigger_Motor_Delay   | 1500          | Tempo para iniciar o motor após o status de lançado [MillisSegundos] |
| AutoLaunch_Elevator   | 18          | Inclinação no Pitch (Elevator) ao fazer o AutoLaunch [Graus]  |
| AutoLaunch_SpinUp   | 100          | Valor de incrimentação no Throttle para Aeros com rodas [uS]  |
| AutoLaunch_SpinUp_Time   | 300          | Tempo de incrimentação no Throttle para Aeros com rodas [MillisSegundos]  |
| AutoLaunch_MaxThrottle   | 1700          | Pulso maximo aplicado ao motor durante o AutoLaunch [uS]  |
| AutoLaunch_Exit   | 5000          | Cancela o AutoLaunch após o estouro desse tempo [MillisSegundos]  |
| AutoLaunch_Altitude   | 0          | Cancela o AutoLaunch após atingir essa altitude (Ignora o tempo acima) [Metros]  |
| Batt_Voltage_Factor   | 259489         | Fator de calibração para converter a leitura ADC em Tensão |
| Batt_Amps_Volt   | 620         | Fator de calibração para converter a leitura ADC em Corrente |
| Batt_Amps_OffSet   | 0         | Ajuste fino do valor da Corrente |
| CrashCheck_BankAngle   | 30         | Ângulo da IMU a ser considerado como Crash [Radianos*100] |
| CrashCheck_Time   | 2         | Estouro de tempo para validar o Crash [Segundos] |
| GimbalMinValue   | 1000         | Valor minimo do pulso a ser aplicado no Gimbal [uS] |
| GimbalMiddleValue   | 1500         | Valor médio do pulso a ser aplicado no Gimbal [uS] |
| GimbalMaxValue   | 2000         | Valor maximo do pulso a ser aplicado no Gimbal [uS] |
| Land_CheckAcc   | 3         | Valor da aceleração da IMU [Metros/Segundo^2] |
| Land_LPF   | 1         | "Valor da frequêcnia de corte da aceleração da IMU (Por Favor,não altere) [Hz] |
| ThrottleFactor   | 1         | Valor do ganho do Thottle para o mix de PID |
| AutoDisarm_Time   | 5         | Estouro de tempo para desarmar a controladora em nivel baixo de Throttle [Segundos] |
| AutoDisarm_Throttle_Min   | 1100         | Valor maximo do pulso do Throttle para iniciar a contagem do Auto-Desarm [uS] |
| AutoDisarm_YPR_Min   | 1450         | Valor minimo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarm [uS] |
| AutoDisarm_YPR_Max   | 1550         | Valor maximo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarm [uS] |
| AirPlane_Wheels   | 0         | 0 - Aeromodelo sem trem de pouso / 1 - Aeromodelo com trem de pouso (Apenas para o AutoLaunch) |
| GPS_Baud_Rate   | 0         | 0 - 9600KBPS / 1 - 19200KBPS / 2 - 38400KBPS / 3 - 57600KBPS / 4 - 115200KBPS |
| Navigation_Vel   | 400        | Velocidade maxima de navegação em modos de voo que utilizam o GPS [Centimetos/Segundo] |
| GPS_WP_Radius   | 2         | Raio do ponto para validar que o mesmo foi alcançado em modo WayPoint e RTH [Metros] |
| GPS_RTH_Land   | 10         | Em modo RTH,inicia o Land se o UAV estiver dentro do tamanho desse raio definido aqui,caso contrario,o UAV irá subir até a altitude definido em 'RTH Altitude' nas configurações basicas,voltar ao Home-Point,e fazer o Land [Metros] |
| GPS_TiltCompensation   | 20         | Parâmetro para compensar o rate de navegação em modo WayPoint e RTH |
| AirSpeed_Samples   | 15         | Número de amostras para calibrar o Tubo de Pitot |
| AirSpeed_Factor   | 1         | Escala dinâmica do Tubo de Pitot |